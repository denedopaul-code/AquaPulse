// AquaPulse - TANK NODE (Uno)
// Handles ultrasonic sensing, LCD display, fault detection, dry-run protection,
// and pump start/stop decisions. The PUMP NODE (Nano) only sees a command line.
//
// COMMAND PROTOCOL (one wire + shared GND):
//   Steady LOW = pump must stop. This is also the pulldown's resting state, so a
//   broken wire fails safe (pump off).
//   Toggling every HEARTBEAT_HALF_MS = pump may run. The pump node turns the
//   motor off if those edges stop, so a hung Uno cannot leave the pump on.
//
// WIRING TO PUMP NODE:
//   COMMAND_PIN (this board) -> Nano's COMMAND_PIN
//   10k resistor from the Nano pin to Nano GND (pulldown). No internal pull-up.
//   This board's GND        -> Nano's GND
//
// RECALIBRATION BUTTON:
//   RECAL_BUTTON_PIN -> momentary pushbutton -> GND
//   Hold while powering on, or hold for RECAL_HOLD_MS while running, to measure
//   a fresh empty-tank height. Otherwise the last CRC-checked height is loaded
//   so a reboot does not recalibrate against a partly full tank.

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <string.h>

// I2C LCD: SDA -> A4, SCL -> A5. Address is probed (0x27, then 0x3F).
LiquidCrystal_I2C* lcd = nullptr;

// ---------- Pin Definitions ----------
constexpr uint8_t TRIG_PIN         = 7;
constexpr uint8_t ECHO_PIN         = 6;
constexpr uint8_t WATER_SOURCE_PIN = A2;   // analog dry-run source sensor
constexpr uint8_t FAULT_LED_PIN    = 2;    // lit on sensor fault / dry-run alert
constexpr uint8_t BUZZER_PIN       = 5;
constexpr uint8_t COMMAND_PIN      = 4;    // toggled while the pump should run
constexpr uint8_t RECAL_BUTTON_PIN = 3;    // active LOW, uses internal pull-up

// ---------- EEPROM Calibration Storage ----------
// Layout: magic, version, float height, CRC-16 of (version + height bytes).
// Version 2 invalidates older "magic + raw float" records on purpose.
constexpr int EEPROM_MAGIC_ADDR    = 0;
constexpr int EEPROM_VERSION_ADDR  = 1;
constexpr int EEPROM_HEIGHT_ADDR   = 2;
constexpr int EEPROM_CRC_ADDR      = 6;
constexpr uint8_t EEPROM_MAGIC     = 0xA5;
constexpr uint8_t EEPROM_VERSION   = 2;

// ---------- Tank Dimensions & Thresholds ----------
constexpr float MIN_TANK_HEIGHT_CM  = 5.0f;
constexpr float MAX_TANK_HEIGHT_CM  = 400.0f;
constexpr float DEFAULT_TANK_HEIGHT = 10.0f;  // fallback if calibration fails (cm)
float TANK_HEIGHT      = DEFAULT_TANK_HEIGHT;
float EFFECTIVE_HEIGHT = 0.0f;
constexpr float SENSOR_OFFSET_RATIO = 0.05f;  // dead zone near the sensor
constexpr uint8_t SAMPLES = 5;
constexpr uint8_t MIN_SAMPLES_FOR_LEVEL = 3;

// ---------- Calibration ----------
constexpr unsigned long CALIBRATION_DURATION = 5000;
constexpr unsigned long CALIBRATION_SAMPLE_INTERVAL = 100;
constexpr unsigned long RECAL_HOLD_MS = 3000;
constexpr int CALIBRATION_MAX_READS = 32;

constexpr int PUMP_START_PERCENT   = 50;
constexpr int PUMP_STOP_PERCENT    = 70;
constexpr int PUMP_RELATCH_PERCENT = 50;

// ---------- Fault Handling ----------
constexpr unsigned long HARD_FAULT_MS = 2500;  // bad echoes this long -> sensor fault
constexpr unsigned long ECHO_TIMEOUT_US = 30000;

// Source sensor hysteresis around the old "water present" point of 300.
// Rising through WET counts as water; falling through DRY counts as empty.
constexpr int SOURCE_DRY_ADC = 250;
constexpr int SOURCE_WET_ADC = 300;

// ---------- Timing ----------
constexpr unsigned long SENSOR_INTERVAL     = 100;
constexpr unsigned long CONTROL_INTERVAL    = 500;
constexpr unsigned long FAULT_BEEP_INTERVAL = 400;
constexpr unsigned long HEARTBEAT_HALF_MS   = 250;  // must stay well under the pump watchdog

// ---------- Global State ----------
unsigned long lastSensorTime    = 0;
unsigned long lastControlTime   = 0;
unsigned long lastFaultBeepTime = 0;
unsigned long lastGoodReadingMs = 0;
unsigned long lastHeartbeatMs   = 0;
unsigned long recalHoldStartMs  = 0;
bool faultBeepState = false;

float distanceSamples[SAMPLES] = {0};
uint8_t sampleIndex = 0;
uint8_t sampleCount = 0;
float currentDistance = -1.0f;
bool hardFaultActive = false;

bool pumpCommanded = false;  // logical request; the pin itself toggles while this is true
bool heartbeatLevel = false;
bool isPumpRunning = false;
bool pumpLatched   = false;
bool sourceWaterAvailable = false;

uint8_t detectLcdAddress() {
  Wire.begin();
  const uint8_t candidates[] = {0x27, 0x3F};
  for (uint8_t i = 0; i < 2; i++) {
    Wire.beginTransmission(candidates[i]);
    if (Wire.endTransmission() == 0) {
      Serial.print("LCD found at 0x");
      Serial.println(candidates[i], HEX);
      return candidates[i];
    }
  }
  Serial.println("LCD not detected; trying 0x27 anyway.");
  return 0x27;
}

bool heightPlausible(float height) {
  return height >= MIN_TANK_HEIGHT_CM && height <= MAX_TANK_HEIGHT_CM;
}

void applyTankHeight(float height) {
  TANK_HEIGHT = height;
  EFFECTIVE_HEIGHT = TANK_HEIGHT * (1.0f - SENSOR_OFFSET_RATIO);
  if (EFFECTIVE_HEIGHT < 1.0f) {
    EFFECTIVE_HEIGHT = 1.0f;
  }
}

uint16_t crc16Update(uint16_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t bit = 0; bit < 8; bit++) {
    if (crc & 1) {
      crc = (crc >> 1) ^ 0xA001;
    } else {
      crc >>= 1;
    }
  }
  return crc;
}

uint16_t calibrationCrc(float height) {
  uint16_t crc = 0xFFFF;
  crc = crc16Update(crc, EEPROM_VERSION);
  uint8_t raw[sizeof(float)];
  memcpy(raw, &height, sizeof(float));
  for (uint8_t i = 0; i < sizeof(raw); i++) {
    crc = crc16Update(crc, raw[i]);
  }
  return crc;
}

bool loadCalibration() {
  if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC) return false;
  if (EEPROM.read(EEPROM_VERSION_ADDR) != EEPROM_VERSION) return false;

  float height = 0.0f;
  uint16_t storedCrc = 0;
  EEPROM.get(EEPROM_HEIGHT_ADDR, height);
  EEPROM.get(EEPROM_CRC_ADDR, storedCrc);

  if (storedCrc != calibrationCrc(height)) return false;
  if (!heightPlausible(height)) return false;

  applyTankHeight(height);
  return true;
}

void saveCalibration(float height) {
  uint16_t crc = calibrationCrc(height);
  EEPROM.update(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
  EEPROM.update(EEPROM_VERSION_ADDR, EEPROM_VERSION);
  EEPROM.put(EEPROM_HEIGHT_ADDR, height);
  EEPROM.put(EEPROM_CRC_ADDR, crc);
}

float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return -1.0f;
  return duration * 0.0343f / 2.0f;
}

void serviceHeartbeat() {
  if (!pumpCommanded) {
    heartbeatLevel = false;
    digitalWrite(COMMAND_PIN, LOW);
    return;
  }

  unsigned long now = millis();
  if (now - lastHeartbeatMs < HEARTBEAT_HALF_MS) return;
  lastHeartbeatMs = now;
  heartbeatLevel = !heartbeatLevel;
  digitalWrite(COMMAND_PIN, heartbeatLevel ? HIGH : LOW);
}

void waitForRecalRelease() {
  while (digitalRead(RECAL_BUTTON_PIN) == LOW) {
    serviceHeartbeat();
    delay(10);
  }
  recalHoldStartMs = 0;
}

void setup() {
  Serial.begin(9600);

  pinMode(COMMAND_PIN, OUTPUT);
  digitalWrite(COMMAND_PIN, LOW);

  pinMode(RECAL_BUTTON_PIN, INPUT_PULLUP);

  uint8_t lcdAddress = detectLcdAddress();
  lcd = new LiquidCrystal_I2C(lcdAddress, 16, 2);
  lcd->init();
  lcd->backlight();
  lcd->print("AQUAPULSE V1");
  delay(2000);
  lcd->clear();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(WATER_SOURCE_PIN, INPUT);
  pinMode(FAULT_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(FAULT_LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  bool forceRecal = (digitalRead(RECAL_BUTTON_PIN) == LOW);
  if (!forceRecal && loadCalibration()) {
    lcd->clear();
    lcd->setCursor(0, 0);
    lcd->print("Loaded saved");
    lcd->setCursor(0, 1);
    lcd->print("calibration");
    delay(1500);
    lcd->clear();

    Serial.print("Loaded stored TANK_HEIGHT: ");
    Serial.println(TANK_HEIGHT);
  } else {
    calibrateTankHeight();
  }

  waitForRecalRelease();
  lastGoodReadingMs = millis();
}

void calibrateTankHeight() {
  lcd->clear();
  lcd->setCursor(0, 0);
  lcd->print("Calibrating...");
  lcd->setCursor(0, 1);
  lcd->print("KEEP TANK EMPTY");
  tone(BUZZER_PIN, 1500, 150);

  float reads[CALIBRATION_MAX_READS];
  int count = 0;
  unsigned long calibrationStart = millis();

  while (millis() - calibrationStart < CALIBRATION_DURATION && count < CALIBRATION_MAX_READS) {
    serviceHeartbeat();
    float reading = readDistanceCm();
    if (reading > 2.0f && reading <= MAX_TANK_HEIGHT_CM) {
      reads[count++] = reading;
    }
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }

  float measured = -1.0f;
  if (count >= 5) {
    for (int i = 1; i < count; i++) {
      float value = reads[i];
      int j = i - 1;
      while (j >= 0 && reads[j] > value) {
        reads[j + 1] = reads[j];
        j--;
      }
      reads[j + 1] = value;
    }
    int drop = count / 5;
    if (drop < 1) drop = 1;
    float sum = 0.0f;
    int kept = 0;
    for (int i = drop; i < count - drop; i++) {
      sum += reads[i];
      kept++;
    }
    if (kept > 0) measured = sum / kept;
  }

  lcd->clear();
  if (heightPlausible(measured)) {
    applyTankHeight(measured);
    Serial.print(TANK_HEIGHT);
    Serial.println(" cm.");
    lcd->setCursor(0, 0);
    lcd->print("Calib complete");
    lcd->setCursor(0, 1);
    lcd->print("Height: ");
    lcd->print(TANK_HEIGHT, 1);
    lcd->print("cm");
    tone(BUZZER_PIN, 2500, 300);
  } else {
    applyTankHeight(DEFAULT_TANK_HEIGHT);
    lcd->setCursor(0, 0);
    lcd->print("Calib FAILED");
    lcd->setCursor(0, 1);
    lcd->print("Using default");
    tone(BUZZER_PIN, 400, 800);
    Serial.println("Calibration failed; stored default height.");
  }
  delay(2000);
  lcd->clear();

  // Persist even a default, so the next reboot does not measure a full tank.
  saveCalibration(TANK_HEIGHT);
}

void loop() {
  serviceHeartbeat();

  unsigned long currentMillis = millis();
  if (currentMillis - lastSensorTime >= SENSOR_INTERVAL) {
    lastSensorTime = currentMillis;
    readUltrasonic();
  }

  if (currentMillis - lastControlTime >= CONTROL_INTERVAL) {
    lastControlTime = currentMillis;
    processControlLogic();
  }

  pollRecalButton();
}

void clearSamples() {
  for (uint8_t i = 0; i < SAMPLES; i++) distanceSamples[i] = 0;
  sampleIndex = 0;
  sampleCount = 0;
}

float medianDistance() {
  float sorted[SAMPLES];
  for (uint8_t i = 0; i < sampleCount; i++) sorted[i] = distanceSamples[i];

  for (uint8_t i = 1; i < sampleCount; i++) {
    float value = sorted[i];
    int j = (int)i - 1;
    while (j >= 0 && sorted[j] > value) {
      sorted[j + 1] = sorted[j];
      j--;
    }
    sorted[j + 1] = value;
  }
  return sorted[sampleCount / 2];
}

void readUltrasonic() {
  float rawDistance = readDistanceCm();
  bool readingOk = rawDistance > 2.0f && rawDistance < (TANK_HEIGHT * 1.15f);

  if (readingOk) {
    lastGoodReadingMs = millis();
    distanceSamples[sampleIndex] = rawDistance;
    sampleIndex = (sampleIndex + 1) % SAMPLES;
    if (sampleCount < SAMPLES) sampleCount++;
    if (sampleCount >= MIN_SAMPLES_FOR_LEVEL) {
      currentDistance = medianDistance();
      hardFaultActive = false;
    }
  } else if (millis() - lastGoodReadingMs >= HARD_FAULT_MS) {
    hardFaultActive = true;
    currentDistance = -1.0f;
    clearSamples();
  }
}

void processControlLogic() {
  if (hardFaultActive) {
    setPumpCommand(false, "CRITICAL FAILURE: Persistent sensor fault.");
    digitalWrite(FAULT_LED_PIN, HIGH);
    soundFaultAlarm();
    lcd->setCursor(0, 0);
    lcd->print("SENSOR FAULT!   ");
    lcd->setCursor(0, 1);
    lcd->print("Pump Halted     ");
    return;
  }

  // No level yet (or the buffer was just cleared). Do not invent a fill percent.
  if (currentDistance < 0.0f) {
    setPumpCommand(false, "Status: No valid level reading. Pump OFF.");
    digitalWrite(FAULT_LED_PIN, LOW);
    noTone(BUZZER_PIN);
    faultBeepState = false;
    lcd->setCursor(0, 0);
    lcd->print("NO READING      ");
    lcd->setCursor(0, 1);
    lcd->print("Pump: OFF       ");
    return;
  }

  float waterHeight = TANK_HEIGHT - currentDistance;
  int fillPercentage = constrain((int)((waterHeight / EFFECTIVE_HEIGHT) * 100.0f), 0, 100);

  int sourceRaw = analogRead(WATER_SOURCE_PIN);
  if (sourceRaw >= SOURCE_WET_ADC) sourceWaterAvailable = true;
  else if (sourceRaw <= SOURCE_DRY_ADC) sourceWaterAvailable = false;

  bool aboutToStart = !isPumpRunning && !pumpLatched && fillPercentage <= PUMP_START_PERCENT;
  if (!sourceWaterAvailable && (isPumpRunning || aboutToStart)) {
    setPumpCommand(false, "ALERT: Dry-run prevented! Source empty.");
    digitalWrite(FAULT_LED_PIN, HIGH);
    soundFaultAlarm();
    lcd->setCursor(0, 0);
    lcd->print("DRY-RUN ALERT   ");
    lcd->setCursor(0, 1);
    lcd->print("NO SOURCE WATER ");
    return;
  }

  digitalWrite(FAULT_LED_PIN, LOW);
  noTone(BUZZER_PIN);
  faultBeepState = false;

  if (!sourceWaterAvailable) {
    setPumpCommand(false, "Status: Source empty. Pump stays off.");
    lcd->setCursor(0, 0);
    lcd->print("SOURCE EMPTY    ");
    lcd->setCursor(0, 1);
    lcd->print("Pump: OFF       ");
    return;
  }

  if (pumpLatched && fillPercentage <= PUMP_RELATCH_PERCENT) {
    pumpLatched = false;
    Serial.println("Status: Latch cleared, pump re-armed.");
  }

  if (isPumpRunning && fillPercentage >= PUMP_STOP_PERCENT) {
    setPumpCommand(false, "Status: Tank full. Pump OFF, latched.");
    pumpLatched = true;
  }

  if (!isPumpRunning && !pumpLatched && fillPercentage <= PUMP_START_PERCENT) {
    setPumpCommand(true, "Status: Tank low. Pump ON.");
  }

  updateDisplay(fillPercentage);
}

void soundFaultAlarm() {
  unsigned long now = millis();
  if (now - lastFaultBeepTime >= FAULT_BEEP_INTERVAL) {
    lastFaultBeepTime = now;
    faultBeepState = !faultBeepState;
    if (faultBeepState) {
      tone(BUZZER_PIN, 2000);
    } else {
      noTone(BUZZER_PIN);
    }
  }
}

void updateDisplay(int fillPercentage) {
  char line0[17];
  char line1[17];

  snprintf(line0, sizeof(line0), "Level: %-3d%%     ", fillPercentage);

  if (isPumpRunning) {
    snprintf(line1, sizeof(line1), "Pump: ON        ");
  } else if (pumpLatched) {
    snprintf(line1, sizeof(line1), "Pump: LATCHED   ");
  } else {
    snprintf(line1, sizeof(line1), "Pump: OFF       ");
  }

  lcd->setCursor(0, 0);
  lcd->print(line0);
  lcd->setCursor(0, 1);
  lcd->print(line1);
}

void pollRecalButton() {
  if (digitalRead(RECAL_BUTTON_PIN) != LOW) {
    recalHoldStartMs = 0;
    return;
  }
  if (recalHoldStartMs == 0) {
    recalHoldStartMs = millis();
    return;
  }
  if (millis() - recalHoldStartMs < RECAL_HOLD_MS) return;

  setPumpCommand(false, "Recalibration requested. Pump OFF.");
  pumpLatched = false;
  calibrateTankHeight();
  clearSamples();
  currentDistance = -1.0f;
  hardFaultActive = false;
  lastGoodReadingMs = millis();
  waitForRecalRelease();
}

// Records the decision. While ON, serviceHeartbeat() toggles COMMAND_PIN.
// While OFF, the pin is held LOW. Prints only when the decision changes.
void setPumpCommand(bool turnOn, const char* reason) {
  if (turnOn == pumpCommanded) return;
  pumpCommanded = turnOn;
  isPumpRunning = turnOn;
  if (!turnOn) {
    heartbeatLevel = false;
    digitalWrite(COMMAND_PIN, LOW);
  } else {
    lastHeartbeatMs = 0;
  }
  Serial.println(reason);
}
