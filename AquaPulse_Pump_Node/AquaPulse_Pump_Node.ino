// AquaPulse - PUMP NODE (Nano)
// Dumb actuator: the TANK NODE (Uno) owns every safety decision. This board
// only turns the relay on while the command line is toggling, and forces the
// motor off if those edges stop.
//
// WIRING TO TANK NODE:
//   COMMAND_PIN (this board, INPUT) <- Uno's COMMAND_PIN (output)
//   10k resistor from COMMAND_PIN to this board's GND (pulldown). Do not also
//   enable the pin's internal pull-up; the two would fight.
//   This board's GND -> Uno's GND (required, shared reference for the signal)
//
// COMMAND PROTOCOL:
//   Steady LOW  = stop (also the state of an open wire, thanks to the pulldown)
//   Toggling    = run (tank flips the line about every 250 ms)
//   Steady HIGH = tank hung while asserting run -> this node times out and stops
//
// Relay module is ACTIVE-LOW: a LOW signal energizes the coil and turns the motor ON.
// Boot therefore drives the relay pin HIGH before it becomes an output, so the
// coil cannot glitch on.

constexpr uint8_t COMMAND_PIN = 2;
constexpr uint8_t RELAY_PIN   = 9;
constexpr uint8_t PUMP_ON_LED = 6;

constexpr uint8_t RELAY_ON_STATE  = LOW;
constexpr uint8_t RELAY_OFF_STATE = HIGH;

// Tank half-period is 250 ms, so a healthy "run" signal edges at least every 250 ms.
// 800 ms is about three missed edges: long enough to ignore one blocked sensor read,
// short enough that a hung tank cannot leave the motor running.
constexpr unsigned long WATCHDOG_MS = 800;
constexpr unsigned long BOOT_IGNORE_MS = 500;

bool lastLevel = false;
bool seenEdge = false;
bool pumpOn = false;
unsigned long lastEdgeMs = 0;
unsigned long bootMs = 0;

void setup() {
  Serial.begin(9600);

  pinMode(COMMAND_PIN, INPUT);

  // Active-low modules turn ON if the pin comes up LOW. Park the output HIGH
  // before enabling it so the coil stays off through reset.
  digitalWrite(RELAY_PIN, RELAY_OFF_STATE);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF_STATE);

  digitalWrite(PUMP_ON_LED, LOW);
  pinMode(PUMP_ON_LED, OUTPUT);
  digitalWrite(PUMP_ON_LED, LOW);

  bootMs = millis();
  lastLevel = digitalRead(COMMAND_PIN);

  Serial.println("AquaPulse pump node ready.");
}

void loop() {
  bool level = digitalRead(COMMAND_PIN);
  unsigned long now = millis();

  if (level != lastLevel) {
    lastLevel = level;
    // Ignore the tank node's own boot edges; a real run request keeps toggling.
    if (now - bootMs >= BOOT_IGNORE_MS) {
      lastEdgeMs = now;
      seenEdge = true;
    }
  }

  bool wantOn = seenEdge && (now - lastEdgeMs) < WATCHDOG_MS;
  if (wantOn != pumpOn) {
    pumpOn = wantOn;
    applyPumpState(pumpOn);
  }
}

void applyPumpState(bool turnOn) {
  digitalWrite(RELAY_PIN, turnOn ? RELAY_ON_STATE : RELAY_OFF_STATE);
  digitalWrite(PUMP_ON_LED, turnOn ? HIGH : LOW);
  Serial.println(turnOn ? "Pump node: relay ON" : "Pump node: relay OFF");
}
