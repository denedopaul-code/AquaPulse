# AquaPulse
AquaPulse keeps a tank filled from a separate water source. Two Arduino boards split the job:
- The **tank node** (Arduino Uno) measures the water level, shows it on an LCD, and decides when the pump may run.
- The **pump node** (Arduino Nano) only switches the relay. It does not make its own level or dry-run decisions.
The tank node is in charge of safety. The pump node turns the motor **off** unless it keeps seeing a live signal from the tank node.
## How a fill cycle works
1. Hold the recalibration button while you power the Uno **with the tank empty**. It measures the empty distance and saves it.
2. Later boots load that saved height, so a reboot while the tank is partly full does not pretend the tank is empty.
3. The pump starts when the level is at or below **50%**.
4. The pump stops at **70%** and stays off until the level falls back to **50%** (so it does not chatter on and off at the top).
5. If the source sensor says the inlet is dry, the pump will not start. If it goes dry while the pump is running, the pump stops and the buzzer sounds.
6. If the ultrasonic sensor fails for about **2.5 seconds**, the pump stops and the LCD shows `SENSOR FAULT!`.
## Command wire
One signal wire goes from Uno pin **4** to Nano pin **2**, plus a shared ground.
| Tank node is... | Signal on the wire | Pump |
| --- | --- | --- |
| Asking the pump to run | Toggles about every 250 ms | On |
| Asking the pump to stop | Steady LOW | Off |
| Reset, unplugged, or hung | Wire stops toggling (pulldown reads LOW, or a stuck level times out) | Off |
The Nano turns the relay off if it sees no change on that wire for **800 ms**. A steady HIGH left behind by a crashed Uno does **not** keep the motor running.
Put a **10k resistor** from Nano pin 2 to Nano GND. Do not turn on the Nano pin's internal pull-up as well; it would fight the resistor.
## Parts
- Arduino Uno (tank node)
- Arduino Nano (pump node)
- HC-SR04 ultrasonic sensor, mounted above the tank looking down
- 16x2 I2C LCD (address `0x27` or `0x3F`)
- Analog source-water sensor on Uno **A2** (wet reads about 300 or higher)
- Momentary button from Uno pin **3** to GND
- Fault LED on Uno pin **2**, buzzer on Uno pin **5**
- Relay module on Nano pin **9** (active-LOW: LOW energizes the coil)
- Pump-on LED on Nano pin **6**
- 10k pulldown on the Nano command pin
Most cheap Arduino relay boards are active-LOW, and this sketch assumes that. **If the motor runs as soon as the Nano powers up, your board is active-HIGH.** In that case swap `RELAY_ON_STATE` and `RELAY_OFF_STATE` in the pump sketch before you trust it. Check this with the pump unplugged from mains, watching the relay LED only.
## Wiring
### Tank node (Uno)
| Part | Uno pin |
| --- | --- |
| HC-SR04 TRIG | 7 |
| HC-SR04 ECHO | 6 |
| HC-SR04 VCC / GND | 5V / GND |
| Source sensor signal | A2 |
| Fault LED | 2 (other leg to GND through a resistor, about 220 ohm) |
| Buzzer | 5 |
| Command wire to the Nano | 4 |
| Recalibration button | 3 (other leg to GND) |
| LCD SDA | A4 |
| LCD SCL | A5 |
| LCD VCC / GND | 5V / GND |
### Pump node (Nano)
| Part | Nano pin |
| --- | --- |
| Command wire from the Uno | 2, with 10k to GND |
| Relay IN | 9 |
| Pump-on LED | 6 (other leg to GND through a resistor) |
| Relay VCC / GND | 5V / GND, or the relay board's JD-VCC if it has a separate coil supply |
Uno GND and Nano GND must be connected together. The relay's motor contacts are a separate circuit. Match the relay's voltage and current to the pump, and do not work on the pump wiring while it is connected to mains.
```mermaid
flowchart LR
  sensor[HC-SR04]
  uno[Tank_Uno]
  nano[Pump_Nano]
  relay[Relay]
  motor[Pump]
  sensor --> uno
  uno -->|"toggle = run"| nano
  nano --> relay --> motor
```
## Upload the sketches
Install the [Arduino IDE](https://www.arduino.cc/en/software). Then install one library:
1. **Sketch → Include Library → Manage Libraries**
2. Search for **LiquidCrystal I2C**
3. Install **LiquidCrystal I2C by Frank de Brabander**
`Wire.h` and `EEPROM.h` are already part of the IDE. You do not install those.
Each `.ino` file has to stay inside a folder of the same name. Those folders are:
- `AquaPulse_Tank_Node/AquaPulse_Tank_Node.ino` → board: **Arduino Uno**
- `AquaPulse_Pump_Node/AquaPulse_Pump_Node.ino` → board: **Arduino Nano**
For each one: **File → Open**, select the board and the port under **Tools**, then click Upload. Open **Tools → Serial Monitor** at **9600** baud if you want the status lines.
The copies under `Documents\Arduino\` are shortcuts to these same files, so the Arduino sketchbook still finds them.
### First run after this version
Saved calibration from older sketches is ignored, because the stored height now has a version and a checksum. On the first boot, either:
- power the Uno while holding the button, with the tank **empty**, or
- hold the button for about 3 seconds while it is already running (the pump is forced off first)
The LCD should show `KEEP TANK EMPTY`, then a height in centimeters. A bad reading falls back to 10 cm and still gets saved, so the next reboot will not measure a full tank by mistake. If the height looks wrong, empty the tank and calibrate again.
## Tuning
The numbers worth knowing are at the top of the tank sketch:
| Name | Default | Meaning |
| --- | --- | --- |
| `PUMP_START_PERCENT` | 50 | Start at or below this level |
| `PUMP_STOP_PERCENT` | 70 | Stop at or above this level |
| `SOURCE_WET_ADC` | 300 | Analog reading that means "source has water" |
| `SOURCE_DRY_ADC` | 250 | Analog reading that means "source is dry" |
| `MIN_TANK_HEIGHT_CM` / `MAX_TANK_HEIGHT_CM` | 5 / 400 | Calibration outside this range is rejected |
Readings between 250 and 300 leave the previous wet/dry decision alone, so a noisy sensor does not chatter the buzzer.
## What you should see
| LCD line | Meaning |
| --- | --- |
| `Level: 42%` and `Pump: ON` | Filling |
| `Pump: LATCHED` | Hit 70% and waiting to fall back to 50% |
| `NO READING` | Ultrasonic buffer is not ready yet; pump stays off |
| `SOURCE EMPTY` | Inlet is dry, but the tank is not asking for water, so the buzzer stays quiet |
| `DRY-RUN ALERT` | It wanted to run, or was running, and the inlet is dry |
| `SENSOR FAULT!` | No believable echo for about 2.5 seconds |
## Project layout
```text
