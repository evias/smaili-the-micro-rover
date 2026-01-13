# micro-rover

This sketch contains the firmware source code for a ESP32 DEVKIT V1 board wired
together with listed components. Note that for practicability, the ESP32 board
is plugged on a `10:30` prototyping breadboard and attached to the bottom plate
of the rover platform.

## 3D Printing

This project includes multiples 3D-printed components. Some modifications were
necessary to fit and plug the custom motors and servo motor.

### Original models

| 3D Model | Attribution | Description |
| --- | --- | --- |
| [Rover platform](https://www.printables.com/model/145064-rover-platform) | [@Nahuel_114346](https://www.printables.com/@Nahuel_114346) | A small rover platform about 20cm x 20cm |
| [Hinged Case for Ultrasonic Sensor](https://www.printables.com/model/1338788-hinged-case-for-ultrasonic-sensor-hc-sr04) | [@ConnorPritch_3339490](https://www.printables.com/@ConnorPritch_3339490) | A case for the HC-SR04 with clean cable case. |
| [Ultrasonic sensor HC-SR04 mount on servo](https://www.printables.com/model/84061-ultrasonic-sensor-hc-sr04-mount-on-servo) | [@Balex_174247](https://www.printables.com/model/84061-ultrasonic-sensor-hc-sr04-mount-on-servo) | A funny-faced case for HC-SR04, compatible with above internal case. |

### Modified parts

- `DCMotorSpindleAdapter`: Uses a custom adapter to put on the DC motors pins, so that they can be plugged in the original spindle component.
- `CoverWithCableCut`: Uses a cable cut on the back-part of the top cover of the rover.
- `CogModifiedForDC`: Uses a custom cog (wheels) that are pluggable with the specific DC motors.
- `OuterTrackFrameModForDC`: Uses a custom outer-track components that needs less room inside the cog.
- `ShelfModifiedForServo`: Uses a custom front-shelf that fits with a servo motor opening in the top-middle.

## Hardware

| Component | Device | Marketplace |
| --- | --- | --- |
| 2x Miniature motor | Micro Motor DC 3V-6V 8000RPM | [AliExpress](https://es.aliexpress.com/item/1005008162727677.html) |
| 1x Servo Motor | Micro Servo `SG-90` | [AliExpress](https://es.aliexpress.com/item/4000126425823.html) |
| 1x Ultrasonic Sensor | `HC-SR04` | [AliExpress](https://es.aliexpress.com/item/1005006368132158.html) |

## Wiring

| Component | Pin 1 | Pin 2 | Pin 3 | Pin 4 |
| --- | --- | --- | --- | --- |
| ESP32 | GND → - | 3v3 → + | | |
| DC Motors RHT (A) | + → B1 | - → B2 | | |
| DC Motors LFT (B) | + → H1 | - → H2 | | |
| Servo Motor | Y → H11 | R → H10 | B → H9 | |
| Ultrasonic Sensor | B → H7 | W → H6 | G → H5 | R → H4

### Right DC Motor Wiring with Transistor, Resistor and Diode

- 1x Miniature motor
- 1x Transistor (`2N2222`)
- 1x 1kΩ Resistor
- 1x Flyback Diode (`1N4001`)

| From | To |
|------|----|
| ESP32 **D2** | **Resistor 1** (one leg) |
| **Resistor 1** (other leg) | **Transistor 1 Base** (middle leg) |
| **Transistor 1 Emitter** (left leg) | **ESP32 GND** |
| **Transistor 1 Collector** (right leg) | **Motor A negative wire** (black or -) |
| **Motor A positive wire** (red or +) | **External Power Supply (+)** |
| **Diode 1 striped end** | **Motor A positive wire** |
| **Diode 1 plain end** | **Motor A negative wire** |

### Left DC Motor Wiring with Transistor, Resistor and Diode

- 1x Miniature motor
- 1x Transistor (`2N2222`)
- 1x 1kΩ Resistor
- 1x Flyback Diode (`1N4001`)

| From | To |
|------|----|
| ESP32 **D4** | **Resistor 2** (one leg) |
| **Resistor 2** (other leg) | **Transistor 2 Base** (middle leg) |
| **Transistor 2 Emitter** (left leg) | **ESP32 GND** |
| **Transistor 2 Collector** (right leg) | **Motor B negative wire** (black or -) |
| **Motor B positive wire** (red or +) | **External Power Supply (+)** |
| **Diode 2 striped end** | **Motor B positive wire** |
| **Diode 2 plain end** | **Motor B negative wire** |

### Servo Motor

| From (Breadboard) | Servo Wire | Connect To |
|-------------------|------------|------------|
| H9 | Brown (GND) | **ESP32 GND** |
| H10 | Red (Power) | **ESP32 5V** (or external 5V supply) |
| H11 | Yellow (Signal) | **ESP32 D5** (GPIO 5) |

### Ultrasonic sensor

| From (Breadboard) | HC-SR04 Pin | Connect To |
|-------------------|-------------|------------|
| H4 | Red (VCC) | **ESP32 5V** (or VIN) |
| H5 | Gray (Trig) | **ESP32 D18** (GPIO 18) |
| H6 | White (Echo) | **ESP32 D15** (GPIO 15) |
| H7 | Black (GND) | **ESP32 GND** |

### Complete Wiring Summary

| ESP32 Pin | GPIO Number | Connected To |
|-----------|-------------|--------------|
| **D2** | GPIO 2 | Motor A (via transistor) |
| **D4** | GPIO 4 | Motor B (via transistor) |
| **D5** | GPIO 5 | Servo Signal (Yellow) |
| **D15** | GPIO 15 | HC-SR04 Echo (White) |
| **D18** | GPIO 18 | HC-SR04 Trigger (Gray) |
| **3v3** | - | Servo Red, HC-SR04 VCC |
| **GND** | - | All grounds (Servo Brown, HC-SR04 Black, Transistor Emitters, External Power -) |

## License

Copyright 2026 Grégory Saive <greg@evi.as> for re:Software S.L. (resoftware.es).

Licensed under the [3-Clause BSD License](./LICENSE).
