# Smaili the MicroRover

A tiny *smiling* (:smiley:) rover that speaks JSON!

This sketch contains the firmware source code (C++) for a ESP32 DEVKIT V1 board
which is wired together with listed components. Note that for practicability,
the ESP32 board is plugged on a `10:30` prototyping breadboard and attached to
the bottom plate of the rover platform.
Refer to [3D Printing](#3d-printing) for the model files.

## Implementation notes

- Main sketch file: [`micro-rover-esp32-v0.ino`](./micro-rover-esp32-v0.ino)
  - Simple interface to Arduino which creates a global `MicroRover` instance.
  - Commands are processed by a global instance of `Processor`.
- `MicroRover` class: [`src/micro_rover.h`](./src/micro_rover.h)
  - Handles the setup/configuration of devices and protocols (serial).
  - Used as a wrapper to retrieve individual device struct objects.
- `Processor` class: [`src/processor.h`](./src/processor.h)
  - Processes a JSON input into a JsonDocument (ArduinoJson.h)
  - Expects to find `command`, e.g. `{"command": "status"}`.
  - Handles the input and options to execute commands from `commands/`.
- `Device` struct: [`src/types.h`](./src/types.h)
  - A **simplistic** hardware-to-software wrapper to configure hardware mapping.
  - i.e. The `pins` property in `Device` contains the number of pins that are
    wired (or soldered) to a ESP32 pin. e.g. a `Device<2>` contains 2 pins that
    may be used with `digitalWrite` or `analogWrite`; whereas a `Device<1>`
    contains 1 pin. Generally speaking, don't count voltage and grounding pins.

For now, use the `Serial Monitor` to send JSON commands to Smaili.

## 3D Printing

This project includes multiples 3D-printed components. Some modifications were
necessary to fit and plug the custom DC motors and servo motor.

### Original models

| 3D Model | Attribution | Description |
| --- | --- | --- |
| [Rover platform](https://www.printables.com/model/145064-rover-platform) | [@Nahuel_114346](https://www.printables.com/@Nahuel_114346) | A small rover platform about 20cm x 20cm |
| [Hinged Case for Ultrasonic Sensor](https://www.printables.com/model/1338788-hinged-case-for-ultrasonic-sensor-hc-sr04) | [@ConnorPritch_3339490](https://www.printables.com/@ConnorPritch_3339490) | A case for the HC-SR04 with clean cable case. |
| [Ultrasonic sensor HC-SR04 mount on servo](https://www.printables.com/model/84061-ultrasonic-sensor-hc-sr04-mount-on-servo) | [@Balex_174247](https://www.printables.com/model/84061-ultrasonic-sensor-hc-sr04-mount-on-servo) | A funny-faced case for HC-SR04, compatible with above internal case. |

### Modified parts

- `DCMotorSpindleAdapter`: Uses a custom adapter to put on the DC motors pins,
  so that they can be plugged in the original spindle component.
- `CoverWithCableCut`: Uses a cable cut on the back-part of the top cover of the
  rover.
- `CogModifiedForDC`: Uses a custom cog (wheels) that are pluggable with the
  specific DC motors.
- `ShelfModifiedForServo`: Uses a custom front-shelf that fits with a servo
  motor opening in the top-middle.

The modified model files are available in `model-files/`, e.g. the full `.3mf`
project, designed to be printed on a `225x225x265` print bed.

WARNING: I did use *glue* for this project as well, e.g. to glue together the
rover case and the DC motor, because my motors used to jump out of position.

### Model remix

| 3D Model | Attribution | Description |
| --- | --- | --- |
| [Smaili the MicroRover - Rover platform remix](https://www.printables.com/model/1558487-smaili-the-microrover-rover-platform-remix) | A remix of the rover platform model to fit custom DC motors and an added HC-SR04 ultrasonic sensor built on a SG90 Servo motor. |

## Hardware

| Component | Device | Marketplace |
| --- | --- | --- |
| 1x Development board | ESP32 DevKit v1 (USB-C) | [AliExpress](https://es.aliexpress.com/item/1005010020285036.html) |
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
| H11 | Yellow (Signal) | **ESP32 D22** (GPIO 22) |

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
| **D22** | GPIO 22 | Servo Signal (Yellow) |
| **D15** | GPIO 15 | HC-SR04 Echo (White) |
| **D18** | GPIO 18 | HC-SR04 Trigger (Gray) |
| **3v3** | - | Servo Red, HC-SR04 VCC |
| **GND** | - | All grounds (Servo Brown, HC-SR04 Black, Transistor Emitters, External Power -) |

## License

Copyright 2026 Grégory Saive <greg@evi.as> for re:Software S.L. (resoftware.es).

Licensed under the [3-Clause BSD License](./LICENSE).
