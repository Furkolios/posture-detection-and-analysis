# Hardware Wiring Reference

Every wire in the system. Use the LaunchPad header pin numbers, not
just the signal names — that way nobody has to look up the silkscreen.

## Components

- **MSP432P401R LaunchPad** (MSP-EXP432P401R, the standard red Rev 2.0)
- **2× MPU-6050 IMU breakout** (typical SY-104 / GY-521 style)
  - "lower" IMU: at the hip/lower back, AD0 tied LOW → I²C address `0x68`
  - "upper" IMU: on the upper back, AD0 tied HIGH → I²C address `0x69`
- **HM-10 BLE module** (3.3V logic, 9600 baud factory default)
- **Active piezo buzzer** (3.3V compatible, draws ≤30 mA)
- **2× 4.7 kΩ resistors** for I²C pull-ups (skip if your IMU breakouts
  already have on-board pull-ups — most do)

## MSP432 pin assignment

| Function                | MSP432 pin | LaunchPad header | Module   |
|-------------------------|------------|------------------|----------|
| I²C SDA                 | P6.4       | J3.10            | EUSCI_B1 |
| I²C SCL                 | P6.5       | J3.9             | EUSCI_B1 |
| UART TX (to HM-10 RX)   | P3.3       | J4.34            | EUSCI_A2 |
| UART RX (from HM-10 TX) | P3.2       | J4.33            | EUSCI_A2 |
| Buzzer                  | P2.4       | J4.31            | GPIO     |
| 3.3V supply             | —          | J3.1             | —        |
| GND                     | —          | J3.20, J6.20     | —        |
 
If you change any of these, update the matching `#define`s in
`src/hal_*_msp432.c` AND the README_teammates.md (which doesn't list
pins but does mention "9600 baud").

## Wire-by-wire connection table

| From            | Pin        | To              | Pin         | Notes                                              |
|-----------------|------------|-----------------|-------------|----------------------------------------------------|
| LaunchPad J3.1  | 3.3V       | I²C bus         | VCC rail    | Powers both IMUs and HM-10                         |
| LaunchPad J6.20 | GND        | I²C bus         | GND rail    | Common ground for everything                       |
| LaunchPad J3.10 | P6.4 / SDA | Both IMUs       | SDA pin     | Shared I²C bus                                     |
| LaunchPad J3.9  | P6.5 / SCL | Both IMUs       | SCL pin     | Shared I²C bus                                     |
| 3.3V rail       | (any)      | 4.7 kΩ resistor | one end     | I²C SDA pull-up (skip if breakout has it)          |
| 4.7 kΩ resistor | other end  | SDA line        | (any point) |                                                    |
| 3.3V rail       | (any)      | 4.7 kΩ resistor | one end     | I²C SCL pull-up (skip if breakout has it)          |
| 4.7 kΩ resistor | other end  | SCL line        | (any point) |                                                    |
| Lower IMU       | AD0        | GND rail        | —           | Sets address to 0x68                               |
| Upper IMU       | AD0        | 3.3V rail       | —           | Sets address to 0x69                               |
| LaunchPad J4.34 | P3.3 (TX)  | HM-10           | RX pin      | UART data out                                      |
| LaunchPad J4.33 | P3.2 (RX)  | HM-10           | TX pin      | UART data in (unused right now, wired for AT cmds) |
| 3.3V rail       | (any)      | HM-10           | VCC         |                                                    |
| GND rail        | (any)      | HM-10           | GND         |                                                    |
| LaunchPad J4.31 | P2.4       | Buzzer          | + terminal  | Active piezo                                       |
| GND rail        | (any)      | Buzzer          | − terminal  |                                                    |

## Cable length notes

- The wire run between the upper IMU and the belt-mounted LaunchPad
  enclosure is the longest in the system. Keep it under **30 cm** —
  longer than that and the I²C pull-ups need to be lower-impedance
  (try 2.2 kΩ instead of 4.7 kΩ) or the bus may not work reliably.
- Use stranded wire or a small ribbon cable with a strain relief
  loop near the IMU. The MPU breakouts have small pads that crack if
  the wire is repeatedly tugged.

## Bring-up checklist

Before powering on with firmware flashed:

1. Continuity-check 3.3V rail is not shorted to GND.
2. Confirm SDA and SCL are NOT swapped at either IMU (easy mistake;
   the chip silently does nothing).
3. Confirm AD0 is at GND on the lower IMU and at 3.3V on the upper.
   Verify with a multimeter, not just by looking at the wire — bent
   pins happen.
4. Confirm HM-10 TX→MSP432 RX, HM-10 RX→MSP432 TX (i.e. the lines
   cross). A common mistake is to wire TX→TX.
5. Power up. The HM-10's red LED should blink fast (advertising). Pair
   with a phone or laptop; the LED goes solid when connected.
6. Run the test snippet from `README_teammates.md` and confirm
   `0xA5`-prefixed packets arrive.

If step 6 fails, narrow down by:
- Removing one IMU and seeing if the other still produces good data
  (rules out address conflicts).
- Replacing the HM-10 with a USB-UART adapter on the same MSP432 TX
  line at 9600 baud, to rule out BLE-side problems.
