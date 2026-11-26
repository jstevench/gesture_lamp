# gesture_lamp
gesture controlled lamp powered by esp32

For ESP32-C3 USB Serial, enable USB CDC On Boot. This does not apply to the
classic ESP32 board with a USB-to-serial bridge described below.

## Current board: classic ESP32 + MPU6050

### Palm angle brightness and animations

The combined sketches are in `gesture_slider_sender` and
`gesture_slider_receiver`. Use the original `sender_espnow` and
`receiver_espnow` sketches for the original flick controls.

Mount the MPU6050 flat against the hand with its X axis running from wrist to
fingertips. Hold your palm level, facing down, and still at power-on. The hand
unit starts locked. Quickly bend your wrist to snap your fingertips up or down,
or give the whole hand a quick upward or downward flick, to toggle the lock.
Roll your palm from facing down to facing up to change brightness while unlocked.

| Palm position | Brightness |
| --- | ---: |
| Down (0 degrees) | 0 (off) |
| Edge-on (90 degrees) | 512 |
| Up (180 degrees) | 1023 (full) |

Between the endpoints the target changes by about 5.7 PWM levels per degree.
The lamp follows the angle as you turn, and holding an angle holds the setting.
A hard sideways flick still selects the next or previous animation while
unlocked. When locked, both hand controls stop sending changes, and the lamp
keeps playing the selected animation at the last brightness setting.

If the sensor's Y axis runs along the hand, set `HAND_FORWARD_AXIS` to `1`.
If left and right are reversed, set `SIDE_SIGN` to `-1`.

1. In Arduino IDE use **esp32 by Espressif Systems 3.3.12** and select the
   lamp receiver's board and COM port. The current GPIO23 lamp receiver uses
   **ESP32 Dev Module**. Upload `gesture_slider_receiver/gesture_slider_receiver.ino`.
   The updated receiver prints `Ready for palm-angle brightness and legacy
   gesture commands.`
2. Open the receiver's Serial Monitor at **115200 baud** and reset it to check
   its station MAC address. If it differs from the address in `receiverMAC`,
   update that value in the sender.
3. Select **ESP32 Dev Module** and the hand unit's COM port. Upload
   `gesture_slider_sender/gesture_slider_sender.ino`. The MPU6050 uses SDA 21 and SCL 22 as below.
4. Flick up once to unlock. Turn your palm from down to up and back, and test
   a sideways animation flick. Flick down once to lock. Check that either hand
   gesture leaves the brightness and selected animation alone while locked.
   Repeat after powering both boards from their standalone supplies.

If a flick is missed, open the hand unit's Serial Monitor at **115200 baud**
and send `d`. The motion report prints the palm angle and peak lift, sideways,
pitch (wrist bend), and roll (palm turn) readings every 250 ms. Send `d` again to stop.

Upload each updated sketch once over USB with its board's external supply
disconnected. Unplug USB before reconnecting that supply. The ESP32 keeps its
firmware in flash and can then run without a laptop. Both boards use ESP-NOW
channel 1 and can talk directly.

The slider scales the animation's whole brightness range without changing its
timing or mode. The selected brightness sets the peak, and zero turns the lamp
off in any mode. An animation continues while the slider is locked. To check
this over USB, enter `a3` in the receiver's Serial Monitor and compare `256`
with `768` while the breathing effect plays. Serial commands and the legacy
receiver commands remain available.

### Standalone sensor test

Use `gesture_test_esp32/gesture_test_esp32.ino` for the replacement ESP32 board.
The existing `gesture_test/gesture_test.ino` currently contains a GPIO contact
test; it is not the flick detector. This new sketch keeps the original flick
algorithm and includes an I2C scan if sensor initialization fails.

Disconnect USB before wiring. Follow the board's printed pin labels, not the
old C3 breadboard row numbers:

| MPU6050 wire | Classic ESP32 pin label |
| --- | --- |
| Red: VCC | 3V3 |
| Black: GND | GND |
| Yellow: SDA | D21 / GPIO21 |
| Green: SCL | D22 / GPIO22 |

In Arduino IDE select **ESP32 Dev Module** and the new board's COM port.
Use the same Adafruit MPU6050 library and dependencies. Upload the new sketch,
open Serial Monitor at **115200 baud**, press **EN** to restart, and hold the
sensor still until `Ready!`. Then test vertical flicks as described below.
After each flick, pause until `Armed. Ready for next flick.` appears. This
version checks stillness independently of the startup angle and refreshes the
vertical reference after a stable pause, so a small change in holding angle
does not leave the detector waiting indefinitely. Keep orientation steady
during each flick; arbitrary rotation can still cause false detections.
This board uses a USB-to-serial bridge; there is no USB CDC On Boot setting
to enable. If uploading remains at `Connecting...`, hold BOOT until writing
starts, then release it.

The C3 instructions below are retained for reference. The new sketch requires
compilation and a hardware test on the replacement board.

## Flick detection: ESP32-C3 Super Mini + MPU6050 (HW-123)

These are the original C3 sensor wiring notes. `gesture_test/gesture_test.ino`
has since been replaced with a GPIO contact test; use `gesture_test_esp32` for
the working classic ESP32 sensor test or `sender_espnow` for wireless gestures.

### Wiring

Disconnect power before connecting these four wires:

| MPU6050 module | ESP32-C3 Super Mini |
| --- | --- |
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO 4 (pin marked 4) |
| SCL | GPIO 5 (pin marked 5) |

Leave INT, XDA and XCL unconnected. AD0 may be connected to GND to select
address 0x68; the sketch checks both 0x68 and 0x69. Use 3.3 V for this setup.
Pins 4 and 5 are deliberately selected in the code, even though the pinout photo
labels 8 and 9 as SDA/SCL. If already wired differently, either rewire as above
or change `SDA_PIN` and `SCL_PIN` to match.

### Upload and try it

1. In Arduino IDE Boards Manager, install **esp32 by Espressif Systems**.
2. In Library Manager, install **Adafruit MPU6050** and accept its dependencies
   (**Adafruit BusIO** and **Adafruit Unified Sensor**).
3. Open `gesture_test/gesture_test.ino`.
4. Select **ESP32C3 Dev Module**, select the connected board's port, and set
   **Tools > USB CDC On Boot > Enabled**.
5. Upload, then open Serial Monitor at **115200 baud**. Press RESET if you missed
   the startup messages. If uploading will not connect, hold BOOT, press and
   release RESET, release BOOT, select the port again, and retry uploading.
6. Hold the sensor still in your normal holding position until `Ready!` appears.
7. Move it sharply upward, pause, then move it sharply downward. Keep roughly
   the same orientation; move the sensor with your hand rather than rotating it.

Expected gesture output:

```text
FLICK UP
FLICK DOWN
```

The sketch samples about 100 times per second and subtracts the resting
acceleration measured at startup. It uses that resting direction to identify
vertical motion, so no particular sensor axis needs to face up. After a gesture,
it waits at least 450 ms and requires 200 ms of stillness before detecting another.
Press RESET to recalibrate if you change how you hold or mount the sensor.

### Troubleshooting and tuning

- **MPU6050 not found:** the sketch now scans the I2C bus and prints responding
  addresses. Copy that output for diagnosis. No responses means checking the four
  wires, matching pin constants, header contact, and solder joints first; a lit
  sensor LED does not confirm data communication. A response with failed
  initialization needs further diagnosis of the sensor and connection.
- **Calibration failed:** keep the sensor still and press RESET to retry.
- **No Serial output:** check the port, 115200 baud, and USB CDC On Boot setting.
- **Flicks missed:** lower `TRIGGER_MS2` from 3.0 to 2.0 and try again.
- **Too sensitive:** raise `TRIGGER_MS2`, for example to 4.0.
- **Inspect acceleration:** set `PRINT_READINGS` to `true` and upload again.
- This is a threshold-based prototype for deliberate vertical flicks. Shaking,
  impacts, or twisting can produce false detections; it does not track arbitrary
  wrist rotation or prove that a whole motion was an intentional gesture.

Hardware verification is still required: upload it to your board and test actual
flicks to tune sensitivity. This repository change has not been flashed to hardware.

API references: [Adafruit MPU6050 example](https://github.com/adafruit/Adafruit_MPU6050/blob/master/examples/basic_readings/basic_readings.ino),
[ESP32 I2C](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/i2c.html),
[ESP32 USB CDC settings](https://docs.espressif.com/projects/arduino-esp32/en/latest/tutorials/cdc_dfu_flash.html).

Low Power Radio Testing
    Very effective with direct LoS
        Probably around 50m
    Ineffective around wall corners
        Not very good at bouncing around corners
        Wall >2ft too thick
    Mildly effective through plastic
        Thicker plastic >10" in thickness would probably stop it
    Not very effective with metal
        Maybe ~5cm of metal
