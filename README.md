# gesture_lamp
gesture controlled lamp powered by esp32

For ESP32-C3 USB Serial, enable USB CDC On Boot. This does not apply to the
classic ESP32 board with a USB-to-serial bridge described below.

## Current board: classic ESP32 + MPU6050

### Wireless gesture sender

`sender_espnow/sender_espnow.ino` now combines the working MPU6050 detector with
ESP-NOW on the **classic ESP32** (SDA 21, SCL 22). It replaces the old C3 BOOT
button sender. The standalone `gesture_test_esp32` sketch remains available.
Use **esp32 by Espressif Systems 3.3.12**, the version used for the successful
sensor test; the send callback signature depends on the core/ESP-IDF version.

1. Upload `receiver_espnow/receiver_espnow.ino` to the **receiver C3**, using
   **ESP32C3 Dev Module** and **USB CDC On Boot > Enabled**. Open its Serial
   Monitor at 115200 and press RESET to read **This receiver's MAC address**.
2. Put that address in `receiverMAC` near the top of the sender sketch. The
   existing `70:AF:09:0D:D0:A8` is retained from the original sender and must be
   verified; do not replace it with the new sender's own MAC.
3. Upload `sender_espnow/sender_espnow.ino` to the sensor-equipped **classic
   ESP32**, using **ESP32 Dev Module** and its COM port. Wiring stays as below.
4. Power both boards. No router is required. Both sketches explicitly use
   **radio channel 1**. Hold the MPU6050 still for calibration, then flick and
   pause until the sender reports `Armed. Ready for next flick.`.
5. Check the sender for `Queued gesture command: 1` (up) or `-1` (down), followed
   by `Send OK`. Check the receiver for `Received: FLICK UP (1)` or
   `Received: FLICK DOWN (-1)` and an onboard LED flash.

The payload is one four-byte signed integer: **1 = up**, **-1 = down**. The old
integer receiver can print these values too; the updated receiver names the
gestures, validates packet length, and keeps work outside the Wi-Fi callback.
`Send OK` means radio delivery, not confirmation that a lamp changed state.
There are no automatic retries or application acknowledgements in this prototype.
If delivery fails, check receiver power, its station MAC, and matching channels.
The receiver LED is GPIO8, active-low, for the original C3 board; other receiver
boards need their LED configuration checked. Actual lamp output remains to be
implemented once its driver/pin wiring and desired gesture actions are known.

The combined wireless sketches still require compilation and a two-board
hardware test. Reference: [Espressif ESP-NOW documentation](https://docs.espressif.com/projects/esp-idf/en/v5.5.6/esp32c3/api-reference/network/esp_now.html).

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
