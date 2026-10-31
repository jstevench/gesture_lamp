# gesture_lamp
gesture controlled lamp powered by esp32

WHEN UPLOADING CODE ENABLE USB CDC ON BOOT VERY IMPORTANT

## Flick detection: ESP32-C3 Super Mini + MPU6050 (HW-123)

Open `gesture_test/gesture_test.ino` for the standalone gesture test. It reads
the MPU6050 and prints `FLICK UP` or `FLICK DOWN`. The existing ESP-NOW sender
and receiver remain separate radio tests; this sketch does not control the lamp.

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

- **MPU6050 not found:** check the four wires, power, and matching pin constants.
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
