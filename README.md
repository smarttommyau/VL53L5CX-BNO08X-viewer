# VL53L5CX-BNO08X-viewer

Real-time 3D point cloud viewer for the VL53L5CX multi-zone time-of-flight sensor with BNO085 IMU orientation tracking.

[![Watch the video](https://img.youtube.com/vi/s32OUzhjf4U/maxresdefault.jpg)](https://youtu.be/s32OUzhjf4U)

## Features

- **64-zone 3D visualization** - See the VL53L5CX's 8x8 measurement grid as rays in 3D space
- **Interactive 3D sensor positioning** - Drag translation gizmos and rotate sensors in 3D space to re-orient physical sensor setups
- **Data Export & Recording** - Record real-time sensor streams and export formatted CSV data and metadata (`sensors_meta_data.txt`)
- **Recording Player** - Replay recorded datasets synchronized in time with timeline scrubbing and playback controls
- **Real-time IMU tracking** - BNO085 orientation rotates the virtual view to match physical movement
- **Temporal filtering** - Exponential moving average smooths noisy measurements
- **Plane fitting** - Least squares and RANSAC methods for surface detection
- **Mapping mode** - Accumulate points over time to build a 3D map of your environment

## Hardware

**Components (~£24 / $30 / €28 from AliExpress):**
- ESP32 dev board (~£4)
- VL53L5CX ToF sensor (~£5)
- BNO085 IMU (~£15)

**Wiring:**

| VL53L5CX | ESP32   |
| -------- | ------- |
| VIN      | 3V3     |
| GND      | GND     |
| SDA      | GPIO 21 |
| SCL      | GPIO 22 |
| LPn      | GPIO 19 |

| BNO085   | ESP32   |
| -------- | ------- |
| VIN      | 3V3     |
| GND      | GND     |
| SDA      | GPIO 21 |
| SCL      | GPIO 22 |

Both sensors share the I2C bus (same SDA/SCL pins).

## Installation

### ESP32 Firmware

```bash
# Install libraries
arduino-cli lib install "SparkFun VL53L5CX Arduino Library"
arduino-cli lib install "SparkFun BNO08x Cortex Based IMU"

# Compile and upload
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/vl53l5cx_reader
arduino-cli upload --fqbn esp32:esp32:esp32 --port /dev/cu.usbserial-0001 firmware/vl53l5cx_reader
```

### Python Viewer

```bash
pip install -r viewer/requirements.txt
```

## Usage

```bash
# USB Serial device with 2 sensors:
python -m viewer --device1 port --port1 /dev/ttyACM0 --sensor-count1 2

# WiFi Telnet device (connecting to telnet port 2340):
python -m viewer --device1 wifi --IP1 192.168.1.100 --sensor-count1 1

# Mixed devices (1 WiFi and 1 USB serial port):
python -m viewer --device1 wifi --IP1 192.168.1.100 --sensor-count1 1 --device2 port --port2 /dev/ttyACM1 --sensor-count2 1

# Replay recorded data:
python -m player export_data/2026-10-04_15-30-00/
```

Open http://localhost:8080 in your browser.

**Options:**
- `--deviceX`: Device type for device X (`port` or `wifi`, default: `port`)
- `--portX`, `--port`, `-p`: Serial port path for USB device X (default: `/dev/cu.usbserial-0001`)
- `--IPX`, `--ipX`, `--IP`: IP address for WiFi device X (connects via Telnet on port `2340`)
- `--sensor-countX`: Number of VL53 sensors expected on device X (default: `1`)
- `--baud`, `-b`: Baud rate for serial devices (default: `115200`)
- `--viser-port`: Viser server port (default: `8080`)
- `--debug`: Enable verbose logging

## Sensor Specs

**VL53L5CX** ([datasheet](https://www.st.com/resource/en/datasheet/vl53l5cx.pdf)):
- **FoV:** 65° diagonal
- **Range:** 20mm - 4000mm
- **Distance type:** Perpendicular (z-axis), not radial

| Resolution | Zones | Max Frequency |
| ---------- | ----- | ------------- |
| 4x4        | 16    | 60 Hz         |
| 8x8        | 64    | 15 Hz         |

Currently configured for 8x8 at 15Hz.

## Serial Protocol

The ESP32 streams JSON over serial at 115200 baud:

```json
{"distances":[d0,d1,...,d63],"status":[s0,s1,...,s63],"quat":[w,x,y,z]}
```

- `distances`: 64 values in mm (perpendicular distance)
- `status`: 64 values (5 = valid measurement)
- `quat`: IMU quaternion (w, x, y, z)

Zones are row-major: 0-7 = row 0, 8-15 = row 1, etc.

## License

MIT
