# Export data from the sensor

## File format
2 types of files are generated when exporting data from the sensor:
1. Sensors meta data file: `sensors_meta_data.txt`
2. Sensor data file: 
   1. For VL53L5CX sensor: `name.VL53L5CX.csv` (e.g. `dev-ttyACM0_4.VL53L5CX.csv`, `192.168.1.100_4.VL53L5CX.csv`)
   2. For mmWave sensor: `name.mmWave.csv` (e.g. `dev-ttyUSB0-dev-ttyUSB1_0.mmWave.csv`)

### Sensor meta data file (`sensors_meta_data.txt`)
It includes the following information:
- Sensor Name: Serial Port or IP address of the sensor + sda pin number
  - E.g. `/dev/ttyACM0_4`, `192.168.1.100_4`
- Sensor Connection Type: `WiFi`, `Serial`, or `mmwave`
- Sda Pin: GPIO pin number for SDA (0 for mmwave)
- 3D posture of the sensor in viser:
  - Position: x, y, z coordinates of the sensor in 3D space
  - Rotation: Quaternion (x, y, z, w) representing the sensor's orientation in 3D space
- Relative location of the sensor data file: relative path to the sensor CSV file from export directory

Example content of `sensors_meta_data.txt`:
```
/dev/ttyACM0_4
Serial
4
0.0, 0.0, 0.0
0.0, 0.0, 0.0, 1.0
./2026-10-04_15-30-00/dev-ttyACM0_4.VL53L5CX.csv
dev-ttyUSB0-dev-ttyUSB1_0
mmwave
0
0.0, 0.0, 0.0
0.0, 0.0, 0.0, 1.0
./2026-10-04_15-30-00/dev-ttyUSB0-dev-ttyUSB1_0.mmWave.csv
```

### Sensor data file for VL53L5CX sensor (`name.VL53L5CX.csv`)
It includes the following information:
- Timestamp: The timestamp of the data point in milliseconds since start of recording
- Distance: The distance measured by the sensor in millimeters (Array separated by ;)
  - E.g. `100;200;300;400;500;600;700;800;900;1000`
- Status: The measurement validity status from sensor (Array separated by ;)
  - E.g. `5;5;5;5;5;5;5;5;5;5`

Example content of `dev-ttyACM0_4.VL53L5CX.csv`:
```
Timestamp,Distance,Status
0,100;200;300;400;500;600;700;800;900;1000,5;5;5;5;5;5;5;5;5;5
10,110;210;310;410;510;610;710;810;910;1010,5;5;5;5;5;5;5;5;5;5
```

### Sensor data file for mmWave sensor (`name.mmWave.csv`)
It includes the following information:
- Timestamp: The timestamp of the data point in milliseconds since start of recording
- DataPoints: 3D point cloud coordinates `(x1;y1;z1)(x2;y2;z2)...` in meters
- Doppler: Doppler velocity values separated by `;` in m/s
- Intensity: Intensity / SNR values separated by `;` in dB

Example content of `dev-ttyUSB0-dev-ttyUSB1_0.mmWave.csv`:
```
Timestamp,DataPoints,Doppler,Intensity
0,(1.20;0.50;2.30)(0.40;1.10;1.80),0.00;-0.15,25.4;18.2
10,(1.21;0.51;2.31)(0.41;1.11;1.81),0.02;-0.14,25.8;18.0
```
