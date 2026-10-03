# Export data from the sensor

## File format
2 type of files are generated when exporting data from the sensor:
1. Sensors meta data file: `sensors_meta_data.txt`
2. Sensor data file: `sensor_data.csv`

### Sensor meta data file
It includes the following information:
- Sensor Name: IP or Serial Port of the sensor + sda pin number
  - E.g. /dev/ttyACM0_4, 192.168.1.100_4
- Sensor Connection Type: `WiFi` or `Serial`
- Sda Pin: GPIO pin number for SDA
- 3D data of the sensor in viser:
  - Position: x, y, z coordinates of the sensor in 3D space
  - Rotation: Quaternion (x, y, z, w) representing the sensor's orientation in 3D space
- relative location of the sensor data in file system: relative path to the sensor data file from the current working directory
Example content of `sensors_meta_data.txt`:
```
/dev/ttyACM0_4
Serial
4
0.0, 0.0, 0.0
0.0, 0.0, 0.0, 1.0
./2026-10-04_15-30-00/dev-ttyACM0_21_data.csv
192.168.1.100_4
WiFi
4
0.0, 0.0, 0.0
0.0, 0.0, 0.0, 1.0
./2026-10-04_15-30-00/192.168.1.100_21_data.csv
192.168.1.100_17
WiFi
17
0.0, 0.0, 0.0
0.0, 0.0, 0.0, 1.0
./2026-10-04_15-30-00/192.168.1.100_17_data.csv
```
### Sensor data file
It includes the following information:
- Timestamp: The timestamp of the data point in milliseconds since start of record
- Distance: The distance measured by the sensor in millimeters (Array seperated by ;)
  - E.g. 100;200;300;400;500;600;700;800;900;1000...
- status: the status from sensor(Array seperated by ;)
  - E.g. 5;5;5;5;5;5;5;5;5;5...
  - 	/* Status indicating the measurement validity (5 & 9 means ranging OK)*/
Example content of `sensor_data.csv`:
```
Timestamp,Distance,Status
0,100;200;300;400;500;600;700;800;900;1000,5;5;5;5;5;5;5;5;5;5
10,110;210;310;410;510;610;710;810;910;1010,5;5;5;5;5;5;5;5;
````
> Note: In real case the distance and status array should be 64 elements long, but for the sake of example, we have shortened it to 10 elements.