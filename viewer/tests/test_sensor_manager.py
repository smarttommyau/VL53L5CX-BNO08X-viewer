"""Tests for sensor_manager and vl53l5cx_sensor modules."""

import sys
import numpy as np
import pytest

from viewer.sensor_manager import SensorManager
from viewer.vl53l5cx_sensor import VL53L5CXSensor
from viewer.viewer import parse_args


class TestSensorManager:
    """Tests for SensorManager class."""

    def test_add_device_preallocates_sensors(self):
        """Should preallocate specified number of sensors with sequential IDs and pending SDA."""
        sm = SensorManager()
        sensors = sm.add_device("/dev/ttyUSB0", sensor_count=3, deviceType="port")

        assert len(sensors) == 3
        assert sensors[0].id == 1
        assert sensors[0].deviceName == "/dev/ttyUSB0"
        assert sensors[0].deviceType == "port"
        assert sensors[0].sda is None

        assert sensors[1].id == 2
        assert sensors[1].deviceName == "/dev/ttyUSB0"
        assert sensors[1].sda is None

        assert sensors[2].id == 3
        assert sensors[2].deviceName == "/dev/ttyUSB0"
        assert sensors[2].sda is None

        assert sm.get_active_count() == 3
        assert sm.get_sensor_ids() == [1, 2, 3]

    def test_process_packet_assigns_sda_sequentially(self):
        """Should assign SDA to unassigned sensors when JSON packet arrives."""
        sm = SensorManager()
        sm.add_device("/dev/ttyUSB0", sensor_count=2)

        distances = np.full(64, 500, dtype=np.float32)
        status = np.full(64, 5, dtype=np.uint8)

        # First packet with sda=4 -> assigns to Sensor 1
        sensor1, newly_assigned1 = sm.process_packet("/dev/ttyUSB0", distances, status, sda=4)
        assert sensor1.id == 1
        assert sensor1.sda == 4
        assert newly_assigned1 is True

        # Second packet with sda=17 -> assigns to Sensor 2
        sensor2, newly_assigned2 = sm.process_packet("/dev/ttyUSB0", distances, status, sda=17)
        assert sensor2.id == 2
        assert sensor2.sda == 17
        assert newly_assigned2 is True

        # Subsequent packet with sda=4 -> routes to Sensor 1 without re-assigning
        sensor1_again, newly_assigned3 = sm.process_packet("/dev/ttyUSB0", distances, status, sda=4)
        assert sensor1_again.id == 1
        assert sensor1_again.sda == 4
        assert newly_assigned3 is False

    def test_process_packet_dynamic_sensor_creation(self):
        """Should create a new sensor dynamically if all pre-allocated sensors are assigned."""
        sm = SensorManager()
        sm.add_device("/dev/ttyUSB0", sensor_count=1)

        distances = np.full(64, 500, dtype=np.float32)
        status = np.full(64, 5, dtype=np.uint8)

        # First packet uses pre-allocated Sensor 1
        s1, _ = sm.process_packet("/dev/ttyUSB0", distances, status, sda=4)
        assert s1.id == 1
        assert s1.sda == 4

        # Second packet with new SDA pin creates Sensor 2 dynamically
        s2, newly_assigned = sm.process_packet("/dev/ttyUSB0", distances, status, sda=17)
        assert s2.id == 2
        assert s2.sda == 17
        assert newly_assigned is True
        assert sm.get_active_count() == 2


class TestVL53L5CXSensor:
    """Tests for VL53L5CXSensor class."""

    def test_sensor_properties_and_backward_compatibility(self):
        """Should support sda and sdaNum getter/setter and correct hierarchy path."""
        sensor = VL53L5CXSensor(deviceName="/dev/test", id=5, sda=4, deviceType="port")

        assert sensor.id == 5
        assert sensor.deviceName == "/dev/test"
        assert sensor.deviceType == "port"
        assert sensor.sda == 4
        assert sensor.sdaNum == 4
        assert sensor.hierarchy_path == "/sensor_5"

        # Setter test
        sensor.sdaNum = 18
        assert sensor.sda == 18
        assert sensor.sdaNum == 18

    def test_update_data_and_fps(self):
        """Should update distances and calculate fps."""
        sensor = VL53L5CXSensor(deviceName="/dev/test", id=1)
        distances = np.ones(64, dtype=np.float32) * 100
        status = np.ones(64, dtype=np.uint8) * 5

        sensor.update_data(distances, status)
        assert sensor.distances is not None
        assert np.array_equal(sensor.distances, distances)
        assert sensor.status is not None
        assert np.array_equal(sensor.status, status)


class TestArgumentParsing:
    """Tests for CLI argument parsing."""

    def test_parse_args_defaults(self, monkeypatch):
        """Should parse default device1, port1 and sensor-count1."""
        monkeypatch.setattr(sys, "argv", ["viewer"])
        args, dev_cfg = parse_args()

        assert dev_cfg[1]["device"] == "port"
        assert dev_cfg[1]["port"] == "/dev/cu.usbserial-0001"
        assert dev_cfg[1]["sensor_count"] == 1

    def test_parse_args_separate_device1_and_port1(self, monkeypatch):
        """Should parse --device1 as device type and --port1 as port path."""
        monkeypatch.setattr(
            sys,
            "argv",
            ["viewer", "--device1=port", "--port1=/dev/ttyUSB0", "--sensor-count1=2"],
        )
        args, dev_cfg = parse_args()

        assert dev_cfg[1]["device"] == "port"
        assert dev_cfg[1]["port"] == "/dev/ttyUSB0"
        assert dev_cfg[1]["sensor_count"] == 2

    def test_parse_args_port_alias(self, monkeypatch):
        """Should accept --port or -p as alias for device 1 port."""
        monkeypatch.setattr(
            sys,
            "argv",
            ["viewer", "--device1", "port", "--port", "/dev/ttyACM0", "--sensor-count1", "3"],
        )
        args, dev_cfg = parse_args()

        assert dev_cfg[1]["device"] == "port"
        assert dev_cfg[1]["port"] == "/dev/ttyACM0"
        assert dev_cfg[1]["sensor_count"] == 3
