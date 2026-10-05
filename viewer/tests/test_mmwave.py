"""Tests for mmwave_reader, iwr6843aopevm_sensor, and mmwave exporter/player."""

import json
from pathlib import Path
import struct
import numpy as np
import pytest

from viewer.iwr6843aopevm_sensor import IWR6843AOPEVMSensor
from viewer.sensor_manager import SensorManager
from viewer.export_data import DataExporter
from player.player import load_recording


class TestMMWaveSensorAndExport:
    """Tests for IWR6843AOPEVMSensor, SensorManager mmwave handling, and exporter/player."""

    def test_sensor_creation_and_properties(self):
        """Should initialize mmwave sensor properties."""
        sensor = IWR6843AOPEVMSensor(
            deviceName="/dev/ttyUSB0_/dev/ttyUSB1",
            cfg_file="sensor_profile/iwr6843AOP_example.cfg",
            cfg_port="/dev/ttyUSB0",
            data_port="/dev/ttyUSB1",
            id=1,
            deviceType="mmwave",
        )

        assert sensor.id == 1
        assert sensor.deviceName == "/dev/ttyUSB0_/dev/ttyUSB1"
        assert sensor.deviceType == "mmwave"
        assert sensor.cfg_file == "sensor_profile/iwr6843AOP_example.cfg"
        assert sensor.sda is None
        assert sensor.sdaNum is None

    def test_sensor_data_update(self):
        """Should update 3D points and calculate distances."""
        sensor = IWR6843AOPEVMSensor(
            deviceName="/dev/ttyUSB0_/dev/ttyUSB1",
            id=1,
            deviceType="mmwave",
        )

        pts = np.array([[1.0, 2.0, 0.5], [0.5, 1.0, -0.2]], dtype=np.float32)
        sensor.update_data(points_3d=pts)

        assert len(sensor.points_3d) == 2
        assert len(sensor.distances) == 2
        assert sensor.distances[0] > 0

    def test_mmwave_display_color_is_red(self):
        """Should return solid red color for mmwave points."""
        sensor = IWR6843AOPEVMSensor(
            deviceName="/dev/ttyUSB0_/dev/ttyUSB1",
            id=1,
            deviceType="mmwave",
        )
        pts = np.array([[1.0, 2.0, 0.5]], dtype=np.float32)
        sensor.update_data(points_3d=pts)
        _, colors = sensor.get_display_data()
        assert len(colors) == 1
        np.testing.assert_array_equal(colors[0], [255, 0, 0])

    def test_sensor_manager_mmwave_device(self):
        """Should preallocate mmwave sensor in SensorManager."""
        sm = SensorManager()
        sensors = sm.add_device(
            deviceName="/dev/ttyUSB0_/dev/ttyUSB1",
            sensor_count=1,
            deviceType="mmwave",
            cfg_file="sensor_profile/iwr6843AOP_example.cfg",
        )

        assert len(sensors) == 1
        sensor = sensors[0]
        assert isinstance(sensor, IWR6843AOPEVMSensor)
        assert sensor.deviceType == "mmwave"
        assert sensor.cfg_file == "sensor_profile/iwr6843AOP_example.cfg"

    def test_export_and_replay_mmwave_recording(self, tmp_path):
        """Should record mmwave 3D datapoints and export to CSV and sensors_meta_data.txt, then load in player."""
        sm = SensorManager()
        sm.add_device(
            deviceName="/dev/ttyUSB0_/dev/ttyUSB1",
            sensor_count=1,
            deviceType="mmwave",
        )

        pts1 = np.array([[1.0, 2.0, 0.5]], dtype=np.float32)
        sensor1, _ = sm.process_packet("/dev/ttyUSB0_/dev/ttyUSB1", pts1)

        exporter = DataExporter(export_base_dir=str(tmp_path))
        exporter.start_recording([sensor1])

        pts2 = np.array([[1.1, 2.1, 0.5], [0.2, 1.0, -0.1]], dtype=np.float32)
        exporter.record_packet(sensor1.id, pts2)

        saved_dir = exporter.stop_recording()
        assert saved_dir is not None

        meta_file = saved_dir / "sensors_meta_data.txt"
        assert meta_file.exists()
        meta_text = meta_file.read_text()
        assert "mmwave" in meta_text

        # Player load test
        sensors_info = load_recording(saved_dir)
        assert len(sensors_info) == 1
        s0 = sensors_info[0]
        assert s0["conn_type"] == "mmwave"
        assert len(s0["packets"]) == 1
        replayed_pts = s0["packets"][0][1]
        assert len(replayed_pts) == 2
        assert abs(replayed_pts[0][0] - 1.1) < 1e-3
