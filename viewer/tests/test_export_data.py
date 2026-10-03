"""Tests for export_data module and player recording loader."""

from pathlib import Path
import numpy as np
import pytest

from viewer.export_data import DataExporter
from viewer.sensor_manager import SensorManager
from player.player import load_recording


class TestExportDataAndPlayer:
    """Tests for DataExporter recording/exporting and player recording loader."""

    def test_export_data_file_creation(self, tmp_path):
        """Should record packets and export sensors_meta_data.txt and CSV files."""
        exporter = DataExporter(export_base_dir=str(tmp_path))

        sm = SensorManager()
        sensors = sm.add_device("/dev/ttyACM0", sensor_count=1, deviceType="port")
        sensor1, _ = sm.process_packet("/dev/ttyACM0", np.ones(64), np.ones(64), sda=4)

        exporter.start_recording([sensor1])
        assert exporter.is_recording is True

        distances = np.full(64, 250, dtype=np.float32)
        status = np.full(64, 5, dtype=np.uint8)

        exporter.record_packet(sensor1.id, distances, status)
        exporter.record_packet(sensor1.id, distances + 10, status)

        saved_dir = exporter.stop_recording()

        assert saved_dir is not None
        assert saved_dir.exists()

        meta_file = saved_dir / "sensors_meta_data.txt"
        assert meta_file.exists()

        meta_text = meta_file.read_text()
        assert "/dev/ttyACM0_4" in meta_text
        assert "Serial" in meta_text
        assert "4" in meta_text

        # Test loading recorded data in player
        sensors_info = load_recording(saved_dir)
        assert len(sensors_info) == 1
        s0 = sensors_info[0]
        assert s0["name"] == "/dev/ttyACM0_4"
        assert s0["conn_type"] == "Serial"
        assert s0["sda"] == 4
        assert len(s0["packets"]) == 2
        assert s0["packets"][0][1][0] == 250.0
        assert s0["packets"][1][1][0] == 260.0
