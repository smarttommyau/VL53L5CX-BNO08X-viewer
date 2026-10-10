"""export_data.py - Module for exporting sensor data according to export_data/README.md."""

import csv
import logging
from pathlib import Path
import time
from typing import Optional

import numpy as np

logger = logging.getLogger("vl53l5cx_viewer.exporter")


class DataExporter:
    """Manages recording and exporting sensor data according to export_data/README.md."""

    def __init__(self, export_base_dir: str = "export_data"):
        self.export_base_dir = Path(export_base_dir)
        self.is_recording: bool = False
        self.start_time: float = 0.0
        self.timestamp_str: str = ""
        self.records: dict[int, dict] = {}  # sensor_id -> {meta, packets}

    def start_recording(self, sensors: list):
        """Start recording sensor data for all active sensors."""
        self.start_time = time.time()
        self.timestamp_str = time.strftime("%Y-%m-%d_%H-%M-%S")
        self.is_recording = True
        self.records.clear()

        for sensor in sensors:
            pos = (0.0, 0.0, 0.0)
            wxyz = (1.0, 0.0, 0.0, 0.0)
            if hasattr(sensor, "frame_handle") and sensor.frame_handle is not None:
                pos = tuple(float(p) for p in sensor.frame_handle.position)
                wxyz = tuple(float(q) for q in sensor.frame_handle.wxyz)
            elif hasattr(sensor, "initial_pos"):
                pos = sensor.initial_pos
                wxyz = sensor.initial_wxyz

            dev_type = getattr(sensor, "deviceType", "port").lower()
            if dev_type == "mmwave":
                conn_type = "mmwave"
                sda_val = 0
            elif dev_type == "wifi":
                conn_type = "WiFi"
                sda_val = sensor.sda if sensor.sda is not None else 0
            else:
                conn_type = "Serial"
                sda_val = sensor.sda if sensor.sda is not None else 0

            self.records[sensor.id] = {
                "sensor": sensor,
                "sensor_name": f"{sensor.deviceName}_{sda_val}",
                "device_name": sensor.deviceName,
                "conn_type": conn_type,
                "sda": sda_val,
                "position": pos,
                "wxyz": wxyz,  # (w, x, y, z)
                "packets": [],  # list of packets
            }

        logger.info(
            "Started recording at %s for %d sensor(s)",
            self.timestamp_str,
            len(sensors),
        )

    def record_packet(
        self,
        sensor_id: int,
        data1: np.ndarray,
        distances: Optional[np.ndarray] = None,
        status: Optional[np.ndarray] = None,
        doppler: Optional[np.ndarray] = None,
        intensity: Optional[np.ndarray] = None,
    ):
        """Record a single packet for a sensor if recording is active."""
        if not self.is_recording or sensor_id not in self.records:
            return

        timestamp_ms = int((time.time() - self.start_time) * 1000)
        rec = self.records[sensor_id]

        if rec["conn_type"] == "mmwave":
            dop_arr = doppler.copy() if doppler is not None else np.zeros(len(data1), dtype=np.float32)
            inten_arr = intensity.copy() if intensity is not None else np.full(len(data1), 20.0, dtype=np.float32)
            rec["packets"].append((timestamp_ms, data1.copy(), dop_arr, inten_arr))
        else:
            rec["packets"].append(
                (timestamp_ms, data1.copy(), distances.copy() if distances is not None else status.copy())
            )

    def stop_recording(self) -> Optional[Path]:
        """Stop recording and write metadata and CSV data files according to export_data/README.md.

        Returns:
            Path to the output directory where files were saved, or None if not recording.
        """
        if not self.is_recording:
            return None

        self.is_recording = False
        export_dir = self.export_base_dir / self.timestamp_str
        export_dir.mkdir(parents=True, exist_ok=True)

        meta_filepath = export_dir / "sensors_meta_data.txt"

        meta_lines = []
        for sensor_id, data in self.records.items():
            device_name = data["device_name"]
            sda = data["sda"]
            conn_type = data["conn_type"]
            pos = data["position"]
            wxyz = data["wxyz"]

            # Filename formatting according to export_data/README.md (name.type.csv)
            sanitized_device = device_name.replace("/", "-").replace(":", "-")
            if conn_type.lower() == "mmwave":
                csv_filename = f"{sanitized_device}_{sda}.mmWave.csv"
            else:
                csv_filename = f"{sanitized_device}_{sda}.VL53L5CX.csv"

            csv_filepath = export_dir / csv_filename
            rel_csv_path = f"./{self.timestamp_str}/{csv_filename}"

            # Write sensor CSV file
            with open(csv_filepath, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                if conn_type.lower() == "mmwave":
                    writer.writerow(["Timestamp", "DataPoints", "Doppler", "Intensity"])
                    for ts_ms, pts, dop, inten in data["packets"]:
                        dp_str = "".join(f"({p[0]:.2f};{p[1]:.2f};{p[2]:.2f})" for p in pts)
                        dop_str = ";".join(f"{d:.2f}" for d in dop)
                        inten_str = ";".join(f"{i:.2f}" for i in inten)
                        writer.writerow([ts_ms, dp_str, dop_str, inten_str])
                else:
                    writer.writerow(["Timestamp", "Distance", "Status"])
                    for ts_ms, dists, stats in data["packets"]:
                        dist_str = ";".join(str(int(d)) for d in dists)
                        stat_str = ";".join(str(int(s)) for s in stats)
                        writer.writerow([ts_ms, dist_str, stat_str])

            # Metadata format (quaternion order x, y, z, w)
            rot_x, rot_y, rot_z, rot_w = wxyz[1], wxyz[2], wxyz[3], wxyz[0]

            meta_lines.append(f"{device_name}_{sda}")
            meta_lines.append(f"{conn_type}")
            meta_lines.append(f"{sda}")
            meta_lines.append(f"{pos[0]}, {pos[1]}, {pos[2]}")
            meta_lines.append(f"{rot_x}, {rot_y}, {rot_z}, {rot_w}")
            meta_lines.append(f"{rel_csv_path}")

        with open(meta_filepath, "w", encoding="utf-8") as f:
            f.write("\n".join(meta_lines) + "\n")

        logger.info("Exported recorded sensor data to %s", export_dir.resolve())
        return export_dir
