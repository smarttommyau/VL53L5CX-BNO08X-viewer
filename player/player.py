"""player.py - Player for replaying exported sensor data recordings."""

import argparse
import csv
import logging
from pathlib import Path
import re
import sys
import time

import numpy as np
import viser

from viewer import config
from viewer.geometry import CoordinateMethod, compute_zone_angles
from viewer.iwr6843aopevm_sensor import IWR6843AOPEVMSensor
from viewer.logging_config import setup_logging
from viewer.scene import (
    _create_board_mesh,
    create_grid,
    update_sensor_rays,
)
from viewer.sensor_manager import SensorManager

logger = logging.getLogger("vl53l5cx_viewer.player")

MMWAVE_PATTERN = re.compile(r"\(([-0-9.]+);([-0-9.]+);([-0-9.]+)\)")

def load_recording(export_dir: Path) -> list[dict]:
    """Parse sensors_meta_data.txt and CSV files from exported directory."""
    meta_filepath = export_dir / "sensors_meta_data.txt"
    if not meta_filepath.exists():
        raise FileNotFoundError(f"Metadata file not found: {meta_filepath}")

    with open(meta_filepath, "r", encoding="utf-8") as f:
        lines = [line.strip() for line in f if line.strip()]

    sensors_info = []
    i = 0
    while i + 5 < len(lines):
        s_name = lines[i]
        conn_type = lines[i + 1]
        sda = int(lines[i + 2])
        pos_parts = [float(x) for x in lines[i + 3].split(",")]
        rot_parts = [float(x) for x in lines[i + 4].split(",")]  # (x, y, z, w)
        csv_rel_path = lines[i + 5]
        i += 6

        # Convert quaternion (x, y, z, w) -> (w, x, y, z) for Viser wxyz
        wxyz = (rot_parts[3], rot_parts[0], rot_parts[1], rot_parts[2])
        position = (pos_parts[0], pos_parts[1], pos_parts[2])

        # Resolve CSV path
        csv_filename = Path(csv_rel_path).name
        csv_file = export_dir / csv_filename
        if not csv_file.exists():
            csv_file = Path(csv_rel_path)

        packets = []
        if csv_file.exists():
            with open(csv_file, "r", encoding="utf-8") as f:
                reader = csv.reader(f)
                header = next(reader, None)  # Skip header
                if conn_type.lower() == "mmwave":
                    for row in reader:
                        if len(row) >= 2:
                            ts = int(row[0])
                            dp_str = row[1]
                            matches = MMWAVE_PATTERN.findall(dp_str)
                            if matches:
                                pts = np.array(
                                    matches
                                    ,
                                    dtype=np.float32,
                                )
                            else:
                                pts = np.empty((0, 3), dtype=np.float32)
                            packets.append((ts, pts))
                else:
                    for row in reader:
                        if len(row) >= 3:
                            ts = int(row[0])
                            dists = np.fromstring(row[1], sep=";", dtype=np.float32)
                            stats = np.fromstring(row[2], sep=";", dtype=np.uint8)
                            packets.append((ts, dists, stats))

        sensors_info.append({
            "name": s_name,
            "conn_type": conn_type,
            "sda": sda,
            "position": position,
            "wxyz": wxyz,
            "packets": packets,
        })

    return sensors_info


class RecordedPlayer:
    """Replays exported sensor recording sessions in Viser."""

    def __init__(self, export_dir: Path, host: str = "0.0.0.0", port: int = 8080):
        self.export_dir = export_dir
        self.host = host
        self.port = port
        self.sensors_info = load_recording(export_dir)

        self.sensor_manager = SensorManager()
        self.zone_angles = compute_zone_angles()

        # Determine total duration
        self.max_time_ms = 0
        for s in self.sensors_info:
            if s["packets"]:
                self.max_time_ms = max(self.max_time_ms, s["packets"][-1][0])

        self.current_time_ms = 0.0
        self.is_playing = True
        self.playback_speed = 1.0
        self.loop_playback = True

        # Pre-register sensor instances
        for s in self.sensors_info:
            dev_name = s["name"].rsplit("_", 1)[0]
            sensor = self.sensor_manager.register_sensor(
                deviceName=dev_name, sda=s["sda"], deviceType=s["conn_type"]
            )
            sensor.initial_pos = s["position"]
            sensor.initial_wxyz = s["wxyz"]

    def _setup_scene(self, server: viser.ViserServer):
        """Setup stationary 3D scene elements (dragging disabled)."""
        server.scene.add_frame("/origin", axes_length=0.002, axes_radius=0.0001)
        create_grid(server)

        sensors = self.sensor_manager.get_all_sensors()
        assets_dir = Path(__file__).parent.parent / "assets"

        for sensor in sensors:
            # Add stationary frame (add_frame instead of add_transform_controls to disable move/drag)
            sensor.frame_handle = server.scene.add_frame(
                sensor.hierarchy_path,
                show_axes=True,
                axes_length=0.01,
                axes_radius=0.001,
                position=sensor.initial_pos,
                wxyz=sensor.initial_wxyz,
            )

            # Board mesh
            board_cfg = (
                config.MMWAVE_BOARD
                if getattr(sensor, "deviceType", "").lower() == "mmwave"
                else config.TOF_BOARD
            )
            sensor.mesh_handle = _create_board_mesh(
                server,
                scene_path=f"{sensor.hierarchy_path}/mesh",
                board_config=board_cfg,
                assets_dir=assets_dir,
            )

            # 3D Label
            sda_str = str(sensor.sda) if sensor.sda is not None else "N/A"
            type_str = "mmWave" if getattr(sensor, "deviceType", "").lower() == "mmwave" else "VL53"
            label_text = f"Sensor {sensor.id} ({type_str})\nDevice: {sensor.deviceName}\nSDA: {sda_str}"
            sensor.label_handle = server.scene.add_label(
                f"{sensor.hierarchy_path}/label",
                text=label_text,
                position=(0.0, 0.0, -0.08),
                anchor="bottom-center"
            )

            # Zone rays (only for VL53)
            if getattr(sensor, "deviceType", "").lower() != "mmwave":
                sensor.rays_handles = update_sensor_rays(
                    server,
                    sensor.hierarchy_path,
                    self.zone_angles,
                    CoordinateMethod.UNIFORM,
                    visible=True,
                )

    def _setup_gui(self, server: viser.ViserServer):
        """Setup playback controls and sensor info GUI folders."""
        with server.gui.add_folder("Playback Controls"):
            self.play_btn = server.gui.add_button("Pause")
            self.speed_slider = server.gui.add_slider(
                "Speed", min=0.1, max=5.0, step=0.1, initial_value=1.0
            )
            self.timeline_slider = server.gui.add_slider(
                "Timeline (ms)",
                min=0,
                max=max(self.max_time_ms, 1),
                step=10,
                initial_value=0,
            )
            self.time_text = server.gui.add_text(
                "Time", initial_value="00:00.0 / 00:00.0", disabled=True
            )
            self.loop_cb = server.gui.add_checkbox("Loop", initial_value=True)

            @self.play_btn.on_click
            def _on_play_toggle(e):
                self.is_playing = not self.is_playing
                self.play_btn.label = "Pause" if self.is_playing else "Play"

            @self.speed_slider.on_update
            def _on_speed_change(e):
                self.playback_speed = self.speed_slider.value

            @self.timeline_slider.on_update
            def _on_timeline_change(e):
                if not self.is_playing:
                    self.current_time_ms = float(self.timeline_slider.value)

            @self.loop_cb.on_update
            def _on_loop_change(e):
                self.loop_playback = self.loop_cb.value

        with server.gui.add_folder("Settings"):
            self.show_fov_cb = server.gui.add_checkbox(
                "Show Boundary Lines (FoV)", initial_value=True
            )

            @self.show_fov_cb.on_update
            def _on_show_fov_toggle(e):
                for sensor in self.sensor_manager.get_all_sensors():
                    if hasattr(sensor, "boundary_handles") and sensor.boundary_handles:
                        for line in sensor.boundary_handles:
                            line.visible = self.show_fov_cb.value

        with server.gui.add_folder("Sensors Info"):
            for sensor in self.sensor_manager.get_all_sensors():
                sda_str = str(sensor.sda) if sensor.sda is not None else "N/A"
                with server.gui.add_folder(f"Sensor {sensor.id}") as folder:
                    sensor.gui_folder = folder
                    sensor.gui_device_text = server.gui.add_text(
                        "DeviceName", initial_value=sensor.deviceName, disabled=True
                    )
                    sensor.gui_sda_text = server.gui.add_text(
                        "SDA", initial_value=sda_str, disabled=True
                    )
                    sensor.gui_pos_text = server.gui.add_text(
                        "Position (m)",
                        initial_value=f"X: {sensor.initial_pos[0]:.2f}, Y: {sensor.initial_pos[1]:.2f}, Z: {sensor.initial_pos[2]:.2f}",
                        disabled=True,
                    )
                    sensor.gui_status_text = server.gui.add_text(
                        "Status", initial_value="Waiting...", disabled=True
                    )
                    sensor.gui_freq_text = server.gui.add_text(
                        "Frequency (Hz)", initial_value="--", disabled=True
                    )

    def run(self):
        """Start the replay server loop."""
        server = viser.ViserServer(host=self.host, port=self.port)
        logger.info("Player Viser server started at http://localhost:%d", self.port)

        @server.on_client_connect
        def on_client_connect(client: viser.ClientHandle) -> None:
            client.camera.position = (0.0, -0.50, 0.50)
            client.camera.look_at = (0.0, 0.0, 0.0)
            client.camera.up = (0.0, 0.0, 1.0)
            client.camera.near = 0.001
            client.camera.fov = 0.35

        self._setup_scene(server)
        self._setup_gui(server)

        try:
            while True:
                frame_start = time.time()

                if self.is_playing:
                    self.current_time_ms += (
                        config.FRAME_TIME * self.playback_speed * 1000.0
                    )
                    if self.current_time_ms > self.max_time_ms:
                        if self.loop_playback:
                            self.current_time_ms = 0.0
                        else:
                            self.current_time_ms = float(self.max_time_ms)
                            self.is_playing = False
                            self.play_btn.label = "Play"

                    self.timeline_slider.value = int(self.current_time_ms)

                cur_sec = self.current_time_ms / 1000.0
                tot_sec = self.max_time_ms / 1000.0
                self.time_text.value = f"{int(cur_sec // 60):02d}:{cur_sec % 60:04.1f} / {int(tot_sec // 60):02d}:{tot_sec % 60:04.1f}"

                # Render closest packet frame for each sensor
                for info in self.sensors_info:
                    packets = info["packets"]
                    if not packets:
                        continue

                    closest_pkt = min(
                        packets, key=lambda p: abs(p[0] - self.current_time_ms)
                    )

                    dev_name = info["name"].rsplit("_", 1)[0]
                    sensor = self.sensor_manager.get_sensor(dev_name)
                    if sensor:
                        if isinstance(sensor, IWR6843AOPEVMSensor) or getattr(sensor, "deviceType", "").lower() == "mmwave":
                            _, points_3d = closest_pkt
                            sensor.update_data(points_3d=points_3d)
                        else:
                            _, distances, status = closest_pkt
                            sensor.update_data(distances=distances, status=status)

                        points_local, colors = sensor.get_display_data(
                            self.zone_angles, CoordinateMethod.UNIFORM
                        )

                        if len(points_local) > 0:
                            server.scene.add_point_cloud(
                                f"{sensor.hierarchy_path}/points",
                                points=points_local,
                                colors=colors,
                                point_size=0.005,
                                point_shape="circle",
                            )

                        sensor.update_gui_and_label()

                elapsed = time.time() - frame_start
                if elapsed < config.FRAME_TIME:
                    time.sleep(config.FRAME_TIME - elapsed)

        except KeyboardInterrupt:
            logger.info("Stopping player...")


def main():
    parser = argparse.ArgumentParser(description="VL53L5CX / mmWave Recording Player")
    parser.add_argument(
        "export_dir",
        type=str,
        help="Path to exported recording directory (e.g. export_data/2026-10-04_15-30-00/)",
    )
    parser.add_argument(
        "--host", default="0.0.0.0", help="Viser server host (default: 0.0.0.0)"
    )
    parser.add_argument(
        "--viser-port", type=int, default=8080, help="Viser server port (default: 8080)"
    )
    parser.add_argument(
        "--debug", "-d", action="store_true", help="Enable debug logging"
    )

    args = parser.parse_args()

    setup_logging(level=logging.DEBUG if args.debug else logging.INFO)

    export_path = Path(args.export_dir)
    if not export_path.exists() or not export_path.is_dir():
        logger.error("Export directory not found: %s", export_path)
        sys.exit(1)

    player = RecordedPlayer(export_path, host=args.host, port=args.viser_port)
    player.run()


if __name__ == "__main__":
    main()
