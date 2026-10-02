#!/usr/bin/env python3
"""VL53L5CX Point Cloud Viewer - Main application."""

import argparse
import logging
from dataclasses import dataclass, field
from pathlib import Path
import re
import sys
import time

import numpy as np
import viser
from scipy.spatial.transform import Rotation

from . import config
from .filters import fit_plane, fit_plane_ransac
from .geometry import (
    CoordinateMethod,
    compute_zone_angles,
    distances_to_points,
    get_colors,
)
from .logging_config import setup_logging
from .scene import (
    _create_board_mesh,
    _yaw_to_wxyz,
    create_grid,
    update_sensor_rays,
)
from .sensor_manager import SensorManager
from .serial_reader import SerialReader
from .vl53l5cx_sensor import VL53L5CXSensor

logger = logging.getLogger("vl53l5cx_viewer.main")


@dataclass
class MappingState:
    """State for mapping mode point accumulation (world coordinates)."""

    accumulated_points: list[np.ndarray] = field(default_factory=list)
    accumulated_colors: list[np.ndarray] = field(default_factory=list)
    _clear_requested: bool = False

    def request_clear(self):
        """Request a clear - will be processed in the main loop."""
        self._clear_requested = True

    def process_clear_if_requested(self) -> bool:
        """Process pending clear request. Returns True if cleared."""
        if self._clear_requested:
            self._clear_requested = False
            self.accumulated_points.clear()
            self.accumulated_colors.clear()
            return True
        return False

    def add(self, points: np.ndarray, colors: np.ndarray):
        self.accumulated_points.append(points)
        self.accumulated_colors.append(colors)

    def get_display_data(self) -> tuple[np.ndarray, np.ndarray]:
        if not self.accumulated_points:
            return np.empty((0, 3), dtype=np.float32), np.empty((0, 3), dtype=np.uint8)
        if len(self.accumulated_points) == 1:
            return self.accumulated_points[0], self.accumulated_colors[0]
        return np.vstack(self.accumulated_points), np.vstack(self.accumulated_colors)

    def total_points(self) -> int:
        return sum(len(p) for p in self.accumulated_points)

    def downsample(self, voxel_size: float, max_points: int):
        if not self.accumulated_points:
            return
        all_points = np.vstack(self.accumulated_points)
        all_colors = np.vstack(self.accumulated_colors)
        all_points, all_colors = voxel_downsample(all_points, all_colors, voxel_size)
        if len(all_points) > max_points:
            all_points = all_points[-max_points:]
            all_colors = all_colors[-max_points:]
        self.accumulated_points.clear()
        self.accumulated_points.append(all_points)
        self.accumulated_colors.clear()
        self.accumulated_colors.append(all_colors)


def voxel_downsample(
    points: np.ndarray, colors: np.ndarray, voxel_size: float
) -> tuple[np.ndarray, np.ndarray]:
    if len(points) == 0:
        return points, colors
    voxel_indices = np.ascontiguousarray(np.floor(points / voxel_size).astype(np.int64))
    keys = voxel_indices.view(
        dtype=[("x", np.int64), ("y", np.int64), ("z", np.int64)]
    ).ravel()
    _, unique_idx = np.unique(keys, return_index=True)
    return points[unique_idx], colors[unique_idx]


class VL53L5CXViewer:
    """Real-time point cloud viewer for VL53L5CX ToF sensors."""

    def __init__(self, devices_config: dict[int, dict], baud: int = 115200):
        self.baud = baud
        self.devices_config = devices_config
        self.sensor_manager = SensorManager()
        self.serial_readers: dict[str, SerialReader] = {}

        self.zone_angles = compute_zone_angles()

        # Initialize devices and pre-allocate expected sensors
        for idx, dev in sorted(devices_config.items()):
            port = dev["port"]
            device_type = dev.get("device", "port")
            sensor_count = dev["sensor_count"]
            self.sensor_manager.add_device(
                deviceName=port, sensor_count=sensor_count, deviceType=device_type
            )
            if port not in self.serial_readers:
                self.serial_readers[port] = SerialReader(port, baud)

        # Legacy backward compatibility property
        first_port = list(self.serial_readers.keys())[0] if self.serial_readers else "/dev/null"
        self.serial_reader = self.serial_readers.get(first_port)

    def _initialize_sensor_viser(
        self,
        server: viser.ViserServer,
        sensor: VL53L5CXSensor,
        index: int,
        total_count: int,
    ):
        """Initialize Viser 3D scene elements and GUI sidebar folder for a sensor."""
        spacing = getattr(config, "SENSOR_SPACING_M", 0.08)
        x_offset = (index - (total_count - 1) / 2.0) * spacing
        sensor_pos = (x_offset, 0.0, 0.0)

        # 3D Sensor frame
        sensor.frame_handle = server.scene.add_frame(
            sensor.hierarchy_path,
            show_axes=True,
            axes_length=0.01,
            axes_radius=0.001,
            position=sensor_pos,
            wxyz=_yaw_to_wxyz(config.TOF_BOARD.sensor_yaw_deg),
        )

        # Board mesh under sensor frame
        assets_dir = Path(__file__).parent.parent / "assets"
        sensor.mesh_handle = _create_board_mesh(
            server,
            scene_path=f"{sensor.hierarchy_path}/mesh",
            board_config=config.TOF_BOARD,
            assets_dir=assets_dir,
        )

        # 3D Label above sensor
        sda_str = str(sensor.sda) if sensor.sda is not None else "Pending..."
        label_text = (
            f"Sensor {sensor.id}\nDevice: {sensor.deviceName}\nSDA: {sda_str}"
        )
        sensor.label_handle = server.scene.add_label(
            f"{sensor.hierarchy_path}/label",
            text=label_text,
            position=(0.0, -0.015, 0.025),
        )

        # Initial zone rays
        coord_method = CoordinateMethod.UNIFORM
        if hasattr(self, "coord_method_dropdown"):
            coord_method = next(
                m
                for m in CoordinateMethod
                if m.value == self.coord_method_dropdown.value
            )

        show_rays = (
            self.show_rays_checkbox.value
            if hasattr(self, "show_rays_checkbox")
            else True
        )
        sensor.rays_handles = update_sensor_rays(
            server,
            sensor.hierarchy_path,
            self.zone_angles,
            coord_method,
            visible=show_rays,
        )

        # Sidebar GUI folder (if dynamically created)
        if sensor.gui_folder is None:
            sda_str = str(sensor.sda) if sensor.sda is not None else "Pending..."
            with server.gui.add_folder("Sensors Info"):
                with server.gui.add_folder(f"Sensor {sensor.id}") as folder:
                    sensor.gui_folder = folder
                    sensor.gui_device_text = server.gui.add_text(
                        "DeviceName", initial_value=sensor.deviceName, disabled=True
                    )
                    sensor.gui_sda_text = server.gui.add_text(
                        "SDA", initial_value=sda_str, disabled=True
                    )
                    sensor.gui_status_text = server.gui.add_text(
                        "Status", initial_value="Waiting...", disabled=True
                    )
                    sensor.gui_freq_text = server.gui.add_text(
                        "Frequency (Hz)", initial_value="0.0", disabled=True
                    )

    def _setup_scene(self, server: viser.ViserServer):
        """Initialize the 3D scene."""
        server.scene.add_frame("/origin", axes_length=0.002, axes_radius=0.0001)
        create_grid(server)

        sensors = self.sensor_manager.get_all_sensors()
        total_count = len(sensors)
        for i, sensor in enumerate(sensors):
            self._initialize_sensor_viser(server, sensor, i, total_count)

    def _setup_gui(self, server: viser.ViserServer, mapping_state: MappingState):
        """Initialize GUI controls."""

        with server.gui.add_folder("Settings"):
            self.point_size_slider = server.gui.add_slider(
                "Point Size", min=0.001, max=0.020, step=0.001, initial_value=0.005
            )
            self.show_rays_checkbox = server.gui.add_checkbox(
                "Show Zone Rays", initial_value=True
            )
            self.clip_rays_checkbox = server.gui.add_checkbox(
                "Clip to Measurement", initial_value=False
            )

            @self.show_rays_checkbox.on_update
            def _on_show_rays_toggle(event: viser.GuiEvent) -> None:
                self.clip_rays_checkbox.disabled = not self.show_rays_checkbox.value

            @self.clip_rays_checkbox.on_update
            def _on_clip_rays_toggle(event: viser.GuiEvent) -> None:
                if not self.clip_rays_checkbox.value:
                    method = next(
                        m
                        for m in CoordinateMethod
                        if m.value == self.coord_method_dropdown.value
                    )
                    for sensor in self.sensor_manager.get_all_sensors():
                        sensor.rays_handles = update_sensor_rays(
                            server,
                            sensor.hierarchy_path,
                            self.zone_angles,
                            method,
                            visible=self.show_rays_checkbox.value,
                        )

            server.gui.add_markdown("---")
            self.coord_method_dropdown = server.gui.add_dropdown(
                "Coordinate Method",
                options=[m.value for m in CoordinateMethod],
                initial_value=CoordinateMethod.UNIFORM.value,
            )

            @self.coord_method_dropdown.on_update
            def _on_coord_method_change(event: viser.GuiEvent) -> None:
                method = next(
                    m
                    for m in CoordinateMethod
                    if m.value == self.coord_method_dropdown.value
                )
                for sensor in self.sensor_manager.get_all_sensors():
                    sensor.rays_handles = update_sensor_rays(
                        server,
                        sensor.hierarchy_path,
                        self.zone_angles,
                        method,
                        visible=self.show_rays_checkbox.value,
                    )

            server.gui.add_markdown("---")
            self.filter_checkbox = server.gui.add_checkbox(
                "Enable Filtering", initial_value=False
            )
            self.filter_strength_slider = server.gui.add_slider(
                "Filter Strength",
                min=0.0,
                max=1.0,
                step=0.05,
                initial_value=0.5,
                disabled=True,
            )

            @self.filter_checkbox.on_update
            def _on_filter_toggle(event: viser.GuiEvent) -> None:
                self.filter_strength_slider.disabled = not self.filter_checkbox.value
                if not self.filter_checkbox.value:
                    for sensor in self.sensor_manager.get_all_sensors():
                        sensor.temporal_filter.reset()

            server.gui.add_markdown("---")
            self.fit_plane_checkbox = server.gui.add_checkbox(
                "Fit Plane", initial_value=False
            )
            self.plane_method_dropdown = server.gui.add_dropdown(
                "Method",
                options=["Least Squares", "RANSAC"],
                initial_value="Least Squares",
                disabled=True,
            )
            self.ransac_threshold_slider = server.gui.add_slider(
                "RANSAC Threshold (mm)",
                min=1,
                max=50,
                step=1,
                initial_value=10,
                visible=False,
            )
            self.plane_error_text = server.gui.add_text(
                "Plane RMSE (mm)", initial_value="--"
            )

            @self.fit_plane_checkbox.on_update
            def _on_fit_plane_toggle(event: viser.GuiEvent) -> None:
                self.plane_method_dropdown.disabled = not self.fit_plane_checkbox.value
                self.ransac_threshold_slider.visible = (
                    self.fit_plane_checkbox.value
                    and self.plane_method_dropdown.value == "RANSAC"
                )
                if not self.fit_plane_checkbox.value:
                    self.plane_error_text.value = "--"

            @self.plane_method_dropdown.on_update
            def _on_plane_method_change(event: viser.GuiEvent) -> None:
                self.ransac_threshold_slider.visible = (
                    self.plane_method_dropdown.value == "RANSAC"
                )

        with server.gui.add_folder("Mapping"):
            self.mapping_checkbox = server.gui.add_checkbox(
                "Mapping Mode", initial_value=False
            )
            self.voxel_size_slider = server.gui.add_slider(
                "Voxel Size (mm)", min=5, max=50, step=5, initial_value=10
            )
            self.max_points_slider = server.gui.add_slider(
                "Max Points (k)", min=10, max=500, step=10, initial_value=100
            )
            self.point_count_text = server.gui.add_text("Points", initial_value="0")
            clear_button = server.gui.add_button("Clear Map")

            @clear_button.on_click
            def _on_clear_click(event: viser.GuiEvent) -> None:
                mapping_state.request_clear()

            @self.mapping_checkbox.on_update
            def _on_mapping_toggle(event: viser.GuiEvent) -> None:
                if self.mapping_checkbox.value:
                    for sensor in self.sensor_manager.get_all_sensors():
                        server.scene.remove_by_name(f"{sensor.hierarchy_path}/points")
                else:
                    mapping_state.request_clear()

    def _process_frame(
        self, server: viser.ViserServer, mapping_state: MappingState
    ):
        """Process incoming frame data from all connected sensors."""
        # Process pending clear request in mapping mode
        if mapping_state.process_clear_if_requested():
            self.point_count_text.value = "0"
            server.scene.remove_by_name("/map/points")

        coord_method = next(
            m
            for m in CoordinateMethod
            if m.value == self.coord_method_dropdown.value
        )

        # Collect pending packets from all serial readers
        for port, reader in self.serial_readers.items():
            packets = reader.get_pending_packets()
            for dev_name, distances, status, sda in packets:
                sensor, newly_assigned = self.sensor_manager.process_packet(
                    dev_name, distances, status, sda
                )

                if sensor.frame_handle is None:
                    all_sensors = self.sensor_manager.get_all_sensors()
                    idx = (
                        all_sensors.index(sensor)
                        if sensor in all_sensors
                        else len(all_sensors) - 1
                    )
                    self._initialize_sensor_viser(
                        server, sensor, idx, len(all_sensors)
                    )

                if self.filter_checkbox.value:
                    distances = sensor.temporal_filter.apply(
                        distances, self.filter_strength_slider.value
                    )

                # Convert sensor measurements to local points
                points_local, colors = sensor.get_display_data(
                    self.zone_angles, coord_method
                )

                if len(points_local) > 0:
                    if self.mapping_checkbox.value:
                        # Transform local points to world space
                        spacing = getattr(config, "SENSOR_SPACING_M", 0.08)
                        all_sensors = self.sensor_manager.get_all_sensors()
                        idx = (
                            all_sensors.index(sensor)
                            if sensor in all_sensors
                            else 0
                        )
                        x_offset = (idx - (len(all_sensors) - 1) / 2.0) * spacing

                        sensor_yaw = Rotation.from_euler(
                            "z", config.TOF_BOARD.sensor_yaw_deg, degrees=True
                        )
                        world_points = (
                            sensor_yaw.apply(points_local)
                            + np.array(config.TOF_BOARD.world_position)
                            + np.array([x_offset, 0.0, 0.0])
                        )
                        mapping_state.add(world_points, colors)

                        if (
                            mapping_state.total_points()
                            > config.DOWNSAMPLE_POINT_THRESHOLD
                            or len(mapping_state.accumulated_points)
                            > config.DOWNSAMPLE_BUFFER_THRESHOLD
                        ):
                            voxel_size_m = self.voxel_size_slider.value / 1000.0
                            max_pts = self.max_points_slider.value * 1000
                            mapping_state.downsample(voxel_size_m, max_pts)

                        display_points, display_colors = (
                            mapping_state.get_display_data()
                        )
                        self.point_count_text.value = f"{len(display_points):,}"

                        server.scene.add_point_cloud(
                            "/map/points",
                            points=display_points,
                            colors=display_colors,
                            point_size=self.point_size_slider.value,
                            point_shape="circle",
                        )
                    else:
                        # Add or update live point cloud for this sensor
                        server.scene.add_point_cloud(
                            f"{sensor.hierarchy_path}/points",
                            points=points_local,
                            colors=colors,
                            point_size=self.point_size_slider.value,
                            point_shape="circle",
                        )

                    # Plane fitting per sensor
                    if self.fit_plane_checkbox.value and len(points_local) >= 3:
                        if self.plane_method_dropdown.value == "RANSAC":
                            threshold_m = (
                                self.ransac_threshold_slider.value / 1000.0
                            )
                            plane_fit = fit_plane_ransac(
                                points_local, threshold=threshold_m
                            )
                        else:
                            plane_fit = fit_plane(points_local)

                        if plane_fit is not None:
                            pos, wxyz, size, rmse_mm = plane_fit
                            self.plane_error_text.value = f"{rmse_mm:.2f}"
                            server.scene.add_box(
                                f"{sensor.hierarchy_path}/plane",
                                dimensions=(size, size, 0.0001),
                                position=pos,
                                wxyz=wxyz,
                                color=(255, 255, 0),
                                opacity=0.5,
                            )

                # Update zone rays for sensor
                if self.show_rays_checkbox.value and self.clip_rays_checkbox.value:
                    sensor.rays_handles = update_sensor_rays(
                        server,
                        sensor.hierarchy_path,
                        self.zone_angles,
                        coord_method,
                        visible=True,
                        distances=distances,
                    )
                else:
                    for ray in sensor.rays_handles:
                        ray.visible = self.show_rays_checkbox.value

                # Update GUI text fields and 3D scene label for this sensor
                sensor.update_gui_and_label()

    def run(self, host: str = "0.0.0.0", port: int = 8080):
        """Start the viewer."""
        for reader in self.serial_readers.values():
            try:
                reader.connect()
                reader.start()
            except Exception as e:
                logger.error("Failed to connect to %s: %s", reader.port, e)

        server = viser.ViserServer(host=host, port=port)
        logger.info("Viser server started at http://localhost:%d", port)

        @server.on_client_connect
        def on_client_connect(client: viser.ClientHandle) -> None:
            client.camera.position = (0.0, -0.50, 0.50)
            client.camera.look_at = (0.0, 0.0, 0.0)
            client.camera.up = (0.0, 0.0, 1.0)
            client.camera.near = 0.001
            client.camera.fov = 0.35

        mapping_state = MappingState()
        self._setup_scene(server)
        self._setup_gui(server, mapping_state)

        try:
            while True:
                frame_start = time.time()
                self._process_frame(server, mapping_state)

                elapsed = time.time() - frame_start
                if elapsed < config.FRAME_TIME:
                    time.sleep(config.FRAME_TIME - elapsed)

        except KeyboardInterrupt:
            logger.info("Shutting down...")
        finally:
            for reader in self.serial_readers.values():
                reader.stop()


def parse_args():
    """Parse command line arguments supporting multiple devices (--deviceX, --portX, --sensor-countX)."""
    parser = argparse.ArgumentParser(description="VL53L5CX Point Cloud Viewer")

    parser.add_argument(
        "--port",
        "-p",
        dest="port_alias",
        default=None,
        help="Fallback serial port for device 1",
    )
    parser.add_argument(
        "--baud", "-b", type=int, default=115200, help="Baud rate (default: 115200)"
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

    args, unknown = parser.parse_known_args()

    devices_config: dict[int, dict] = {}

    def get_device_entry(idx: int) -> dict:
        if idx not in devices_config:
            devices_config[idx] = {
                "device": "port",
                "port": "/dev/cu.usbserial-0001",
                "sensor_count": 1,
            }
        return devices_config[idx]

    tokens = sys.argv[1:]
    i = 0
    while i < len(tokens):
        token = tokens[i]

        m_dev = re.match(r"^--device(\d+)(?:=(.*))?$", token)
        if m_dev:
            idx = int(m_dev.group(1))
            val = m_dev.group(2)
            if val is None and i + 1 < len(tokens) and not tokens[i + 1].startswith("-"):
                val = tokens[i + 1]
                i += 1
            if val:
                get_device_entry(idx)["device"] = val
            i += 1
            continue

        m_port = re.match(r"^--port(\d+)(?:=(.*))?$", token)
        if m_port:
            idx = int(m_port.group(1))
            val = m_port.group(2)
            if val is None and i + 1 < len(tokens) and not tokens[i + 1].startswith("-"):
                val = tokens[i + 1]
                i += 1
            if val:
                get_device_entry(idx)["port"] = val
            i += 1
            continue

        m_cnt = re.match(r"^--sensor-count(\d+)(?:=(.*))?$", token)
        if m_cnt:
            idx = int(m_cnt.group(1))
            val = m_cnt.group(2)
            if val is None and i + 1 < len(tokens) and not tokens[i + 1].startswith("-"):
                val = tokens[i + 1]
                i += 1
            if val and val.isdigit():
                get_device_entry(idx)["sensor_count"] = int(val)
            i += 1
            continue

        i += 1

    # Fallback if no device arguments were provided
    if not devices_config:
        dev1_port = args.port_alias if args.port_alias is not None else "/dev/cu.usbserial-0001"
        devices_config[1] = {
            "device": "port",
            "port": dev1_port,
            "sensor_count": 1,
        }
    elif args.port_alias is not None and 1 in devices_config:
        if "--port1" not in " ".join(sys.argv):
            devices_config[1]["port"] = args.port_alias

    return args, devices_config


def main():
    args, devices_config = parse_args()

    setup_logging(level=logging.DEBUG if args.debug else logging.INFO)

    viewer = VL53L5CXViewer(devices_config=devices_config, baud=args.baud)
    viewer.run(host=args.host, port=args.viser_port)


if __name__ == "__main__":
    main()
