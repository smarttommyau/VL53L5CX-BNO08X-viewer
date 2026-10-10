#!/usr/bin/env python3
"""VL53L5CX and mmWave Point Cloud Viewer - Main application."""

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
from .export_data import DataExporter
from .filters import fit_plane, fit_plane_ransac
from .geometry import (
    CoordinateMethod,
    compute_zone_angles,
)
from .logging_config import setup_logging
from .mmwave_reader import MMWaveReader
from .network_reader import NetworkReader
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
    """Real-time point cloud viewer for VL53L5CX ToF and IWR6843 mmWave sensors."""

    def __init__(self, devices_config: dict[int, dict], baud: int = 115200):
        self.baud = baud
        self.devices_config = devices_config
        self.sensor_manager = SensorManager()
        self.data_readers: dict[str, object] = {}
        self.exporter = DataExporter()

        self.zone_angles = compute_zone_angles()

        # Initialize devices and pre-allocate expected sensors
        for idx, dev in sorted(devices_config.items()):
            dev_type = dev.get("device", "port").lower()
            sensor_count = dev.get("sensor_count", 1)

            if dev_type == "mmwave":
                cfg_port = dev.get("cfg_port", "/dev/ttyUSB0")
                data_port = dev.get("data_port", "/dev/ttyUSB1")
                cfg_file = dev.get("cfg_file", "sensor_profile/iwr6843AOP_example.cfg")
                device_name = f"{cfg_port}_{data_port}"

                self.sensor_manager.add_device(
                    deviceName=device_name,
                    sensor_count=sensor_count,
                    deviceType="mmwave",
                    cfg_file=cfg_file,
                    cfg_port=cfg_port,
                    data_port=data_port,
                )
                if device_name not in self.data_readers:
                    self.data_readers[device_name] = MMWaveReader(
                        cfg_port=cfg_port,
                        data_port=data_port,
                        cfg_file=cfg_file,
                    )
            elif dev_type == "wifi":
                ip = dev.get("ip", "192.168.1.100")
                net_port = dev.get("telnet_port", 2340)
                device_name = ip
                self.sensor_manager.add_device(
                    deviceName=device_name, sensor_count=sensor_count, deviceType="wifi"
                )
                if device_name not in self.data_readers:
                    self.data_readers[device_name] = NetworkReader(
                        host=ip, port=net_port
                    )
            else:  # "port" / USB mode
                port = dev.get("port", "/dev/cu.usbserial-0001")
                device_name = port
                self.sensor_manager.add_device(
                    deviceName=device_name, sensor_count=sensor_count, deviceType="port"
                )
                if device_name not in self.data_readers:
                    self.data_readers[device_name] = SerialReader(port, baud)

        # Backward compatibility properties
        first_reader = list(self.data_readers.values())[0] if self.data_readers else None
        self.serial_readers = self.data_readers
        self.serial_reader = first_reader

    def _initialize_sensor_viser(
        self,
        server: viser.ViserServer,
        sensor,
        index: int,
        total_count: int,
    ):
        """Initialize Viser 3D scene elements and interactive transform controls for a sensor."""
        spacing = getattr(config, "SENSOR_SPACING_M", 2.0)
        x_offset = (index - (total_count - 1) / 2.0) * spacing
        sensor.initial_pos = (x_offset, 0.0, 0.0)

        yaw_deg = (
            0.0
            if getattr(sensor, "deviceType", "").lower() == "mmwave"
            else config.TOF_BOARD.sensor_yaw_deg
        )
        sensor.initial_wxyz = _yaw_to_wxyz(yaw_deg)

        # Interactive 3D Transform Controls frame allowing position dragging and rotation
        sensor.frame_handle = server.scene.add_transform_controls(
            sensor.hierarchy_path,
            scale=0.08,
            position=sensor.initial_pos,
            wxyz=sensor.initial_wxyz,
        )

        @sensor.frame_handle.on_update
        def _on_transform_update(
            handle: viser.TransformControlsHandle, s=sensor
        ) -> None:
            s.update_gui_and_label()

        # Board mesh under sensor frame (using iwr6843aopevm.png texture for mmWave)
        assets_dir = Path(__file__).parent.parent / "assets"
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

        # 3D Label above sensor
        if getattr(sensor, "deviceType", "").lower() == "mmwave":
            cfg_name = Path(getattr(sensor, "cfg_file", "")).name
            label_text = f"Sensor {sensor.id} (mmWave)\nDevice: {sensor.deviceName}\nCFG: {cfg_name}"
        else:
            sda_str = str(sensor.sda) if sensor.sda is not None else "Pending..."
            label_text = f"Sensor {sensor.id}\nDevice: {sensor.deviceName}\nSDA: {sda_str}"

        sensor.label_handle = server.scene.add_label(
            f"{sensor.hierarchy_path}/label",
                text=label_text,
                position=(0.0, 0.0, -0.08),
                anchor="bottom-center"
        )

        # Initial zone rays (for VL53 sensors)
        if getattr(sensor, "deviceType", "").lower() != "mmwave":
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
            with server.gui.add_folder("Sensors Info"):
                with server.gui.add_folder(f"Sensor {sensor.id}") as folder:
                    sensor.gui_folder = folder
                    sensor.gui_device_text = server.gui.add_text(
                        "DeviceName", initial_value=sensor.deviceName, disabled=True
                    )
                    if getattr(sensor, "deviceType", "").lower() == "mmwave":
                        sensor.gui_cfg_text = server.gui.add_text(
                            "Config", initial_value=Path(sensor.cfg_file).name, disabled=True
                        )
                    else:
                        sda_str = str(sensor.sda) if sensor.sda is not None else "Pending..."
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
                        "Frequency (Hz)", initial_value="0.0", disabled=True
                    )
                    gizmo_cb = server.gui.add_checkbox(
                        "Show Gizmo", initial_value=True
                    )
                    reset_btn = server.gui.add_button("Reset Pose")

                    sensor.gui_gizmo_cb = gizmo_cb
                    sensor.gui_reset_btn = reset_btn

                    if self.exporter.is_recording:
                        if sensor.frame_handle is not None:
                            sensor.frame_handle.disable_axes = True
                            sensor.frame_handle.disable_sliders = True
                            sensor.frame_handle.disable_rotations = True
                        gizmo_cb.disabled = True
                        reset_btn.disabled = True

                    @gizmo_cb.on_update
                    def _on_gizmo_toggle(
                        event: viser.GuiEvent, s=sensor, cb=gizmo_cb
                    ) -> None:
                        if s.frame_handle is not None and not self.exporter.is_recording:
                            s.frame_handle.disable_axes = not cb.value
                            s.frame_handle.disable_sliders = not cb.value
                            s.frame_handle.disable_rotations = not cb.value

                    @reset_btn.on_click
                    def _on_reset_click(event: viser.GuiEvent, s=sensor) -> None:
                        if s.frame_handle is not None and not self.exporter.is_recording:
                            s.frame_handle.position = s.initial_pos
                            s.frame_handle.wxyz = s.initial_wxyz
                            s.update_gui_and_label()

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

        # sensors are setup dynamically in _initialize_sensor_viser, so we only setup global controls here

        with server.gui.add_folder("Recording / Export"):
            self.rec_status_text = server.gui.add_text(
                "Record Status", initial_value="Not Recording", disabled=True
            )
            self.rec_button = server.gui.add_button("Start Recording")

            @self.rec_button.on_click
            def _on_rec_click(event: viser.GuiEvent) -> None:
                if not self.exporter.is_recording:
                    # Start recording
                    active_sensors = self.sensor_manager.get_all_sensors()
                    self.exporter.start_recording(active_sensors)
                    self.rec_status_text.value = "Recording..."
                    self.rec_button.label = "Stop Recording"

                    # Disallow posture changing while recording (disable gizmo handles)
                    for sensor in active_sensors:
                        if sensor.frame_handle is not None:
                            sensor.frame_handle.disable_axes = True
                            sensor.frame_handle.disable_sliders = True
                            sensor.frame_handle.disable_rotations = True
                        if sensor.gui_gizmo_cb is not None:
                            sensor.gui_gizmo_cb.disabled = True
                        if sensor.gui_reset_btn is not None:
                            sensor.gui_reset_btn.disabled = True

                    modal = server.gui.add_modal("Recording Started")
                    with modal:
                        server.gui.add_markdown(
                            f"**Recording Started!**\n\nRecording sensor data for {len(active_sensors)} active sensor(s).\n\n*Note: Sensor posture controls are locked during recording.*"
                        )
                        ok_btn = server.gui.add_button("OK")

                        @ok_btn.on_click
                        def _(e, m=modal):
                            m.close()
                else:
                    # Stop recording and export data
                    saved_dir = self.exporter.stop_recording()
                    self.rec_status_text.value = "Not Recording"
                    self.rec_button.label = "Start Recording"

                    # Restore posture controls and gizmo handles based on checkbox
                    for sensor in self.sensor_manager.get_all_sensors():
                        if sensor.gui_gizmo_cb is not None:
                            sensor.gui_gizmo_cb.disabled = False
                            if sensor.frame_handle is not None:
                                enabled = sensor.gui_gizmo_cb.value
                                sensor.frame_handle.disable_axes = not enabled
                                sensor.frame_handle.disable_sliders = not enabled
                                sensor.frame_handle.disable_rotations = not enabled
                        elif sensor.frame_handle is not None:
                            sensor.frame_handle.disable_axes = False
                            sensor.frame_handle.disable_sliders = False
                            sensor.frame_handle.disable_rotations = False

                        if sensor.gui_reset_btn is not None:
                            sensor.gui_reset_btn.disabled = False

                    if saved_dir:
                        modal = server.gui.add_modal("Recording Saved")
                        with modal:
                            server.gui.add_markdown(
                                f"**Recording Saved Successfully!**\n\n**Export Location:**\n`{saved_dir.resolve()}`"
                            )
                            ok_btn = server.gui.add_button("OK")

                            @ok_btn.on_click
                            def _(e, m=modal):
                                m.close()

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
                        if getattr(sensor, "deviceType", "").lower() != "mmwave":
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
                    if getattr(sensor, "deviceType", "").lower() != "mmwave":
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

        # Collect pending packets from all data readers (serial, network, mmwave)
        for reader_id, reader in self.data_readers.items():
            packets = reader.get_pending_packets()
            for packet in packets:
                if isinstance(reader, MMWaveReader):
                    dev_name, points_3d, dists_mm, status_arr, sda, doppler_arr, intensity_arr = packet
                    sensor, newly_assigned = self.sensor_manager.process_packet(
                        dev_name, points_3d
                    )
                    sensor.update_data(
                        points_3d=points_3d,
                        distances=dists_mm,
                        status=status_arr,
                        doppler=doppler_arr,
                        intensity=intensity_arr,
                    )
                    if self.exporter.is_recording:
                        self.exporter.record_packet(
                            sensor.id,
                            data1=points_3d,
                            doppler=doppler_arr,
                            intensity=intensity_arr,
                        )
                else:  # ToF (SerialReader or NetworkReader)
                    dev_name, distances, status, sda = packet
                    sensor, newly_assigned = self.sensor_manager.process_packet(
                        dev_name, distances, status, None, sda
                    )
                    if self.exporter.is_recording:
                        self.exporter.record_packet(sensor.id, distances, status)

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

                # Convert sensor measurements to local points
                points_local, colors = sensor.get_display_data(
                    self.zone_angles, coord_method
                )

                if len(points_local) > 0:
                    if self.mapping_checkbox.value:
                        # Transform local points to world space using the sensor's current user-dragged 3D pose
                        if sensor.frame_handle is not None:
                            sensor_pos = np.array(sensor.frame_handle.position)
                            sensor_wxyz = sensor.frame_handle.wxyz
                        else:
                            spacing = getattr(config, "SENSOR_SPACING_M", 2.0)
                            all_sensors = self.sensor_manager.get_all_sensors()
                            idx = (
                                all_sensors.index(sensor)
                                if sensor in all_sensors
                                else 0
                            )
                            x_offset = (idx - (len(all_sensors) - 1) / 2.0) * spacing
                            sensor_pos = np.array([x_offset, 0.0, 0.0])
                            sensor_wxyz = _yaw_to_wxyz(
                                0.0
                                if getattr(sensor, "deviceType", "").lower() == "mmwave"
                                else config.TOF_BOARD.sensor_yaw_deg
                            )

                        sensor_rot = Rotation.from_quat([
                            sensor_wxyz[1],
                            sensor_wxyz[2],
                            sensor_wxyz[3],
                            sensor_wxyz[0],
                        ])

                        board_pos = (
                            config.MMWAVE_BOARD.world_position
                            if getattr(sensor, "deviceType", "").lower() == "mmwave"
                            else config.TOF_BOARD.world_position
                        )

                        world_points = (
                            sensor_rot.apply(points_local)
                            + np.array(board_pos)
                            + sensor_pos
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
                    if (
                        self.fit_plane_checkbox.value
                        and len(points_local) >= 3
                        and getattr(sensor, "deviceType", "").lower() != "mmwave"
                    ):
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

                # Update zone rays for sensor (if VL53)
                if getattr(sensor, "deviceType", "").lower() != "mmwave":
                    if self.show_rays_checkbox.value and self.clip_rays_checkbox.value:
                        sensor.rays_handles = update_sensor_rays(
                            server,
                            sensor.hierarchy_path,
                            self.zone_angles,
                            coord_method,
                            visible=True,
                            distances=sensor.distances,
                        )
                    else:
                        for ray in sensor.rays_handles:
                            ray.visible = self.show_rays_checkbox.value

                # Update GUI text fields and 3D scene label for this sensor
                sensor.update_gui_and_label()

    def run(self, host: str = "0.0.0.0", port: int = 8080):
        """Start the viewer."""
        for reader in self.data_readers.values():
            try:
                reader.connect()
                reader.start()
            except Exception as e:
                logger.error(
                    "Failed to connect reader for %s: %s",
                    getattr(reader, "deviceName", reader),
                    e,
                )

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
            for reader in self.data_readers.values():
                reader.stop()


def parse_args():
    """Parse command line arguments supporting multiple devices (--deviceX, --portX, --IPX, --CFG_portX, --DATA_portX, --ConfigX, --sensor-countX)."""
    parser = argparse.ArgumentParser(
        description="VL53L5CX and mmWave Point Cloud Viewer",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Device syntax:\n"
            "  Base aliases apply to device 1: --device, --port, --IP, --CFG_port, --DATA_port, --Config, --sensor-count\n"
            "  Indexed forms configure additional devices: --deviceX, --portX, --IPX, --CFG_portX, --DATA_portX, --ConfigX, --sensor-countX\n\n"
            "Examples:\n"
            "  python -m viewer --device1 port --port1 /dev/ttyACM0 --sensor-count1 2\n"
            "  python -m viewer --device1 wifi --IP1 192.168.1.100 --sensor-count1 1\n"
            "  python -m viewer --device1 mmwave --CFG_port1 /dev/ttyUSB0 --DATA_port1 /dev/ttyUSB1 --Config1 sensor_profile/iwr6843AOP_example.cfg\n"
        ),
    )

    parser.add_argument(
        "--device",
        dest="device_alias",
        default=None,
        help="Fallback device type for device 1 (port, wifi, or mmwave)",
    )

    parser.add_argument(
        "--port",
        "-p",
        dest="port_alias",
        default=None,
        help="Fallback serial port for device 1",
    )
    parser.add_argument(
        "--IP",
        "--ip",
        dest="ip_alias",
        default=None,
        help="Fallback IP address for WiFi device 1",
    )
    parser.add_argument(
        "--CFG_port",
        "--cfg_port",
        dest="cfg_port_alias",
        default="/dev/ttyUSB0",
        help="Fallback CLI config port for mmWave device 1",
    )
    parser.add_argument(
        "--DATA_port",
        "--data_port",
        dest="data_port_alias",
        default="/dev/ttyUSB1",
        help="Fallback data payload port for mmWave device 1",
    )
    parser.add_argument(
        "--Config",
        "--config",
        dest="cfg_file_alias",
        default="sensor_profile/iwr6843AOP_example.cfg",
        help="Fallback profile config file for mmWave device 1",
    )
    parser.add_argument(
        "--sensor-count",
        dest="sensor_count_alias",
        type=int,
        default=None,
        help="Fallback sensor count for device 1",
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
                "ip": "192.168.1.100",
                "telnet_port": 2340,
                "cfg_port": args.cfg_port_alias,
                "data_port": args.data_port_alias,
                "cfg_file": args.cfg_file_alias,
                "sensor_count": 1,
            }
        return devices_config[idx]

    tokens = sys.argv[1:]
    i = 0
    while i < len(tokens):
        token = tokens[i]

        m_dev = re.match(r"^--device(\d+)(?:=(.*))?$", token, re.IGNORECASE)
        if m_dev:
            idx = int(m_dev.group(1))
            val = m_dev.group(2)
            if val is None and i + 1 < len(tokens) and not tokens[i + 1].startswith("-"):
                val = tokens[i + 1]
                i += 1
            if val:
                get_device_entry(idx)["device"] = val.lower()
            i += 1
            continue

        m_port = re.match(r"^--port(\d+)(?:=(.*))?$", token, re.IGNORECASE)
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

        m_ip = re.match(r"^--ip(\d+)(?:=(.*))?$", token, re.IGNORECASE)
        if m_ip:
            idx = int(m_ip.group(1))
            val = m_ip.group(2)
            if val is None and i + 1 < len(tokens) and not tokens[i + 1].startswith("-"):
                val = tokens[i + 1]
                i += 1
            if val:
                entry = get_device_entry(idx)
                entry["ip"] = val
                entry["device"] = "wifi"
            i += 1
            continue

        m_cfg = re.match(r"^--cfg_?port(\d+)(?:=(.*))?$", token, re.IGNORECASE)
        if m_cfg:
            idx = int(m_cfg.group(1))
            val = m_cfg.group(2)
            if val is None and i + 1 < len(tokens) and not tokens[i + 1].startswith("-"):
                val = tokens[i + 1]
                i += 1
            if val:
                entry = get_device_entry(idx)
                entry["cfg_port"] = val
                if entry.get("device") == "port":
                    entry["device"] = "mmwave"
            i += 1
            continue

        m_data = re.match(r"^--data_?port(\d+)(?:=(.*))?$", token, re.IGNORECASE)
        if m_data:
            idx = int(m_data.group(1))
            val = m_data.group(2)
            if val is None and i + 1 < len(tokens) and not tokens[i + 1].startswith("-"):
                val = tokens[i + 1]
                i += 1
            if val:
                entry = get_device_entry(idx)
                entry["data_port"] = val
                if entry.get("device") == "port":
                    entry["device"] = "mmwave"
            i += 1
            continue

        m_cfgfile = re.match(r"^--config(\d+)(?:=(.*))?$", token, re.IGNORECASE)
        if m_cfgfile:
            idx = int(m_cfgfile.group(1))
            val = m_cfgfile.group(2)
            if val is None and i + 1 < len(tokens) and not tokens[i + 1].startswith("-"):
                val = tokens[i + 1]
                i += 1
            if val:
                entry = get_device_entry(idx)
                entry["cfg_file"] = val
                if entry.get("device") == "port":
                    entry["device"] = "mmwave"
            i += 1
            continue

        m_cnt = re.match(r"^--sensor-count(\d+)(?:=(.*))?$", token, re.IGNORECASE)
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
        if args.ip_alias:
            devices_config[1] = {
                "device": "wifi",
                "ip": args.ip_alias,
                "telnet_port": 2340,
                "sensor_count": args.sensor_count_alias or 1,
            }
        else:
            device_type = (args.device_alias or "port").lower()
            dev1_port = args.port_alias if args.port_alias is not None else "/dev/cu.usbserial-0001"
            devices_config[1] = {
                "device": device_type,
                "port": dev1_port,
                "ip": "192.168.1.100",
                "telnet_port": 2340,
                "sensor_count": args.sensor_count_alias or 1,
                "cfg_port": args.cfg_port_alias,
                "data_port": args.data_port_alias,
                "cfg_file": args.cfg_file_alias,
            }
            if device_type == "wifi":
                devices_config[1]["ip"] = args.ip_alias or devices_config[1]["ip"]
            elif device_type == "mmwave":
                devices_config[1]["cfg_port"] = args.cfg_port_alias
                devices_config[1]["data_port"] = args.data_port_alias
                devices_config[1]["cfg_file"] = args.cfg_file_alias
    elif args.ip_alias is not None and 1 in devices_config:
        devices_config[1]["ip"] = args.ip_alias
        devices_config[1]["device"] = "wifi"
    elif args.device_alias is not None and 1 in devices_config:
        devices_config[1]["device"] = args.device_alias.lower()
    elif args.port_alias is not None and 1 in devices_config:
        if "--port1" not in " ".join(sys.argv):
            devices_config[1]["port"] = args.port_alias

    if args.sensor_count_alias is not None and 1 in devices_config:
        devices_config[1]["sensor_count"] = args.sensor_count_alias

    return args, devices_config


def main():
    args, devices_config = parse_args()

    setup_logging(level=logging.DEBUG if args.debug else logging.INFO)

    viewer = VL53L5CXViewer(devices_config=devices_config, baud=args.baud)
    viewer.run(host=args.host, port=args.viser_port)


if __name__ == "__main__":
    main()
