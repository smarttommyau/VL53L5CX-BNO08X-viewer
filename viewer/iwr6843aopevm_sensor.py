"""iwr6843aopevm_sensor.py - Sensor class for managing TI IWR6843AOPEVM mmWave sensors."""

import logging
from pathlib import Path
import time
from typing import Optional

import numpy as np
import viser

from . import config
from .filters import TemporalFilter

logger = logging.getLogger("vl53l5cx_viewer.mmwave_sensor")


class IWR6843AOPEVMSensor:
    """Represents a single TI IWR6843AOPEVM mmWave radar sensor instance.

    Attributes:
        deviceName: Identifier string formed by appending ports (e.g., "/dev/ttyUSB0_/dev/ttyUSB1")
        deviceType: Device type descriptor ("mmwave")
        cfg_file: Path to mmWave CLI config file (e.g., "sensor_profile/iwr6843AOP_example.cfg")
        cfg_port: CLI command serial port (baud 115200)
        data_port: Binary payload data serial port (baud 921600)
        id: Unique sensor ID assigned sequentially (1, 2, ...)
    """

    def __init__(
        self,
        deviceName: str,
        cfg_file: str = "sensor_profile/iwr6843AOP_example.cfg",
        cfg_port: str = "/dev/ttyUSB0",
        data_port: str = "/dev/ttyUSB1",
        id: int = 1,
        deviceType: str = "mmwave",
    ):
        self.deviceName: str = deviceName
        self.deviceType: str = deviceType
        self.cfg_file: str = cfg_file
        self.cfg_port: str = cfg_port
        self.data_port: str = data_port
        self._id: int = id
        self.sda: Optional[int] = None  # mmWave has no SDA pin

        self._active: bool = True
        self.last_seen: float = 0.0

        # Pose in 3D world space
        self.initial_pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.initial_wxyz: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)

        # Data storage
        self.points_3d: Optional[np.ndarray] = None  # N x 3 numpy array of (X, Y, Z) in meters
        self.distances: Optional[np.ndarray] = None  # Range values in mm
        self.status: Optional[np.ndarray] = None
        self.doppler: Optional[np.ndarray] = None  # Doppler velocity in m/s
        self.intensity: Optional[np.ndarray] = None  # SNR / intensity in dB
        self.temporal_filter = TemporalFilter()

        # Data FPS tracking
        self.data_fps: float = 0.0
        self._frame_count: int = 0
        self._last_fps_time: float = time.time()

        # Viser handles (3D scene)
        self.frame_handle: Optional[viser.TransformControlsHandle] = None
        self.mesh_handle: Optional[viser.MeshHandle] = None
        self.points_handle: Optional[viser.PointCloudHandle] = None
        self.rays_handles: list = []  # Empty for mmWave
        self.boundary_handles: list = []  # Empty for mmWave
        self.label_handle: Optional[viser.LabelHandle] = None
        self.plane_handle = None

        # GUI handles (sidebar)
        self.gui_folder = None
        self.gui_device_text = None
        self.gui_cfg_text = None
        self.gui_sda_text = None
        self.gui_pos_text = None
        self.gui_status_text = None
        self.gui_freq_text = None
        self.gui_gizmo_cb = None
        self.gui_reset_btn = None

    @property
    def id(self) -> int:
        """Get the sensor's unique ID."""
        return self._id

    @id.setter
    def id(self, value: int):
        self._id = value

    @property
    def sdaNum(self) -> Optional[int]:
        """Returns None as mmWave sensors do not use SDA pin numbers."""
        return None

    @property
    def hierarchy_path(self) -> str:
        """Get the full hierarchy path in viser."""
        return f"/sensor_{self._id}"

    def activate(self, sensor_id: int):
        """Mark sensor as active with given ID."""
        self._id = sensor_id
        self._active = True
        logger.info(
            "mmWave Sensor %s (ID=%d, Config=%s) activated",
            self.deviceName,
            self._id,
            self.cfg_file,
        )

    def deactivate(self):
        """Mark sensor as inactive."""
        if not self._active:
            return

        logger.info("mmWave Sensor %s (ID=%d) deactivated", self.deviceName, self._id)
        self._active = False

    def update_data(
        self,
        points_3d: np.ndarray,
        distances: Optional[np.ndarray] = None,
        status: Optional[np.ndarray] = None,
        doppler: Optional[np.ndarray] = None,
        intensity: Optional[np.ndarray] = None,
    ):
        """Update 3D point cloud, doppler, and intensity data for mmWave sensor."""
        self.points_3d = (
            points_3d.copy() if points_3d is not None else np.empty((0, 3), dtype=np.float32)
        )
        n_pts = len(self.points_3d)

        if doppler is not None and len(doppler) == n_pts:
            self.doppler = doppler.copy()
        else:
            self.doppler = np.zeros((n_pts,), dtype=np.float32)

        if intensity is not None and len(intensity) == n_pts:
            self.intensity = intensity.copy()
        else:
            self.intensity = np.full((n_pts,), 20.0, dtype=np.float32)

        if distances is not None:
            self.distances = distances.copy()
        elif n_pts > 0:
            # Calculate Euclidean distance in mm
            self.distances = np.linalg.norm(self.points_3d, axis=1) * 1000.0
        else:
            self.distances = np.empty((0,), dtype=np.float32)

        if status is not None:
            self.status = status.copy()
        else:
            self.status = np.full(n_pts, 5, dtype=np.uint8)

        self.last_seen = time.time()

        # Calculate per-sensor FPS
        self._frame_count += 1
        now = time.time()
        elapsed = now - self._last_fps_time
        if elapsed >= 1.0:
            self.data_fps = self._frame_count / elapsed
            self._frame_count = 0
            self._last_fps_time = now

    def get_display_data(
        self, zone_angles=None, coord_method=None
    ) -> tuple[np.ndarray, np.ndarray]:
        """Get valid 3D points and solid red colors for mmWave visualization."""
        if self.points_3d is None or len(self.points_3d) == 0:
            return np.empty((0, 3), dtype=np.float32), np.empty((0, 3), dtype=np.uint8)

        colors = np.tile(
            np.array([255, 0, 0], dtype=np.uint8), (len(self.points_3d), 1)
        )
        return self.points_3d.astype(np.float32), colors

    def update_gui_and_label(self):
        """Update GUI sidebar text controls and 3D scene label."""
        if self.gui_cfg_text is not None:
            self.gui_cfg_text.value = Path(self.cfg_file).name

        if self.gui_device_text is not None:
            self.gui_device_text.value = self.deviceName

        if self.frame_handle is not None and self.gui_pos_text is not None:
            pos = self.frame_handle.position
            self.gui_pos_text.value = (
                f"X: {pos[0]:.2f}, Y: {pos[1]:.2f}, Z: {pos[2]:.2f}"
            )

        num_pts = len(self.points_3d) if self.points_3d is not None else 0
        status_str = f"Points: {num_pts}"

        if self.gui_status_text is not None:
            self.gui_status_text.value = status_str

        if self.gui_freq_text is not None:
            self.gui_freq_text.value = f"{self.data_fps:.1f}"

        if self.label_handle is not None:
            cfg_name = Path(self.cfg_file).name
            self.label_handle.text = (
                f"Sensor {self.id} (mmWave)\nDevice: {self.deviceName}\nCFG: {cfg_name}"
            )


# Alias for convenience
MMWaveSensor = IWR6843AOPEVMSensor
