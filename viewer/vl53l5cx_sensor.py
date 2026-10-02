"""vl53l5cx_sensor.py - Sensor class for managing individual VL53L5CX sensors."""

import logging
import time
from typing import Optional

import numpy as np
import viser

from . import config
from .filters import TemporalFilter
from .scene import _yaw_to_wxyz, update_sensor_rays

logger = logging.getLogger("vl53l5cx_viewer.sensor")


class VL53L5CXSensor:
    """Represents a single VL53L5CX sensor instance.

    Attributes:
        deviceName: Serial port identifier (e.g., "/dev/ttyUSB0")
        deviceType: Device type descriptor (e.g., "port")
        sda: SDA pin number from JSON (None until assigned)
        id: Unique sensor ID assigned sequentially (1, 2, ...)
    """

    def __init__(
        self,
        deviceName: str,
        id: int = 1,
        sda: Optional[int] = None,
        sdaNum: Optional[int] = None,
        deviceType: str = "port",
    ):
        self.deviceName: str = deviceName
        self.deviceType: str = deviceType
        self._id: int = id
        self.sda: Optional[int] = sda if sda is not None else sdaNum

        self._active: bool = True
        self.last_seen: float = 0.0

        # Data storage
        self.distances: Optional[np.ndarray] = None
        self.status: Optional[np.ndarray] = None
        self.temporal_filter = TemporalFilter()

        # Data FPS tracking
        self.data_fps: float = 0.0
        self._frame_count: int = 0
        self._last_fps_time: float = time.time()

        # Viser handles (3D scene)
        self.frame_handle: Optional[viser.FrameHandle] = None
        self.mesh_handle: Optional[viser.MeshHandle] = None
        self.points_handle: Optional[viser.PointCloudHandle] = None
        self.rays_handles: list = []
        self.label_handle: Optional[viser.LabelHandle] = None
        self.plane_handle = None

        # GUI handles (sidebar)
        self.gui_folder = None
        self.gui_device_text = None
        self.gui_sda_text = None
        self.gui_status_text = None
        self.gui_freq_text = None

    @property
    def id(self) -> int:
        """Get the sensor's unique ID."""
        return self._id

    @id.setter
    def id(self, value: int):
        self._id = value

    @property
    def sdaNum(self) -> Optional[int]:
        """Backward compatibility property for sda pin number."""
        return self.sda

    @sdaNum.setter
    def sdaNum(self, value: Optional[int]):
        self.sda = value

    @property
    def hierarchy_path(self) -> str:
        """Get the full hierarchy path in viser."""
        return f"/sensor_{self._id}"

    def activate(self, sensor_id: int):
        """Mark sensor as active with given ID."""
        self._id = sensor_id
        self._active = True
        logger.info(
            "Sensor %s (ID=%d, SDA=%s) activated",
            self.deviceName,
            self._id,
            str(self.sda),
        )

    def deactivate(self):
        """Mark sensor as inactive and cleanup visualization."""
        if not self._active:
            return

        logger.info("Sensor %s (ID=%d) deactivated", self.deviceName, self._id)
        self._active = False

    def update_data(self, distances: np.ndarray, status: np.ndarray):
        """Update distance and status data for this sensor."""
        self.distances = distances.copy()
        self.status = status.copy()
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
        self, zone_angles, coord_method
    ) -> tuple[np.ndarray, np.ndarray]:
        """Get valid points and colors for visualization."""
        if self.distances is None or self.status is None:
            return np.empty((0, 3), dtype=np.float32), np.empty((0, 3), dtype=np.uint8)

        from .geometry import CoordinateMethod, distances_to_points, get_colors

        points_local = distances_to_points(self.distances, zone_angles, coord_method)
        colors = get_colors(self.distances, self.status)

        valid_mask = (self.status == 5) & (self.distances >= config.MIN_RANGE_MM)

        if np.any(valid_mask):
            return points_local[valid_mask].astype(np.float32), colors[valid_mask]

        return np.empty((0, 3), dtype=np.float32), np.empty((0, 3), dtype=np.uint8)

    def update_gui_and_label(self):
        """Update GUI sidebar text controls and 3D scene label."""
        sda_str = str(self.sda) if self.sda is not None else "Pending..."

        if self.gui_sda_text is not None:
            self.gui_sda_text.value = sda_str

        if self.gui_device_text is not None:
            self.gui_device_text.value = self.deviceName

        if self.distances is not None and self.status is not None:
            valid_mask = (self.status == 5) & (self.distances >= config.MIN_RANGE_MM)
            if np.any(valid_mask):
                valid_dist = self.distances[valid_mask]
                status_str = f"Range: {valid_dist.min():.0f}-{valid_dist.max():.0f}mm"
            else:
                status_str = "No valid targets"
        else:
            status_str = "Waiting for data..."

        if self.gui_status_text is not None:
            self.gui_status_text.value = status_str

        if self.gui_freq_text is not None:
            self.gui_freq_text.value = f"{self.data_fps:.1f}"

        if self.label_handle is not None:
            self.label_handle.text = (
                f"Sensor {self.id}\nDevice: {self.deviceName}\nSDA: {sda_str}"
            )
