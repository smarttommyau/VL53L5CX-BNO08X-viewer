"""sensor_manager.py - Manager for multiple VL53L5CX sensors."""

import logging
from typing import Optional

import numpy as np

from .vl53l5cx_sensor import VL53L5CXSensor

logger = logging.getLogger("vl53l5cx_viewer.manager")


class SensorManager:
    """Manages multiple VL53L5CX sensors.

    Features:
    - Pre-allocates sensors per device based on expected sensor count (--sensor-countX)
    - Sequential ID distribution starting from 1
    - Auto-assigns SDA pins when JSON packets containing "sda" arrive
    - Tracks active sensors and their states
    """

    def __init__(self):
        self._sensors: dict[int, VL53L5CXSensor] = {}  # sensor_id -> sensor
        self._next_id: int = 1                          # Next available ID

    def add_device(
        self, deviceName: str, sensor_count: int = 1, deviceType: str = "port"
    ) -> list[VL53L5CXSensor]:
        """Pre-allocate sensor instances for a device.

        Args:
            deviceName: Serial port identifier (e.g., "/dev/ttyUSB0")
            sensor_count: Number of expected sensors on this device
            deviceType: Device type descriptor (default: "port")

        Returns:
            List of created sensor instances
        """
        created = []
        for _ in range(sensor_count):
            sensor = self.register_sensor(
                deviceName=deviceName, sda=None, deviceType=deviceType
            )
            created.append(sensor)
        return created

    def register_sensor(
        self,
        deviceName: str,
        sda: Optional[int] = None,
        sdaNum: Optional[int] = None,
        deviceType: str = "port",
    ) -> VL53L5CXSensor:
        """Register a single new sensor instance for a device."""
        effective_sda = sda if sda is not None else sdaNum
        sensor_id = self._next_id
        self._next_id += 1

        sensor = VL53L5CXSensor(
            deviceName=deviceName, id=sensor_id, sda=effective_sda, deviceType=deviceType
        )
        self._sensors[sensor_id] = sensor

        logger.info(
            "Registered new sensor: %s (Type=%s, ID=%d, SDA=%s)",
            deviceName,
            deviceType,
            sensor_id,
            str(effective_sda),
        )

        return sensor

    def unregister_sensor(self, sensor_id: int):
        """Remove a sensor by ID."""
        if sensor_id in self._sensors:
            sensor = self._sensors.pop(sensor_id)
            sensor.deactivate()
            logger.info("Unregistered sensor: %s (ID=%d)", sensor.deviceName, sensor_id)

    def process_packet(
        self,
        deviceName: str,
        distances: np.ndarray,
        status: np.ndarray,
        sda: Optional[int] = None,
    ) -> tuple[VL53L5CXSensor, bool]:
        """Process incoming sensor data packet and assign SDA pin if unassigned.

        Args:
            deviceName: The serial port / device name
            distances: 64 distance measurements in mm
            status: 64 target status flags
            sda: SDA pin number from JSON packet (or None)

        Returns:
            Tuple of (sensor_instance, is_sda_newly_assigned)
        """
        device_sensors = [
            s for s in self._sensors.values() if s.deviceName == deviceName
        ]

        target_sensor: Optional[VL53L5CXSensor] = None
        newly_assigned = False

        if sda is not None:
            # 1. Look for a sensor for this device that already has this SDA assigned
            for s in device_sensors:
                if s.sda == sda:
                    target_sensor = s
                    break

            # 2. If not found, assign to the first unassigned sensor for this device
            if target_sensor is None:
                for s in device_sensors:
                    if s.sda is None:
                        target_sensor = s
                        target_sensor.sda = sda
                        newly_assigned = True
                        logger.info(
                            "Assigned SDA pin %d to sensor ID=%d (device=%s)",
                            sda,
                            s.id,
                            deviceName,
                        )
                        break

            # 3. If no unassigned sensor exists, dynamically register a new one
            if target_sensor is None:
                target_sensor = self.register_sensor(deviceName, sda=sda)
                newly_assigned = True
        else:
            # Handle packets without SDA pin information
            for s in device_sensors:
                if s.sda is None or s.sda == 0:
                    target_sensor = s
                    break

            if target_sensor is None and device_sensors:
                target_sensor = device_sensors[0]
            elif target_sensor is None:
                target_sensor = self.register_sensor(deviceName, sda=0)
                newly_assigned = True

        target_sensor.update_data(distances, status)
        return target_sensor, newly_assigned

    def update_from_packet(self, deviceName: str, sdaNum: Optional[int]):
        """Legacy helper for updating or registering sensor from packet."""
        device_sensors = [
            s for s in self._sensors.values() if s.deviceName == deviceName
        ]
        if not device_sensors:
            self.register_sensor(deviceName, sda=sdaNum)
        else:
            if sdaNum is not None:
                for s in device_sensors:
                    if s.sda is None or s.sda == sdaNum:
                        s.sda = sdaNum
                        break

    def update_sensor_data(
        self, deviceName: str, distances: np.ndarray, status: np.ndarray
    ):
        """Update distance and status data for a specific sensor."""
        dummy_sensor, _ = self.process_packet(deviceName, distances, status)

    def get_sensor_by_id(self, sensor_id: int) -> Optional[VL53L5CXSensor]:
        """Get sensor by unique ID."""
        return self._sensors.get(sensor_id)

    def get_sensor(self, deviceName: str) -> Optional[VL53L5CXSensor]:
        """Get first sensor matching device name."""
        for s in self._sensors.values():
            if s.deviceName == deviceName:
                return s
        return None

    def get_all_sensors(self) -> list[VL53L5CXSensor]:
        """Get all active sensors sorted by ID."""
        return sorted(self._sensors.values(), key=lambda s: s.id)

    def get_sensors_for_device(self, deviceName: str) -> list[VL53L5CXSensor]:
        """Get all sensors belonging to a specific device."""
        return [
            s
            for s in sorted(self._sensors.values(), key=lambda s: s.id)
            if s.deviceName == deviceName
        ]

    def get_sensor_data(
        self, deviceName: str
    ) -> Optional[tuple[np.ndarray, np.ndarray]]:
        """Get distance and status data for a specific device's sensor."""
        sensor = self.get_sensor(deviceName)
        if sensor is not None and sensor.distances is not None and sensor.status is not None:
            return (sensor.distances.copy(), sensor.status.copy())
        return None

    def get_active_count(self) -> int:
        """Return number of active sensors."""
        return len(self._sensors)

    def get_sensor_ids(self) -> list[int]:
        """Get list of all sensor IDs."""
        return [s.id for s in self.get_all_sensors()]
