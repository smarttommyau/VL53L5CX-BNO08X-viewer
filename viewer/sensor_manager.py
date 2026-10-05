"""sensor_manager.py - Manager for multiple VL53L5CX and mmWave sensors."""

import logging
from typing import Optional, Union

import numpy as np

from .iwr6843aopevm_sensor import IWR6843AOPEVMSensor
from .vl53l5cx_sensor import VL53L5CXSensor

logger = logging.getLogger("vl53l5cx_viewer.manager")


class SensorManager:
    """Manages multiple VL53L5CX and mmWave radar sensors.

    Features:
    - Pre-allocates sensors per device based on expected sensor count (--sensor-countX)
    - Sequential ID distribution starting from 1
    - Supports both ToF (VL53) and TI mmWave (IWR6843) sensor classes
    - Tracks active sensors and their states
    """

    def __init__(self):
        self._sensors: dict[int, Union[VL53L5CXSensor, IWR6843AOPEVMSensor]] = {}
        self._next_id: int = 1  # Next available ID

    def add_device(
        self,
        deviceName: str,
        sensor_count: int = 1,
        deviceType: str = "port",
        cfg_file: str = "sensor_profile/iwr6843AOP_example.cfg",
        cfg_port: str = "/dev/ttyUSB0",
        data_port: str = "/dev/ttyUSB1",
    ) -> list:
        """Pre-allocate sensor instances for a device."""
        created = []
        for _ in range(sensor_count):
            sensor = self.register_sensor(
                deviceName=deviceName,
                sda=None,
                deviceType=deviceType,
                cfg_file=cfg_file,
                cfg_port=cfg_port,
                data_port=data_port,
            )
            created.append(sensor)
        return created

    def register_sensor(
        self,
        deviceName: str,
        sda: Optional[int] = None,
        sdaNum: Optional[int] = None,
        deviceType: str = "port",
        cfg_file: str = "sensor_profile/iwr6843AOP_example.cfg",
        cfg_port: str = "/dev/ttyUSB0",
        data_port: str = "/dev/ttyUSB1",
    ) -> Union[VL53L5CXSensor, IWR6843AOPEVMSensor]:
        """Register a single new sensor instance for a device."""
        effective_sda = sda if sda is not None else sdaNum
        sensor_id = self._next_id
        self._next_id += 1

        if deviceType.lower() == "mmwave":
            sensor = IWR6843AOPEVMSensor(
                deviceName=deviceName,
                cfg_file=cfg_file,
                cfg_port=cfg_port,
                data_port=data_port,
                id=sensor_id,
                deviceType="mmwave",
            )
        else:
            sensor = VL53L5CXSensor(
                deviceName=deviceName,
                id=sensor_id,
                sda=effective_sda,
                deviceType=deviceType,
            )

        self._sensors[sensor_id] = sensor

        logger.info(
            "Registered new sensor: %s (Type=%s, ID=%d)",
            deviceName,
            deviceType,
            sensor_id,
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
        data1: np.ndarray,
        distances: Optional[np.ndarray] = None,
        status: Optional[np.ndarray] = None,
        sda: Optional[int] = None,
    ) -> tuple[Union[VL53L5CXSensor, IWR6843AOPEVMSensor], bool]:
        """Process incoming sensor data packet and assign SDA pin if unassigned."""
        device_sensors = [
            s for s in self._sensors.values() if s.deviceName == deviceName
        ]

        target_sensor = None
        newly_assigned = False

        if sda is not None:
            for s in device_sensors:
                if s.sda == sda:
                    target_sensor = s
                    break

            if target_sensor is None:
                for s in device_sensors:
                    if s.sda is None:
                        target_sensor = s
                        target_sensor.sda = sda
                        newly_assigned = True
                        break

            if target_sensor is None:
                target_sensor = self.register_sensor(deviceName, sda=sda)
                newly_assigned = True
        else:
            for s in device_sensors:
                if s.sda is None or s.sda == 0:
                    target_sensor = s
                    break

            if target_sensor is None and device_sensors:
                target_sensor = device_sensors[0]
            elif target_sensor is None:
                target_sensor = self.register_sensor(deviceName, sda=0)
                newly_assigned = True

        # Check if packet comes from mmWave sensor (data1 is 3D points array N x 3)
        if isinstance(target_sensor, IWR6843AOPEVMSensor):
            target_sensor.update_data(
                points_3d=data1, distances=distances, status=status
            )
        else:
            # VL53 sensor packet (data1 is distances array)
            target_sensor.update_data(distances=data1, status=distances if status is None else status)

        return target_sensor, newly_assigned

    def get_sensor_by_id(self, sensor_id: int):
        """Get sensor by unique ID."""
        return self._sensors.get(sensor_id)

    def get_sensor(self, deviceName: str):
        """Get first sensor matching device name."""
        for s in self._sensors.values():
            if s.deviceName == deviceName:
                return s
        return None

    def get_all_sensors(self) -> list:
        """Get all active sensors sorted by ID."""
        return sorted(self._sensors.values(), key=lambda s: s.id)

    def get_sensors_for_device(self, deviceName: str) -> list:
        """Get all sensors belonging to a specific device."""
        return [
            s
            for s in sorted(self._sensors.values(), key=lambda s: s.id)
            if s.deviceName == deviceName
        ]

    def get_active_count(self) -> int:
        """Return number of active sensors."""
        return len(self._sensors)

    def get_sensor_ids(self) -> list[int]:
        """Get list of all sensor IDs."""
        return [s.id for s in self.get_all_sensors()]
