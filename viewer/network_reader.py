"""Network socket communication with VL53L5CX sensor via ESP32 telnet server (port 2340)."""

import json
import logging
import math
import socket
import threading
import time
from typing import Optional

import numpy as np

from . import config

logger = logging.getLogger("vl53l5cx_viewer.network")


class NetworkReader:
    """Background thread for reading sensor data over WiFi/Telnet socket (port 2340)."""

    def __init__(self, host: str, port: int = 2340):
        self.host = host
        self.port = port
        self.deviceName = host  # Device name used for sensor tracking (IP address)

        self.sock: Optional[socket.socket] = None
        self.sock_file = None
        self.running = False

        # Data storage
        self.distances = np.zeros(config.NUM_ZONES, dtype=np.float32)
        self.status = np.zeros(config.NUM_ZONES, dtype=np.uint8)
        self._packets_queue: list[tuple[str, np.ndarray, np.ndarray, Optional[int]]] = []
        self._data_lock = threading.Lock()

        # FPS tracking
        self._frame_count = 0
        self._last_fps_time = time.time()
        self._data_fps = 0.0

        self._thread: Optional[threading.Thread] = None

    @property
    def data_fps(self) -> float:
        """Current data frame rate from network socket."""
        with self._data_lock:
            return self._data_fps

    def connect(self):
        """Connect to remote ESP32 telnet server."""
        logger.info("Connecting to WiFi device at %s:%d...", self.host, self.port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(5.0)
        self.sock.connect((self.host, self.port))
        self.sock.settimeout(1.0)
        self.sock_file = self.sock.makefile("r", encoding="utf-8", errors="ignore")
        logger.info("Network connected to %s:%d", self.host, self.port)

    def start(self):
        """Start the background network reader thread."""
        if self._thread is not None:
            return

        self.running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop the reader thread and close socket connection."""
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
            self.sock_file = None

        if self._thread:
            self._thread.join(timeout=1)
            self._thread = None

    def get_data(self) -> tuple[np.ndarray, np.ndarray]:
        """Get a copy of the latest distance and status data."""
        with self._data_lock:
            return self.distances.copy(), self.status.copy()

    def get_pending_packets(
        self,
    ) -> list[tuple[str, np.ndarray, np.ndarray, Optional[int]]]:
        """Get all pending packets received since last call.

        Returns:
            List of (deviceName, distances_array, status_array, sda_pin)
        """
        with self._data_lock:
            packets = self._packets_queue[:]
            self._packets_queue.clear()
            return packets

    def _validate_distances(self, distances: list) -> bool:
        """Validate distance values are within expected range."""
        for d in distances:
            if not isinstance(d, (int, float)):
                return False
            if math.isnan(d) or math.isinf(d):
                return False
        return True

    def _reconnect(self) -> bool:
        """Attempt to reconnect to remote ESP32 telnet server."""
        try:
            if self.sock:
                try:
                    self.sock.close()
                except Exception:
                    pass
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((self.host, self.port))
            self.sock.settimeout(1.0)
            self.sock_file = self.sock.makefile("r", encoding="utf-8", errors="ignore")
            logger.info("Network reconnected to %s:%d", self.host, self.port)
            return True
        except (socket.error, OSError) as e:
            logger.debug("Network reconnection failed on %s:%d: %s", self.host, self.port, e)
            return False

    def _read_loop(self):
        """Background thread loop to read TCP stream line by line."""
        logger.info("Network reader thread started for %s:%d", self.host, self.port)
        while self.running:
            try:
                if self.sock_file:
                    line = self.sock_file.readline()
                    if not line:
                        raise socket.error("Connection closed by remote host")

                    line_str = line.strip()
                    if line_str.startswith("{"):
                        try:
                            data = json.loads(line_str)
                            if "distances" in data and "status" in data:
                                distances = data["distances"]
                                status = data["status"]
                                sda = data.get("sda")

                                # Skip if status/distances are not list objects
                                if not isinstance(distances, list) or not isinstance(status, list):
                                    logger.debug("Received non-measurement JSON packet: %s", data)
                                    continue

                                if (
                                    len(distances) != config.NUM_ZONES
                                    or len(status) != config.NUM_ZONES
                                ):
                                    logger.warning(
                                        "Invalid array lengths: distances=%d, status=%d (expected %d)",
                                        len(distances),
                                        len(status),
                                        config.NUM_ZONES,
                                    )
                                    continue

                                if not self._validate_distances(distances):
                                    logger.warning(
                                        "Invalid distance values detected (NaN/Inf)"
                                    )
                                    continue

                                distances_arr = np.array(distances, dtype=np.float32)
                                status_arr = np.array(status, dtype=np.uint8)

                                with self._data_lock:
                                    self.distances = distances_arr
                                    self.status = status_arr
                                    self._packets_queue.append(
                                        (self.deviceName, distances_arr, status_arr, sda)
                                    )

                                self._frame_count += 1
                                now = time.time()
                                elapsed = now - self._last_fps_time
                                if elapsed >= 1.0:
                                    with self._data_lock:
                                        self._data_fps = self._frame_count / elapsed
                                    self._frame_count = 0
                                    self._last_fps_time = now
                        except json.JSONDecodeError as e:
                            logger.debug("JSON decode error: %s", e)
            except (socket.error, OSError, TimeoutError) as e:
                if not self.running:
                    break
                logger.warning(
                    "Network connection lost on %s:%d: %s", self.host, self.port, e
                )
                with self._data_lock:
                    self._data_fps = 0.0
                while self.running:
                    if self._reconnect():
                        break
                    time.sleep(2)
