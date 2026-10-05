"""mmwave_reader.py - Serial communicator and binary parser for TI IWR6843AOPEVM mmWave radar."""

import logging
from pathlib import Path
import struct
import threading
import time
from typing import Optional

import numpy as np
import serial

logger = logging.getLogger("vl53l5cx_viewer.mmwave")

# TI mmWave Magic Word
MAGIC_WORD = bytes([2, 1, 4, 3, 6, 5, 8, 7])


class MMWaveReader:
    """Communicates with TI IWR6843AOPEVM mmWave sensor over dual serial ports.

    - CFG_port (115200 baud): Sends CLI configuration profile commands sequentially, waiting for Done / Skipped response.
    - DATA_port (921600 baud): Receives binary payload data frames containing 3D point cloud measurements.
    """

    def __init__(
        self,
        cfg_port: str,
        data_port: str,
        cfg_file: str,
        cfg_baud: int = 115200,
        data_baud: int = 921600,
    ):
        self.cfg_port = cfg_port
        self.data_port = data_port
        self.cfg_file = cfg_file
        self.cfg_baud = cfg_baud
        self.data_baud = data_baud

        # Device name appended from both serial ports
        self.deviceName = f"{cfg_port}_{data_port}"

        self.cfg_serial: Optional[serial.Serial] = None
        self.data_serial: Optional[serial.Serial] = None
        self.running = False

        # Data storage
        self.points_3d = np.empty((0, 3), dtype=np.float32)
        self.distances = np.empty((0,), dtype=np.float32)
        self.status = np.empty((0,), dtype=np.uint8)
        self._packets_queue: list[
            tuple[str, np.ndarray, np.ndarray, np.ndarray, Optional[int]]
        ] = []
        self._data_lock = threading.Lock()

        # FPS tracking
        self._frame_count = 0
        self._last_fps_time = time.time()
        self._data_fps = 0.0

        self._thread: Optional[threading.Thread] = None

    @property
    def data_fps(self) -> float:
        """Current data frame rate from mmWave data port."""
        with self._data_lock:
            return self._data_fps

    def _send_config_file(self):
        """Read CLI config profile and send commands sequentially over CFG_port, waiting explicitly for Done or Skipped."""
        cfg_path = Path(self.cfg_file)
        if not cfg_path.exists():
            logger.error("mmWave profile config file not found: %s", self.cfg_file)
            return

        logger.info(
            "Sending mmWave configuration from %s to %s...",
            self.cfg_file,
            self.cfg_port,
        )

        with open(cfg_path, "r", encoding="utf-8") as f:
            lines = [
                line.strip()
                for line in f
                if line.strip() and not line.strip().startswith("%")
            ]

        if not self.cfg_serial or not self.cfg_serial.is_open:
            logger.error("CFG_port %s is not open", self.cfg_port)
            return

        # Wake up CLI prompt
        self.cfg_serial.reset_input_buffer()
        self.cfg_serial.write(b"\r\n")
        time.sleep(0.2)
        while self.cfg_serial.in_waiting:
            prompt_line = (
                self.cfg_serial.readline().decode("utf-8", errors="ignore").strip()
            )
            # if prompt_line:
                # logger.info("CFG_port prompt: %s", prompt_line)
    
        for idx, line in enumerate(lines):
            # logger.info("CFG_port [%d/%d] sending: %s", idx + 1, len(lines), line)

            # Reset input buffer before sending command to ignore stale echos
            self.cfg_serial.reset_input_buffer()
            self.cfg_serial.write((line + "\r\n").encode("utf-8"))

            start_wait = time.time()
            done_received = False
            wait_timeout = 5.0 if "sensorStart" in line else 2.0

            while time.time() - start_wait < wait_timeout:
                if self.cfg_serial.in_waiting:
                    resp_line = (
                        self.cfg_serial.readline()
                        .decode("utf-8", errors="ignore")
                        .strip()
                    )
                    if resp_line:
                        # Ignore command echo
                        if resp_line == line or resp_line == line + "\r":
                            continue

                        # logger.info("CFG_port response for [%s]: %s", line, resp_line)

                        if "Done" in resp_line or "Skipped" in resp_line:
                            done_received = True
                            break
                        elif "Error" in resp_line:
                            logger.error(
                                "CFG_port error on command '%s': %s", line, resp_line
                            )
                            done_received = True
                            break

                time.sleep(0.01)

            if not done_received:
                logger.warning("Timeout waiting for 'Done' on command: %s", line)

            time.sleep(0.05)

        logger.info("mmWave configuration sequence completed.")

    def connect(self):
        """Open dual serial ports and transmit configuration commands."""
        logger.info(
            "Connecting mmWave sensor - CFG: %s (%d), DATA: %s (%d)...",
            self.cfg_port,
            self.cfg_baud,
            self.data_port,
            self.data_baud,
        )

        try:
            self.cfg_serial = serial.Serial(
                self.cfg_port, self.cfg_baud, timeout=1
            )
            time.sleep(0.5)
        except Exception as e:
            logger.error("Failed to open CFG_port %s: %s", self.cfg_port, e)

        try:
            self.data_serial = serial.Serial(
                self.data_port, self.data_baud, timeout=1
            )
            time.sleep(0.5)
            self.data_serial.reset_input_buffer()
        except Exception as e:
            logger.error("Failed to open DATA_port %s: %s", self.data_port, e)

        # Transmit configuration file to sensor
        self._send_config_file()

    def start(self):
        """Start background reader thread for DATA_port payload parsing."""
        if self._thread is not None:
            return

        self.running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop background thread and close serial connections."""
        self.running = False
        if self.cfg_serial:
            try:
                self.cfg_serial.close()
            except Exception:
                pass
            self.cfg_serial = None

        if self.data_serial:
            try:
                self.data_serial.close()
            except Exception:
                pass
            self.data_serial = None

        if self._thread:
            self._thread.join(timeout=1)
            self._thread = None

    def get_data(self) -> tuple[np.ndarray, np.ndarray]:
        """Get a copy of latest distance and status data."""
        with self._data_lock:
            return self.distances.copy(), self.status.copy()

    def get_pending_packets(self):
        """Get pending packet tuples received since last call.

        Returns:
            List of (deviceName, points_3d, distances_mm, status_array, None)
        """
        with self._data_lock:
            packets = self._packets_queue[:]
            self._packets_queue.clear()
            return packets

    def _read_loop(self):
        """Read binary data stream from DATA_port, parse header and TLV payloads."""
        logger.info("mmWave data reader thread started for %s", self.data_port)
        buffer = bytearray()

        while self.running:
            try:
                if self.data_serial and self.data_serial.is_open:
                    bytes_to_read = max(self.data_serial.in_waiting, 1)
                    raw_data = self.data_serial.read(bytes_to_read)
                    if raw_data:
                        buffer.extend(raw_data)

                        while len(buffer) >= 40:
                            # Search for magic word
                            magic_idx = buffer.find(MAGIC_WORD)
                            if magic_idx == -1:
                                # Keep last 7 bytes in case magic word is split
                                buffer = buffer[-7:]
                                break

                            if magic_idx > 0:
                                buffer = buffer[magic_idx:]

                            if len(buffer) < 40:
                                break

                            # Parse 40-byte header
                            (
                                version,
                                totalPacketLen,
                                platform,
                                frameNumber,
                                timeCpuCycles,
                                numDetectedObj,
                                numTLVs,
                                subFrameNumber,
                            ) = struct.unpack("<IIIIIIII", buffer[8:40])

                            # Validate packet length sanity
                            if (
                                totalPacketLen < 40
                                or totalPacketLen > 65536
                                or numTLVs > 50
                            ):
                                logger.debug(
                                    "Invalid mmWave header parameters (len=%d, tlvs=%d), re-syncing buffer...",
                                    totalPacketLen,
                                    numTLVs,
                                )
                                buffer = buffer[8:]
                                continue

                            if len(buffer) < totalPacketLen:
                                break  # Wait for complete packet

                            packet = buffer[:totalPacketLen]
                            buffer = buffer[totalPacketLen:]

                            logger.debug(
                                "mmWave packet #%d received: len=%d, numTLVs=%d, numDetectedObj_hdr=%d",
                                frameNumber,
                                totalPacketLen,
                                numTLVs,
                                numDetectedObj,
                            )

                            # Parse TLVs
                            tlv_offset = 40
                            points_list = []

                            for t_idx in range(min(numTLVs, 50)):
                                if tlv_offset + 8 > len(packet):
                                    break

                                tlv_type, tlv_len = struct.unpack(
                                    "<II", packet[tlv_offset : tlv_offset + 8]
                                )
                                tlv_payload_offset = tlv_offset + 8
                                tlv_offset += 8 + tlv_len  # Advance to next TLV in packet

                                if tlv_payload_offset + tlv_len > len(packet):
                                    logger.debug(
                                        "TLV #%d length %d exceeds packet bounds",
                                        t_idx + 1,
                                        tlv_len,
                                    )
                                    break

                                tlv_payload = packet[
                                    tlv_payload_offset : tlv_payload_offset + tlv_len
                                ]

                                if tlv_type == 1:  # MMWDEMO_OUTPUT_MSG_DETECTED_POINTS
                                    # SDK 3.x format: Each detected point is 16 bytes (float32 X, Y, Z, Doppler)
                                    if len(tlv_payload) % 16 == 0 and len(tlv_payload) > 0:
                                        num_pts = len(tlv_payload) // 16
                                        for i in range(num_pts):
                                            pt_x, pt_y, pt_z, dop = struct.unpack(
                                                "<ffff",
                                                tlv_payload[i * 16 : (i + 1) * 16],
                                            )
                                            # Convert TI radar coords (X right, Y depth, Z height) -> sensor local frame (X right, Y down, Z forward)
                                            points_list.append([pt_x, -pt_z, pt_y])
                                    elif len(tlv_payload) >= 4:
                                        # Legacy SDK 1.x Q-format structure (descriptor + int16 points)
                                        num_obj, q_fmt = struct.unpack(
                                            "<HH", tlv_payload[:4]
                                        )

                                        if q_fmt <= 20:
                                            q_scale = float(2**q_fmt)
                                            obj_offset = 4

                                            for _ in range(min(num_obj, 1000)):
                                                if obj_offset + 12 > len(tlv_payload):
                                                    break

                                                (
                                                    r_idx,
                                                    d_idx,
                                                    peak,
                                                    x_raw,
                                                    y_raw,
                                                    z_raw,
                                                ) = struct.unpack(
                                                    "<HHHhhh",
                                                    tlv_payload[
                                                        obj_offset : obj_offset + 12
                                                    ],
                                                )
                                                obj_offset += 12

                                                pt_x = float(x_raw) / q_scale
                                                pt_y = float(y_raw) / q_scale
                                                pt_z = float(z_raw) / q_scale

                                                points_list.append([pt_x, -pt_z, pt_y])

                            # if points_list:
                                # logger.info(
                                #     "mmWave parsed %d 3D points in frame #%d",
                                #     len(points_list),
                                #     frameNumber,
                                # )

                            points_3d = (
                                np.array(points_list, dtype=np.float32)
                                if points_list
                                else np.empty((0, 3), dtype=np.float32)
                            )

                            if len(points_3d) > 0:
                                dists_mm = np.linalg.norm(points_3d, axis=1) * 1000.0
                                stats_arr = np.full(len(points_3d), 5, dtype=np.uint8)
                            else:
                                dists_mm = np.empty((0,), dtype=np.float32)
                                stats_arr = np.empty((0,), dtype=np.uint8)

                            with self._data_lock:
                                self.points_3d = points_3d
                                self.distances = dists_mm
                                self.status = stats_arr
                                self._packets_queue.append(
                                    (
                                        self.deviceName,
                                        points_3d,
                                        dists_mm,
                                        stats_arr,
                                        None,
                                    )
                                )

                            self._frame_count += 1
                            now = time.time()
                            elapsed = now - self._last_fps_time
                            if elapsed >= 1.0:
                                with self._data_lock:
                                    self._data_fps = self._frame_count / elapsed
                                self._frame_count = 0
                                self._last_fps_time = now

            except Exception as e:
                if not self.running:
                    break
                logger.warning("mmWave data read exception on %s: %s", self.data_port, e)
                time.sleep(0.5)
