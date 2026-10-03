"""Tests for network_reader module."""

import json
import socket
import threading
import time
import numpy as np
import pytest

from viewer.network_reader import NetworkReader
from viewer import config


class TestNetworkReader:
    """Tests for NetworkReader class."""

    def test_initialization(self):
        """Should initialize with host and port."""
        reader = NetworkReader("192.168.1.100", 2340)

        assert reader.host == "192.168.1.100"
        assert reader.port == 2340
        assert reader.deviceName == "192.168.1.100"
        assert not reader.running
        assert reader.data_fps == 0.0

    def test_reading_mock_tcp_stream(self):
        """Should connect to TCP server and parse streaming JSON packets."""
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.bind(("127.0.0.1", 0))
        _, port = server_sock.getsockname()
        server_sock.listen(1)

        def mock_server():
            conn, _ = server_sock.accept()
            d = [100] * 64
            s = [5] * 64
            pkt = json.dumps({"distances": d, "status": s, "sda": 4}) + "\n"
            for _ in range(3):
                conn.sendall(pkt.encode("utf-8"))
                time.sleep(0.05)
            conn.close()

        srv_thread = threading.Thread(target=mock_server)
        srv_thread.start()

        reader = NetworkReader("127.0.0.1", port)
        reader.connect()
        reader.start()

        time.sleep(0.2)
        packets = reader.get_pending_packets()

        reader.stop()
        server_sock.close()
        srv_thread.join()

        assert len(packets) >= 1
        dev_name, distances, status, sda = packets[0]
        assert dev_name == "127.0.0.1"
        assert len(distances) == 64
        assert len(status) == 64
        assert sda == 4
