"""Tests for geometry module."""

import numpy as np
import pytest

from viewer.geometry import compute_zone_angles, distances_to_points


class TestComputeZoneAngles:
    """Tests for zone angle computation."""

    def test_returns_correct_shape(self):
        """Should return arrays with 64 elements."""
        zone_angles = compute_zone_angles()

        assert zone_angles.tan_x.shape == (64,)
        assert zone_angles.tan_y.shape == (64,)
        assert zone_angles.ray_dir_x.shape == (64,)
        assert zone_angles.ray_dir_y.shape == (64,)
        assert zone_angles.ray_dir_z.shape == (64,)

    def test_ray_directions_normalized(self):
        """Ray directions should be unit vectors."""
        zone_angles = compute_zone_angles()

        norms = np.sqrt(
            zone_angles.ray_dir_x**2 +
            zone_angles.ray_dir_y**2 +
            zone_angles.ray_dir_z**2
        )

        np.testing.assert_allclose(norms, 1.0, atol=1e-6)


class TestDistancesToPoints:
    """Tests for distance to point conversion."""

    def test_converts_mm_to_meters(self):
        """Distances in mm should convert to meters for z-coordinate."""
        zone_angles = compute_zone_angles()
        distances = np.full(64, 1000.0, dtype=np.float32)  # 1000mm = 1m

        points = distances_to_points(distances, zone_angles)

        # Center zone should have z ≈ 1.0m
        center_idx = 27  # Approximately center
        assert 0.9 < points[center_idx, 2] < 1.1

    def test_output_shape(self):
        """Output should be Nx3 array."""
        zone_angles = compute_zone_angles()
        distances = np.ones(64, dtype=np.float32) * 500

        points = distances_to_points(distances, zone_angles)

        assert points.shape == (64, 3)
