"""Utility module for lidar number crunching"""
from typing import List
import numpy as np

VERTICAL_ANGLES_16 = [
    -15.0,
    -13.0,
    -11.0,
    -9.0,
    -7.0,
    -5.0,
    -3.0,
    -1.0,
    1.0,
    3.0,
    5.0,
    7.0,
    9.0,
    11.0,
    13.0,
    15.0,
]


def sph2cart(azimuths: np.ndarray, elevations: np.ndarray, distances: np.ndarray):
    """Convert from spherical to cartestian coordinates

    Args:
        azimuths (np.ndarray): Azimuth angles of point
        elevations (np.ndarray): Elevation angles from ground plane of points
        distance (np.ndarray): Distances from origin to points

    Returns:
        (x, y, z): Cartesian coordinates of points in relation to origin
    """
    els, azs = np.meshgrid(elevations, azimuths)
    x_coords = distances * np.cos(els.flatten()) * np.cos(azs.flatten())
    y_coords = distances * np.cos(els.flatten()) * np.sin(azs.flatten())
    z_coords = distances * np.sin(els.flatten())
    return x_coords, y_coords, z_coords


def get_number_of_points(distances: bytes):
    """Return number of points in this CPC

    Args:
        distances (bytes): The bytes containing the distance values

    Returns:
        int: Number of points in this CPC
    """
    return len(distances) / 2


def get_number_of_azimuths(number_of_points: int, entries_per_azimuth: int):
    """Return number of azimuths in this CPC

    Args:
        number_of_points (int): The number of points in the CPC
        entries_per_azimuth (int): The number of vertical entries per azimuth

    Returns:
        int: Number of azimuths in this CPC
    """
    return number_of_points / entries_per_azimuth


def get_points(
    azimuths: List[float],
    elevations: List[float],
    binary_distances: bytes,
):
    """Extract the points from this CPC

    Args:
        azimuths (List[float]): The unique azimuths represented in this CPC
        elevations (List[float]): The unique elevations represented in this CPC
        binary_distances (bytes): The bytes containing the data values

    Returns:
        np.ndarray: The point cloud as a 2D np.ndarray with shape (n, 3)
    """
    points = []

    distances = np.frombuffer(
        binary_distances,
        dtype=">u2",
    ).astype(float)
    distances /= 100

    x_coords, y_coords, z_coords = sph2cart(azimuths, elevations, distances)

    points = np.column_stack([x_coords, -y_coords, z_coords])

    # Filter for usuable points
    norms = np.linalg.norm(points, axis=1)
    condition = np.logical_and(norms >= 0.5, norms <= 200)

    return points[condition]
