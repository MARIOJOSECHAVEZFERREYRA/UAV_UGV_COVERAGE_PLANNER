"""Shared geometry primitives used across algorithms."""

import math


def pts_equal(a, b, tol=1e-6):
    """Check if two 2D points are equal within tolerance."""
    return abs(a[0] - b[0]) < tol and abs(a[1] - b[1]) < tol


def dist(a, b):
    """Euclidean distance between two 2D points."""
    return math.hypot(b[0] - a[0], b[1] - a[1])


def path_length(coords):
    """Total length of a polyline given as [(x,y), ...]."""
    if not coords or len(coords) < 2:
        return 0.0
    total = 0.0
    for i in range(len(coords) - 1):
        total += math.hypot(
            coords[i + 1][0] - coords[i][0],
            coords[i + 1][1] - coords[i][1],
        )
    return total
