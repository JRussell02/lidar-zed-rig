"""Edges shared by the calibration checks: LiDAR depth edges and camera image edges."""
import cv2
import numpy as np
from ouster.sdk import core


def depth_edge_mask(info, rng_staggered, jump=0.3, horizontal_only=False):
    """Foreground side of every depth jump (> `jump` m) = object outlines, as a STAGGERED mask.

    Computed on the destaggered range image (neighbors are spatial neighbors there), then mapped
    back so each pixel keeps its measurement column (needed for per-column timing).
    horizontal_only: only jumps between beam rows, i.e. horizontal outlines like a box top.
    """
    rng = core.destagger(info, rng_staggered).astype(np.float64) / 1000
    fg = np.zeros(rng.shape, bool)
    for axis in ((0,) if horizontal_only else (0, 1)):
        d = np.diff(rng, axis=axis)
        a = rng.take(range(rng.shape[axis] - 1), axis=axis)
        b = rng.take(range(1, rng.shape[axis]), axis=axis)
        v = (a > 0) & (b > 0)
        if axis == 0:
            fg[:-1][v & (d > jump)] = True
            fg[1:][v & (-d > jump)] = True
        else:
            fg[:, :-1][v & (d > jump)] = True
            fg[:, 1:][v & (-d > jump)] = True
    return core.destagger(info, fg.astype(np.uint8), inverse=True).astype(bool) & (rng_staggered > 0)


def image_edge_distance(gray, horizontal_only=False):
    """Distance (px) from every pixel to the nearest Canny edge."""
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    canny = cv2.Canny(blur, 40, 120) if not horizontal_only else cv2.Canny(blur, 30, 90)
    if horizontal_only:
        gy = np.abs(cv2.Sobel(blur, cv2.CV_32F, 0, 1, ksize=3))
        gx = np.abs(cv2.Sobel(blur, cv2.CV_32F, 1, 0, ksize=3))
        canny = np.where((canny > 0) & (gy > 1.5 * gx), 255, 0).astype(np.uint8)
    return cv2.distanceTransform(255 - canny, cv2.DIST_L2, 5)


def project(points_cam, K, dist, shape, min_z=0.5, margin=2):
    """Project camera-frame points; returns (uv, depth) of those inside the image."""
    pc = points_cam[points_cam[:, 2] > min_z]
    if len(pc) == 0:
        return np.empty((0, 2)), np.empty(0)
    uv = cv2.projectPoints(pc, np.zeros(3), np.zeros(3), K, dist)[0].reshape(-1, 2)
    h, w = shape[:2]
    m = (uv[:, 0] >= margin) & (uv[:, 0] < w - margin) & (uv[:, 1] >= margin) & (uv[:, 1] < h - margin)
    return uv[m], pc[m, 2]


def to_camera(points_lidar, T_lidar_camera):
    """LiDAR-frame points -> camera frame."""
    T = np.linalg.inv(T_lidar_camera)
    return points_lidar @ T[:3, :3].T + T[:3, 3]
