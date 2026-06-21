
#!/usr/bin/env python3
"""
Computer vision utilities for blob detection.

Accepts a BGR numpy array (np.ndarray) directly — no pygame dependency.
On the simulator path, the simulator node converts its pygame Surface to
numpy before calling anything here. On the hardware path, cv_bridge delivers
numpy directly from the ROS sensor_msgs/Image topic.
"""
import cv2
import numpy as np
from typing import List, Dict


def find_blobs_hsv(img_bgr: np.ndarray,
                   hsv_ranges: List[tuple],
                   min_pixels: int) -> List[Dict]:
    """
    Finds contiguous blobs of pixels matching a set of HSV color ranges.

    Args:
        img_bgr:    BGR image as a numpy array (H x W x 3, dtype uint8).
                    This is the native format from cv_bridge and OpenCV.
        hsv_ranges: List of (lower, upper) HSV bound tuples.
                    Values expected as (H: 0-360, S: 0-100, V: 0-100).
        min_pixels: Minimum contour area in pixels to be considered a blob.

    Returns:
        List of dicts, each describing one blob:
            center_x, center_y  — centroid in pixel coordinates
            height, width       — bounding box dimensions
            min_x, max_x        — bounding box horizontal extent
            min_y, max_y        — bounding box vertical extent
            area                — contour area in pixels
    """
    # Convert BGR to HSV. We use HSV_FULL for a Hue range of 0-255,
    # which maps cleanly from our config's 0-360 range.
    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV_FULL)

    # Build a combined mask across all provided HSV ranges.
    combined_mask = None
    for lower_hsv, upper_hsv in hsv_ranges:
        lower = np.array([
            lower_hsv[0] * 255 / 360,
            lower_hsv[1] * 255 / 100,
            lower_hsv[2] * 255 / 100
        ])
        upper = np.array([
            upper_hsv[0] * 255 / 360,
            upper_hsv[1] * 255 / 100,
            upper_hsv[2] * 255 / 100
        ])
        mask = cv2.inRange(img_hsv, lower, upper)
        combined_mask = mask if combined_mask is None else cv2.bitwise_or(combined_mask, mask)

    if combined_mask is None:
        return []

    contours, _ = cv2.findContours(combined_mask,
                                   cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    blobs = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area >= min_pixels:
            x, y, w, h = cv2.boundingRect(cnt)
            blobs.append({
                'center_x': x + w / 2,
                'center_y': y + h / 2,
                'height':   h,
                'width':    w,
                'min_x':    x,
                'max_x':    x + w,
                'min_y':    y,
                'max_y':    y + h,
                'area':     area,
            })

    return blobs
