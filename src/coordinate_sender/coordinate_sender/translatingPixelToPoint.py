import numpy as np
import cv2
from srvs_and_msgs.srv import SetBurnLength

# Camera parameters
scale = 10 / 824  # mm per pixel ≈ 0.012135
image_width = 4000
image_height = 3000

# Points: from camera mm → FluidNC mm
shift_x = 0.3  # Shift in mm to align with FluidNC coordinates
shift_y = -0.6  # Shift in mm to align with FluidNC coordinates
camera_mm_points = np.array([
    [-15.2 + shift_x, 11.1 + shift_y],
    [12.35 + shift_x, 11.5 + shift_y],
    [-15.5 + shift_x, -11.5 + shift_y],
    [12.75 + shift_x, -11.7 + shift_y]
], dtype=np.float32)
fluidnc_points = np.array([
    [280, 0],
    [0, 0],
    [280, 230],
    [0, 230]
], dtype=np.float32)

# Compute transform matrix (homography)
mm_to_fluidnc_matrix = cv2.getPerspectiveTransform(camera_mm_points, fluidnc_points)

def pixel_to_mm(u, v):
    """Converts pixel (u,v) to mm relative to center of image."""
    dx = u - image_width / 2
    dy = -(v - image_height / 2)
    x_mm = dx * scale
    y_mm = dy * scale
    return x_mm, y_mm

def mm_to_fluidnc_limited(x_mm, y_mm):
    pt = np.array([x_mm, y_mm, 1.0], dtype=np.float32)
    result = mm_to_fluidnc_matrix @ pt
    result /= result[2]
    x_fluidnc, y_fluidnc = result[0], result[1]

    # Define bounds from fluidnc_points
    x_min, x_max = 0, 280
    y_min, y_max = 0, 230

    if not (x_min <= x_fluidnc <= x_max) or not (y_min <= y_fluidnc <= y_max):
        return None
    return x_fluidnc, y_fluidnc

def pixel_to_fluidnc(u, v):
    x_mm, y_mm = pixel_to_mm(u, v)
    return mm_to_fluidnc_limited(x_mm, y_mm)