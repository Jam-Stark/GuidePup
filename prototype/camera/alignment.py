"""Project calibrated native depth into RGB; preserve native depth separately.

Pixel-area splatting and a nearest-depth buffer follow depth-to-color alignment:
no RGB stretching, cross-edge interpolation, scene geometry or hole filling.
"""
import numpy as np


def align_depth_to_rgb(depth_z_m, depth_intrinsics, depth_camera_to_base,
                       rgb_intrinsics, rgb_camera_to_base, rgb_shape, valid_range_m):
    depth = np.asarray(depth_z_m)
    near, far = valid_range_m
    yy, xx = np.nonzero(np.isfinite(depth) & (depth > near) & (depth < far))
    zz = depth[yy, xx]
    transform = np.linalg.inv(rgb_camera_to_base) @ depth_camera_to_base
    kd, kr = depth_intrinsics, rgb_intrinsics
    corners = []
    for du, dv in ((-.5, -.5), (.5, -.5), (-.5, .5), (.5, .5)):
        optical = np.column_stack(((xx+du-kd['cx'])*zz/kd['fx'],
                                   (yy+dv-kd['cy'])*zz/kd['fy'], zz))
        rgb = optical @ transform[:3, :3].T + transform[:3, 3]
        corners.append(np.column_stack((kr['fx']*rgb[:, 0]/rgb[:, 2]+kr['cx'],
                                        kr['fy']*rgb[:, 1]/rgb[:, 2]+kr['cy'])))
    center = np.column_stack(((xx-kd['cx'])*zz/kd['fx'], (yy-kd['cy'])*zz/kd['fy'], zz))
    rgb_z = (center @ transform[:3, :3].T + transform[:3, 3])[:, 2]
    projected = np.stack(corners)
    low = np.floor(projected.min(axis=0)+.5).astype(int)
    high = np.floor(projected.max(axis=0)+.5).astype(int)
    height, width = rgb_shape
    inside = (rgb_z > 0) & (high[:, 0] >= 0) & (high[:, 1] >= 0) & (low[:, 0] < width) & (low[:, 1] < height)
    low, high, rgb_z = low[inside], high[inside], rgb_z[inside]
    low = np.maximum(low, [0, 0])
    high = np.minimum(high, [width-1, height-1])
    aligned = np.full(height*width, np.inf, dtype=np.float32)
    spans = high-low
    if len(spans):
        for dy in range(int(spans[:, 1].max())+1):
            for dx in range(int(spans[:, 0].max())+1):
                selected = (spans[:, 0] >= dx) & (spans[:, 1] >= dy)
                pixels = low[selected]+[dx, dy]
                np.minimum.at(aligned, pixels[:, 1]*width+pixels[:, 0], rgb_z[selected])
    aligned[~np.isfinite(aligned)] = np.nan
    return aligned.reshape(height, width)
