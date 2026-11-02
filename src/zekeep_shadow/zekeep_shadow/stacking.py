"""Measured two-block placement geometry; the model never chooses coordinates."""
import math

import numpy as np


BLOCK_CLASSES = {f'{color} block' for color in ('red', 'blue', 'green', 'yellow', 'purple', 'gray')}
POSITION_TOLERANCE_M = .005
SIZE_TOLERANCE_M = .003
YAW_SINE_TOLERANCE = .17


def block_geometry(detection, cfg):
    if detection.get('class_name') not in BLOCK_CLASSES or detection.get('position_reference') != 'measured_object_center':
        raise ValueError('stacking requires a measured colored block')
    geometry = detection.get('geometry', {})
    size = np.asarray(geometry.get('size_m'), dtype=float)
    center = np.asarray(detection.get('position'), dtype=float)
    yaw, top = float(geometry.get('yaw_rad', math.nan)), float(geometry.get('top_z_m', math.nan))
    if (size.shape != (3,) or center.shape != (3,)
            or not np.all(np.isfinite(size)) or not np.all(np.isfinite(center))
            or not math.isfinite(yaw) or not math.isfinite(top)
            or np.any(size < .020) or np.any(size > .060) or np.ptp(size) > .008):
        raise ValueError('stacking requires measured 20–60 mm cubes with size variation <= 8 mm')
    table = float(cfg['safety']['table_z_m'])
    if not math.isfinite(table) or abs(top - center[2] - size[2]/2) > .003 or abs(top - table - size[2]) > .003:
        raise ValueError('stacking geometry must describe a single block resting on the table')
    for index, axis in enumerate(('x', 'y', 'z')):
        low, high = cfg['robot']['workspace'][axis]
        if not low <= center[index] <= high:
            raise ValueError('stacking block outside configured workspace')
    pose = np.eye(4)
    c, s = math.cos(yaw), math.sin(yaw)
    pose[:2, :2] = [[c, -s], [s, c]]
    pose[:3, 3] = center
    return size, pose, top


def validate_stack(source, support, cfg):
    size, pose, _ = block_geometry(source, cfg)
    support_size, support_pose, _ = block_geometry(support, cfg)
    if (source.get('id') == support.get('id') or source['class_name'] == support['class_name']
            or np.linalg.norm(pose[:2, 3] - support_pose[:2, 3]) < (min(size[:2]) + min(support_size[:2]))/2):
        raise ValueError('stacking requires two distinct, separated blocks of different colors')
    if np.any(size[:2] > support_size[:2] + .002):
        raise ValueError('source block footprint exceeds the support footprint')


def stack_poses(source, support, grasp_end_pose, cfg):
    validate_stack(source, support, cfg)
    size, source_pose, _ = block_geometry(source, cfg)
    _, target_pose, top = block_geometry(support, cfg)
    settings = cfg.get('stacking', {})
    gap = float(settings.get('release_clearance_m', .010))
    clearance = float(settings.get('preplace_clearance_m', .080))
    padding = float(cfg.get('place', {}).get('object_padding_m', .005))
    if (not all(math.isfinite(x) for x in (gap, clearance, padding)) or padding < 0
            or not padding + .003 <= gap <= .020 or not .030 <= clearance <= .150):
        raise ValueError('invalid stacking clearances or payload padding')
    grasp = np.asarray(grasp_end_pose, dtype=float)
    if grasp.shape != (4, 4) or not np.all(np.isfinite(grasp)):
        raise ValueError('missing measured grasp transform')
    target_pose[2, 3] = top + gap + size[2]/2
    # Keep the measured object-to-tool offset while aligning the two cube footprints.
    release = target_pose @ np.linalg.inv(source_pose) @ grasp
    above = release.copy()
    above[2, 3] += clearance
    for pose in (release, above):
        for index, axis in enumerate(('x', 'y', 'z')):
            low, high = cfg['robot']['workspace'][axis]
            if not low <= pose[index, 3] <= high:
                raise ValueError('stacking waypoint outside configured workspace')
        if pose[2, 3] < cfg['robot'].get('min_tcp_z_m', .005):
            raise ValueError('stacking waypoint below TCP floor')
    return above, release


def support_collision_box(support, cfg, *, added_height=0.0, payload_size_m=None):
    size, pose, _ = block_geometry(support, cfg)
    footprint = size[:2]
    margin = POSITION_TOLERANCE_M
    height = size[2] + added_height + SIZE_TOLERANCE_M
    if payload_size_m is not None:
        payload = np.asarray(payload_size_m, dtype=float)
        if payload.shape != (3,) or not np.all(np.isfinite(payload)) or np.any(payload <= 0):
            raise ValueError('invalid stacked payload size')
        footprint = np.maximum(footprint, payload[:2])
        margin += POSITION_TOLERANCE_M
        height += POSITION_TOLERANCE_M + SIZE_TOLERANCE_M
    # Enclose every footprint yaw plus the position/size change admitted by verification.
    width = float(np.linalg.norm(footprint + SIZE_TOLERANCE_M)) + 2*margin
    extent = np.array([width, width, height])
    center = pose[:3, 3].copy()
    center[2] = float(cfg['safety']['table_z_m']) + height/2
    return center.tolist(), extent.tolist()
