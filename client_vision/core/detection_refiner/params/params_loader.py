import yaml
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os

def _load():
    config_path = os.path.join(
        get_package_share_directory('client_vision'),
        'config', 'camera_info.yaml'
    )
    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)['ros__parameters']

    cam  = cfg['camera']
    mat  = cfg['camera_matrix']['data']
    dist = cfg['distortion_coefficients']['data']
    rect = cfg['rectification_matrix']['data']
    proj = cfg['projection_matrix']['data']

    height_m = cam['height']
    tilt_deg = cam['tilt_deg']
    fx, cx   = mat[0], mat[2]
    fy, cy   = mat[4], mat[5]

    K = np.array([[fx, 0.0, cx],
                  [0.0, fy, cy],
                  [0.0, 0.0, 1.0]], dtype=np.float64)
    D = np.array(dist, dtype=np.float64)
    R = np.array(rect, dtype=np.float64).reshape(3, 3)
    P = np.array(proj, dtype=np.float64).reshape(3, 4)

    return height_m, tilt_deg, fx, fy, cx, cy, K, D, R, P

CAMERA_HEIGHT, CAMERA_TILT, FX, FY, CX, CY, CAMERA_K, CAMERA_D, CAMERA_R, CAMERA_P = _load()
