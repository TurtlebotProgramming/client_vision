import yaml
from ament_index_python.packages import get_package_share_directory
import os

def _load():
    config_path = os.path.join(
        get_package_share_directory('client_vision'),
        'config', 'camera_info.yaml'
    )
    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)['ros__parameters']

    cam   = cfg['camera']
    mat   = cfg['camera_matrix']['data']

    height_m    = cam['height'] 
    tilt_deg    = cam['tilt_deg']
    fx, cx      = mat[0], mat[2]
    fy, cy      = mat[4], mat[5]

    return height_m, tilt_deg, fx, fy, cx, cy

CAMERA_HEIGHT, CAMERA_TILT, FX, FY, CX, CY = _load()
