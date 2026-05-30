import math
import numpy as np
import cv2
from dataclasses import dataclass
from ..params.params_loader import CAMERA_HEIGHT, CAMERA_TILT, FX, FY, CX, CY, CAMERA_K, CAMERA_D, CAMERA_R, CAMERA_P

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi


@dataclass
class ObjectPos:
    dist: float = 0.0   # 지면 거리 (m)
    theta: float = 0.0  # 로봇 전방 기준 각도 (도, 왼쪽 양수)
    x: float = 0.0      # 전방 거리 (m)
    y: float = 0.0      # 측면 편차 (m, 왼쪽 양수)


def calc_object_distance(pixel_x: float, pixel_y: float) -> ObjectPos:
    # 왜곡 보정 + 정류: 픽셀 → rectified 픽셀 좌표 → P로 정규화
    pts = np.array([[[pixel_x, pixel_y]]], dtype=np.float64)
    rectified = cv2.undistortPoints(pts, CAMERA_K, CAMERA_D, R=CAMERA_R, P=CAMERA_P)
    px = float(rectified[0, 0, 0])
    py = float(rectified[0, 0, 1])
    u = (px - CAMERA_P[0, 2]) / CAMERA_P[0, 0]
    v = (py - CAMERA_P[1, 2]) / CAMERA_P[1, 1]

    pos = ObjectPos()

    # 카메라 높이 to 전방 지면 거리
    C_P_ = CAMERA_HEIGHT * math.tan(
        (math.pi / 2) + CAMERA_TILT * DEG2RAD - math.atan(v)
    )

    # 3D 거리 분해
    CP_ = math.sqrt(CAMERA_HEIGHT ** 2 + C_P_ ** 2)
    Cp_ = math.sqrt(1 + v * v)
    PP_ = u * CP_ / Cp_   # 측면 편차 (m)

    pos.dist  = math.sqrt(C_P_ ** 2 + PP_ ** 2)
    pos.theta = -math.atan2(PP_, C_P_) * RAD2DEG
    pos.x     = C_P_
    pos.y     = -PP_

    return pos
