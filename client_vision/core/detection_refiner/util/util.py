import math
from dataclasses import dataclass
from ..params.params_loader import CAMERA_HEIGHT, CAMERA_TILT, FX, FY, CX, CY

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi


@dataclass
class ObjectPos:
    dist: float = 0.0   # 지면 거리 (m)
    theta: float = 0.0  # 로봇 전방 기준 각도 (도, 왼쪽 양수)


def calc_object_distance(pixel_x: float, pixel_y: float) -> ObjectPos:
    u = (pixel_x - CX) / FX
    v = (pixel_y - CY) / FY

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

    return pos
