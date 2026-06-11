from dataclasses import dataclass


@dataclass
class VesselState:
    """
    経路生成アルゴリズムに渡す船舶状態．

    x, y  : 位置 [m]
    yaw   : 方位 [rad]
    u     : 前後方向速度 [m/s]
    v     : 横方向速度 [m/s]
    r     : 回頭角速度 [rad/s]
    """

    x: float
    y: float
    yaw: float
    u: float
    v: float
    r: float