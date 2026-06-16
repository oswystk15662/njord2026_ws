import math
from typing import List, Tuple, Optional

from geometry_msgs.msg import PoseStamped, Transform, TwistStamped
from nav_msgs.msg import Odometry, Path

from asv_trajectory_planner.vessel_state import VesselState
from asv_trajectory_planner.mppi_torch import MPPIPlanner


class TrajectoryGenerator:
    """
    ROS 2 messageと自前アルゴリズムの橋渡しをするクラス．

    役割：
        1. 自船Odometryから自船のmap基準姿勢を取得する
        2. ゴールをmap基準からbase_link基準に変換する
        3. 他船TFとTwistStampedからbase_link基準の他船状態を作る
        4. MyPlanningAlgorithmでbase_link基準の点列を作る
        5. base_link基準点列をmap基準Pathに変換する
    """

    def __init__(
        self,
        frame_id: str = "map",
        point_spacing: float = 0.5,
        avoid_radius: float = 2.0,
        avoid_offset: float = 3.0,
        other_twist_is_relative: bool = True,
    ):
        self.frame_id = frame_id
        self.other_twist_is_relative = other_twist_is_relative

        self.planner = MPPIPlanner()

    def generate(
        self,
        own_odom: Odometry,
        other_transform: Optional[Transform],
        other_twist: Optional[TwistStamped],
        waypoint1_pose: PoseStamped,
        waypoint2_pose: PoseStamped,
    ) -> Path:
        """
        自船Odometry，他船TF，他船TwistStamped，2つのウェイポイントからPathを生成する．

        planningはbase_link基準で行い，最後にmap基準のPathへ変換する．
        """

        # --------------------------------------------------
        # 1. 自船のmap基準姿勢を取得
        # --------------------------------------------------
        own_map_x, own_map_y, own_map_yaw = self._own_pose_in_map(own_odom)

        # --------------------------------------------------
        # 2. 自船状態をbase_link基準で作る
        # --------------------------------------------------
        own_state_base = self._own_odom_to_base_state(own_odom)

        # --------------------------------------------------
        # 3. waypoint1をmap基準からbase_link基準へ変換
        # --------------------------------------------------
        waypoint1_base = self._map_to_base_link(
            x_m=waypoint1_pose.pose.position.x,
            y_m=waypoint1_pose.pose.position.y,
            own_x=own_map_x,
            own_y=own_map_y,
            own_yaw=own_map_yaw,
        )

        # --------------------------------------------------
        # 4. waypoint2をmap基準からbase_link基準へ変換
        # --------------------------------------------------
        waypoint2_base = self._map_to_base_link(
            x_m=waypoint2_pose.pose.position.x,
            y_m=waypoint2_pose.pose.position.y,
            own_x=own_map_x,
            own_y=own_map_y,
            own_yaw=own_map_yaw,
        )

        # --------------------------------------------------
        # 5. 他船状態をbase_link基準で作る
        # --------------------------------------------------
        if other_transform is not None and other_twist is not None:
            other_state_base = self._other_tf_twist_to_base_state(
                own_state_base=own_state_base,
                other_transform=other_transform,
                other_twist=other_twist,
            )
        else:
            other_state_base = None

        # --------------------------------------------------
        # 6. base_link基準でplanning
        # --------------------------------------------------
        points_base = self.planner.plan(
            own=own_state_base,
            other=other_state_base,
            waypoint1=waypoint1_base,
            waypoint2=waypoint2_base,
        )

        # --------------------------------------------------
        # 7. base_link基準点列をmap基準Pathへ変換
        # --------------------------------------------------
        path = self.base_points_to_map_path(
            points_base=points_base,
            own_x=own_map_x,
            own_y=own_map_y,
            own_yaw=own_map_yaw,
        )

        return path

    def _own_pose_in_map(
        self,
        own_odom: Odometry,
    ) -> Tuple[float, float, float]:
        """
        自船Odometryからmap基準の位置・方位を取り出す．
        """

        x = own_odom.pose.pose.position.x
        y = own_odom.pose.pose.position.y

        q = own_odom.pose.pose.orientation
        yaw = self._yaw_from_quaternion(q.x, q.y, q.z, q.w)

        return x, y, yaw

    def _own_odom_to_base_state(
        self,
        own_odom: Odometry,
    ) -> VesselState:
        """
        自船状態をbase_link基準で作る．

        planning内部では，
            自船位置 = (0, 0)
            自船方位 = 0
        とする．
        """

        u = own_odom.twist.twist.linear.x
        v = own_odom.twist.twist.linear.y
        r = own_odom.twist.twist.angular.z

        return VesselState(
            x=0.0,
            y=0.0,
            yaw=0.0,
            u=u,
            v=v,
            r=r,
            vx=u,
            vy=v,
        )

    def _other_tf_twist_to_base_state(
    self,
    own_state_base: VesselState,
    other_transform: Transform,
    other_twist: TwistStamped,
    ) -> VesselState:
        """
        base_link基準の他船TFとTwistStampedから，他船状態を作る．

        VesselState内では，
            yaw: deg
            r  : deg/s
        として扱う．
        """

        other_x = other_transform.translation.x
        other_y = other_transform.translation.y

        q = other_transform.rotation

        # quaternion -> yaw[rad] -> yaw[deg]
        other_yaw_rad = self._yaw_from_quaternion(q.x, q.y, q.z, q.w)
        other_yaw_deg = math.degrees(other_yaw_rad)

        tw_x = other_twist.twist.linear.x
        tw_y = other_twist.twist.linear.y

        # ROSのangular.zはrad/sなのでdeg/sへ変換
        tw_r_deg = math.degrees(other_twist.twist.angular.z)

        if self.other_twist_is_relative:
            other_vx = own_state_base.vx + tw_x
            other_vy = own_state_base.vy + tw_y
            other_r_deg = own_state_base.r + tw_r_deg
        else:
            other_vx = tw_x
            other_vy = tw_y
            other_r_deg = tw_r_deg

        # u, v計算では三角関数を使うので，deg -> radに戻して使う
        cy = math.cos(math.radians(other_yaw_deg))
        sy = math.sin(math.radians(other_yaw_deg))

        other_u = cy * other_vx + sy * other_vy
        other_v = -sy * other_vx + cy * other_vy

        return VesselState(
            x=other_x,
            y=other_y,
            yaw=other_yaw_deg,
            u=other_u,
            v=other_v,
            r=other_r_deg,
            vx=other_vx,
            vy=other_vy,
        )

    def base_points_to_map_path(
        self,
        points_base: List[Tuple[float, float]],
        own_x: float,
        own_y: float,
        own_yaw: float,
    ) -> Path:
        """
        base_link基準の点列をmap基準のnav_msgs/Pathに変換する．
        """

        path = Path()
        path.header.frame_id = self.frame_id

        points_map: List[Tuple[float, float]] = []

        for x_b, y_b in points_base:
            x_m, y_m = self._base_link_to_map(
                x_b=x_b,
                y_b=y_b,
                own_x=own_x,
                own_y=own_y,
                own_yaw=own_yaw,
            )
            points_map.append((x_m, y_m))

        for i, (x_m, y_m) in enumerate(points_map):
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id

            pose.pose.position.x = x_m
            pose.pose.position.y = y_m
            pose.pose.position.z = 0.0

            yaw_m = self._calc_yaw(points_map, i)

            pose.pose.orientation.z = math.sin(yaw_m / 2.0)
            pose.pose.orientation.w = math.cos(yaw_m / 2.0)

            path.poses.append(pose)

        return path

    def _map_to_base_link(
        self,
        x_m: float,
        y_m: float,
        own_x: float,
        own_y: float,
        own_yaw: float,
    ) -> Tuple[float, float]:
        """
        map基準の点をbase_link基準へ変換する．
        """

        dx = x_m - own_x
        dy = y_m - own_y

        c = math.cos(own_yaw)
        s = math.sin(own_yaw)

        x_b = c * dx + s * dy
        y_b = -s * dx + c * dy

        return x_b, y_b

    def _base_link_to_map(
        self,
        x_b: float,
        y_b: float,
        own_x: float,
        own_y: float,
        own_yaw: float,
    ) -> Tuple[float, float]:
        """
        base_link基準の点をmap基準へ変換する．
        """

        c = math.cos(own_yaw)
        s = math.sin(own_yaw)

        x_m = own_x + c * x_b - s * y_b
        y_m = own_y + s * x_b + c * y_b

        return x_m, y_m

    def _calc_yaw(
        self,
        points: List[Tuple[float, float]],
        index: int,
    ) -> float:
        """
        Path各点の向きを隣接点から計算する．
        """

        if len(points) <= 1:
            return 0.0

        if index < len(points) - 1:
            x0, y0 = points[index]
            x1, y1 = points[index + 1]
        else:
            x0, y0 = points[index - 1]
            x1, y1 = points[index]

        return math.atan2(y1 - y0, x1 - x0)

    def _yaw_from_quaternion(
        self,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
    ) -> float:
        """
        quaternionからyawを計算する．
        """

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)

        return math.atan2(siny_cosp, cosy_cosp)