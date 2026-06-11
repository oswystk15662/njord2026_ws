import math
from typing import List, Tuple, Optional

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path

from asv_trajectory_planner.vessel_state import VesselState
from asv_trajectory_planner.my_planning_algorithm import MyPlanningAlgorithm


class TrajectoryGenerator:
    """
    ROS 2 messageと自前アルゴリズムの橋渡しをするクラス．

    役割：
        1. OdometryからVesselStateを作る
        2. MyPlanningAlgorithmで点列を作る
        3. 点列をnav_msgs/Pathに変換する
    """

    def __init__(
        self,
        frame_id: str = "map",
        point_spacing: float = 0.5,
        avoid_radius: float = 2.0,
        avoid_offset: float = 3.0,
    ):
        self.frame_id = frame_id

        self.planner = MyPlanningAlgorithm(
            point_spacing=point_spacing,
            avoid_radius=avoid_radius,
            avoid_offset=avoid_offset,
        )

    def generate(
        self,
        own_odom: Odometry,
        other_ship_odom: Optional[Odometry],
        goal_pose: PoseStamped,
    ) -> Path:
        """
        OdometryとGoal PoseからNav2用Pathを生成する．
        """

        own_state = self._odom_to_state(own_odom)

        if other_ship_odom is not None:
            other_state = self._odom_to_state(other_ship_odom)
        else:
            other_state = None

        goal = (
            goal_pose.pose.position.x,
            goal_pose.pose.position.y,
        )

        # 自前アルゴリズムclassを呼び出す
        points = self.planner.plan(
            own_state=own_state,
            other_state=other_state,
            goal=goal,
        )

        path = self.points_to_path(points)

        return path

    def _odom_to_state(self, odom: Odometry) -> VesselState:
        """
        nav_msgs/OdometryからVesselStateを作る．
        """

        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y

        q = odom.pose.pose.orientation
        yaw = self._yaw_from_quaternion(q.x, q.y, q.z, q.w)

        u = odom.twist.twist.linear.x
        v = odom.twist.twist.linear.y
        r = odom.twist.twist.angular.z

        return VesselState(
            x=x,
            y=y,
            yaw=yaw,
            u=u,
            v=v,
            r=r,
        )

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

    def points_to_path(
        self,
        points: List[Tuple[float, float]],
    ) -> Path:
        """
        点列をnav_msgs/Pathに変換する．
        """

        path = Path()
        path.header.frame_id = self.frame_id

        for i, (x, y) in enumerate(points):
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            yaw = self._calc_yaw(points, i)

            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)

            path.poses.append(pose)

        return path

    def _calc_yaw(
        self,
        points: List[Tuple[float, float]],
        index: int,
    ) -> float:
        """
        Path各点の向きを計算する．
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