"""Fail closed unless every Task 3 gate TF is currently resolvable."""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, TransformException


class Task3ReadinessAdapter(Node):
    def __init__(self):
        super().__init__("task3_readiness_adapter")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("gate_frames", ["b31_red_gate", "b31_green_gate", "b32_red_gate", "b32_green_gate"])
        self.buffer = Buffer(); self.listener = TransformListener(self.buffer, self)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(Bool, "/mission/readiness/dynamic_gate_tf", qos)
        self.create_timer(0.1, self.publish)

    def publish(self):
        map_frame = str(self.get_parameter("map_frame").value)
        try:
            for frame in self.get_parameter("gate_frames").value:
                self.buffer.lookup_transform(map_frame, str(frame), rclpy.time.Time())
            ready = True
        except TransformException:
            ready = False
        self.publisher.publish(Bool(data=ready))


def main(args=None):
    rclpy.init(args=args); node = Task3ReadinessAdapter()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
