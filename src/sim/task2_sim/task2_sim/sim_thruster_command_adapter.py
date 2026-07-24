"""Simulation-only bridge from the real thruster command to the sim dynamics input.

test07089's thruster_driver publishes /thruster_command as
std_msgs/Float32MultiArray in Newtons (FR, FL, RR, RL), while
dutyed_tf_pub_with_disturbance still consumes std_msgs/Int16MultiArray duty
counts. This node converts Newtons back to duty counts so the simulator can
run against the unchanged real-hardware command chain. Never launch this on
the real vessel.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int16MultiArray

from task2_sim.thruster_conversion import (
    four_thruster_forces_to_differential_duties,
)


class SimThrusterCommandAdapter(Node):
    def __init__(self):
        super().__init__("sim_thruster_command_adapter")

        self.declare_parameter("input_topic", "/thruster_command")
        self.declare_parameter("output_topic", "/sim/thruster_duty")
        # Must match thruster_driver's FR, FL, RR, RL geometry.
        self.declare_parameter(
            "real_thruster_angles_rad",
            [0.785398, -0.785398, 2.356194, -2.356194],
        )
        self.declare_parameter(
            "real_thruster_reverse",
            [False, True, False, True],
        )
        self.declare_parameter(
            "real_thruster_positions_xy",
            [0.1803, -0.25, 0.1803, 0.25, -0.1803, -0.25, -0.1803, 0.25],
        )
        # Must match dutyed_tf_pub_with_disturbance config.
        self.declare_parameter("simulator_half_beam_m", 0.35)
        self.declare_parameter("simulator_max_forward_newton", 50.0)
        self.declare_parameter("simulator_max_reverse_newton", 40.0)
        self.declare_parameter("duty_resolution", 1000)

        self._angles = [
            float(v) for v in self.get_parameter("real_thruster_angles_rad").value
        ]
        self._reverse = list(
            self.get_parameter("real_thruster_reverse").value
        )
        flat_positions = list(
            self.get_parameter("real_thruster_positions_xy").value
        )
        self._positions = list(zip(flat_positions[::2], flat_positions[1::2]))
        self._half_beam = float(
            self.get_parameter("simulator_half_beam_m").value
        )
        self._max_forward = float(
            self.get_parameter("simulator_max_forward_newton").value
        )
        self._max_reverse = float(
            self.get_parameter("simulator_max_reverse_newton").value
        )
        self._duty_resolution = int(self.get_parameter("duty_resolution").value)

        self._pub = self.create_publisher(
            Int16MultiArray, self.get_parameter("output_topic").value, 10
        )
        self._sub = self.create_subscription(
            Float32MultiArray,
            self.get_parameter("input_topic").value,
            self._on_command,
            10,
        )

    def _on_command(self, msg: Float32MultiArray):
        duties = Int16MultiArray()
        duties.data = four_thruster_forces_to_differential_duties(
            msg.data,
            self._angles,
            self._reverse,
            self._positions,
            self._half_beam,
            self._max_forward,
            self._max_reverse,
            self._duty_resolution,
        )
        self._pub.publish(duties)


def main(args=None):
    rclpy.init(args=args)
    node = SimThrusterCommandAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
