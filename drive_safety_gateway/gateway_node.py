import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy

from drive_safety_gateway.gateway_logic import (
    GatewayLogic,
    GatewayParams,
    VelocityCommand,
)


class DriveSafetyGateway(Node):

    def __init__(self):
        super().__init__("drive_safety_gateway")

        # Parameters:
        self.declare_parameter("slow_distance", 1.0)
        self.declare_parameter("stop_distance", 0.3)
        self.declare_parameter("timeout_sec", 0.5)

        self.declare_parameter("publish_rate", 20.0)

        self.declare_parameter("max_linear_vel", 1.0)
        self.declare_parameter("max_angular_vel", 1.5)

        params = GatewayParams(
            slow_distance=self.get_parameter("slow_distance").value,
            stop_distance=self.get_parameter("stop_distance").value,
            timeout_sec=self.get_parameter("timeout_sec").value,
            max_linear_vel=self.get_parameter("max_linear_vel").value,
            max_angular_vel=self.get_parameter("max_angular_vel").value,
        )

        self.logic = GatewayLogic(params)

        # QoS Profile:
        safety_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Internal State:
        self.latest_cmd = VelocityCommand(linear_x=0.0, angular_z=0.0)

        self.estop = False

        self.obstacle_distance = float("inf")

        self.last_cmd_time = self.get_clock().now()

        # Publishers:

        self.cmd_pub = self.create_publisher(Twist, "/drive/cmd_vel", safety_qos)

        self.state_pub = self.create_publisher(
            String, "drive/gateway_state", safety_qos
        )

        # Subscriber:

        self.cmd_sub = self.create_subscription(
            Twist,
            "/nav/cmd_vel",
            self.cmd_callback,
            safety_qos,
        )

        self.estop_sub = self.create_subscription(
            Bool,
            "/safety/estop",
            self.estop_callback,
            safety_qos,
        )

        self.obstacle_sub = self.create_subscription(
            Float32,
            "safety/min_obstacle_dist",
            self.obstacle_callback,
            safety_qos,
        )

        # Timer (20 Hz)
        publish_rate = self.get_parameter("publish_rate").value

        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        self.get_logger().info("Drive Safety Gateway started")

    # CALLBACKS:

    def cmd_callback(self, msg: Twist):

        self.latest_cmd = VelocityCommand(
            linear_x=msg.linear.x, angular_z=msg.angular.z
        )

        self.last_cmd_time = self.get_clock().now()

    def estop_callback(self, msg: Bool):

        self.estop = msg.data

    def obstacle_callback(self, msg: Float32):

        self.obstacle_distance = msg.data

    # TIMER:

    def timer_callback(self):
        current_time = self.get_clock().now()

        time_since_last_cmd = (current_time - self.last_cmd_time).nanoseconds / 1e9

        final_cmd, state = self.logic.evaluate(
            cmd=self.latest_cmd,
            estop=self.estop,
            obstacle_distance=self.obstacle_distance,
            time_since_last_cmd=time_since_last_cmd,
        )

        # Publish Twist

        twist_msg = Twist()

        twist_msg.linear.x = final_cmd.linear_x
        twist_msg.angular.z = final_cmd.angular_z

        self.cmd_pub.publish(twist_msg)

        # Publish State:

        state_msg = String()

        state_msg.data = state.value

        self.state_pub.publish(state_msg)

        ## FOR DEBUGGING INFO
        # self.get_logger().info(
        #     f"State={state.value}, "
        #     f"Obstacle={self.obstacle_distance:.2f}, "
        #     f"ESTOP={self.estop}, "
        #     f"cmd=({final_cmd.linear_x:.2f}, "
        #     f"{final_cmd.angular_z:.2f})"
        # )


def main(args=None):
    rclpy.init(args=args)
    node = DriveSafetyGateway()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()
