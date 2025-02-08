import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # Subscribe to the /cmd_vel topic.
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10  # QoS profile (10 is a good default)
        )
        
        # Publishers for left and right thruster commands.
        self.publisherl = self.create_publisher(Float64, '/aquabot/thrusters/left/thrust', 10)
        self.publisherr = self.create_publisher(Float64, '/aquabot/thrusters/right/thrust', 10)

        # Set the distance between thrusters (meters).
        self.L = 0.3  # Adjust based on your boat's geometry.

        # Conversion factor:
        # Suppose 1 m/s forward should be mapped to 1000 command units.
        # You might need to calibrate this value based on your system.
        self.K_linear = 100.0  # Gain for linear speed (m/s -> thruster units).
        # Using the same gain for the angular component.
        self.K_angular = self.K_linear/2  # Gain for angular speed (rad/s -> thruster units).

    def cmd_callback(self, msg):
        # Retrieve linear and angular velocities from the cmd_vel message.
        v = msg.linear.x     # Linear speed in m/s.
        w = msg.angular.z    # Angular speed in rad/s.

        # Differential drive equations:
        v_left = -w * self.L / 2.0
        v_right = w * self.L / 2.0

        # Scale the computed speeds to thruster command units.
        cmd_left = self.K_linear * v + self.K_angular * v_left
        cmd_right = self.K_linear * v + self.K_angular * v_right

        # Clamp the commands to the allowed range [-5000, 5000].
        cmd_left = clamp(cmd_left, -5000, 5000)
        cmd_right = clamp(cmd_right, -5000, 5000)

        # Log the computed values.
        self.get_logger().info(
            f'Input: v={v:.2f} m/s, w={w:.2f} rad/s | '
            f'Computed: Left={cmd_left:.0f}, Right={cmd_right:.0f}'
        )

        # Publish the computed thruster commands.
        left_msg = Float64()
        right_msg = Float64()
        left_msg.data = cmd_left
        right_msg.data = cmd_right

        self.publisherl.publish(left_msg)
        self.publisherr.publish(right_msg)

def main(args=None):
    rclpy.init(args=args)
    control_diff = ControlNode()
    rclpy.spin(control_diff)

    # Clean up the node.
    control_diff.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
