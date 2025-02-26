import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

from math import atan2, pi, cos, sin
from scipy.optimize import minimize
import numpy as np



def euler_from_quaternion(quaternion):
    
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # Subscribe to the /cmd_vel topic.
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback_opt,
            10  # QoS profile (10 is a good default)
        )

        self.subscription = self.create_subscription(
            Odometry,
            'aquabot/odom',
            self.odom_callback,
            10  # QoS profile (10 est une bonne valeur par défaut)
        )        
        # Publishers for left and right thruster commands.
        self.publisherl = self.create_publisher(Float64, '/aquabot/thrusters/left/thrust', 10)
        self.publisherr = self.create_publisher(Float64, '/aquabot/thrusters/right/thrust', 10)
        self.pubtl = self.create_publisher(Float64, '/aquabot/thrusters/left/pos', 10)
        self.pubtr = self.create_publisher(Float64, '/aquabot/thrusters/right/pos', 10)

        # Set the distance between thrusters (meters).
        self.L = 0.3  # Adjust based on your boat's geometry.

        # Conversion factor:
        # Suppose 1 m/s forward should be mapped to 1000 command units.
        # You might need to calibrate this value based on your system.
        self.K_linear = 100.0  # Gain for linear speed (m/s -> thruster units).
        # Using the same gain for the angular component.
        self.K_angular = self.K_linear/1.5  # Gain for angular speed (rad/s -> thruster units).
        self.odom_received = False

    def odom_callback(self, msg):

        if not self.odom_received : 
            self.get_logger().info("first odom received")
        self.posbateau=(msg.pose.pose.position.x,msg.pose.pose.position.y)
        self.vbateau=(msg.twist.twist.linear.x,msg.twist.twist.linear.y)
        
        self.roll, self.pitch, self.yaw = euler_from_quaternion(msg.pose.pose.orientation)
        #self.yaw=2*atan2(msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        self.wbateau=(msg.twist.twist.angular.z)
        self.odom_received = True
        # Afficher les informations de position et de vitesse reçues
        #self.get_logger().info(f"Position -> x: {msg.pose.pose.position.x}, y: {msg.pose.pose.position.y}, z: {msg.pose.pose.position.z}")
        #self.get_logger().info(f"Orientation -> x: {msg.pose.pose.orientation.x}, y: {msg.pose.pose.orientation.y}, z: {msg.pose.pose.orientation.z}, w: {msg.pose.pose.orientation.w}")
        #self.get_logger().info(f"Orientation -> yaw: {self.yaw}, roll: {self.roll}, pitch: {self.pitch}")

        #self.get_logger().info(f"Vitesse linéaire -> x: {msg.twist.twist.linear.x}, y: {msg.twist.twist.linear.y}, z: {msg.twist.twist.linear.z}")
        #self.get_logger().info(f"Vitesse angulaire -> x: {msg.twist.twist.angular.x}, y: {msg.twist.twist.angular.y}, z: {msg.twist.twist.angular.z}")



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


    def cmd_callback_opt(self,msg):

        if not self.odom_received : return 



        def force(u):
    
            x = -3
            xt = -0.278156
            y = 0.6
            fl,fr,tl,tr = u
            fx = fl*cos(tl) + fr*cos(tr)
            fy = fl*sin(tl) + fr*sin(tr)
            m = fl*(x*sin(tl) - y*cos(tl)) + fr*(x*sin(tr) + y*cos(tr))

            return (fx-self.fxd)**2 + (fy-self.fyd)**2 + (m-self.md)**2 + 0.1*tl**2 + 0.1*tr**2
        
            

        self.fxd = 300*(5*msg.linear.x -self.vbateau[0])    
        self.md = 300*(msg.angular.z-3*self.wbateau)
        self.fyd=0

        fl,fr,tl,tr = minimize(force, [0,0,0,0], method='SLSQP',
                                bounds = [(-5000,5000)]*2 + [(-pi/4,pi/4)]*2).x

    
        left_msg = Float64()
        right_msg = Float64()
        tl_msg = Float64()
        tr_msg = Float64()
        tl_msg.data = tl
        tr_msg.data = tr
        left_msg.data = fl
        right_msg.data = fr

        self.get_logger().info(
            f'Input: v={msg.linear.x:.2f} m/s, w={msg.angular.z:.2f} rad/s \n '
            f'Pose : v={self.vbateau[0]:.2f}m/s, w={self.wbateau:.2f} rad/s \n'
            f'Computed: Left={fl:.0f}, Right={fr:.0f}'
        )


        self.publisherl.publish(left_msg)
        self.publisherr.publish(right_msg)
        self.pubtl.publish(tl_msg)
        self.pubtr.publish(tr_msg)

    
    


def main(args=None):
    rclpy.init(args=args)
    control_diff = ControlNode()
    rclpy.spin(control_diff)

    # Clean up the node.
    control_diff.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
