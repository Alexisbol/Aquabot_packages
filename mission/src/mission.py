import rclpy
from rclpy.node import Node
from sensor_msgs import *
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *

class Mission(Node):
    def __init__(self):
        super().__init__('mission')

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/aquabot/odom',
            self.odom_callback,
            10)
        
        self.turbines_subscription = self.create_subscription(
            PoseArray,
            '/aquabot/ais_sensor/windturbines_positions',
            self.turbinespose_callback,
            10
        )

        self.phase_subscription = self.create_subscription(
            UInt32,
            '/vrx/windturbineinspection/current_phase',
            self.phase_callback,
            10
        )

        self.qr_subscription = self.create_subscription(
            String,
            '/aquabot/qrcode_data',
            self.qrcode_callback,
            10
        )

        self.goal_publishers = self.create_publisher(Point,'/aquabot/goal',10)
        self.camera_publishers = self.create_publisher(Point,'/aquabot/camera_look_at',10)
        self.qr_publishers = self.create_publisher(String,'/vrx/windturbineinspection/windturbine_checkup',10)

        self.timers = self.create_timer(0.1, self.timer_callback)

        self.odom = Odometry()
        self.odom_received = False
        self.liste_turbines = PoseArray()
        self.turbines_received = False
        self.qrcode = String()
        self.qrcode_received = False
        self.phase = UInt32

        self.turbinesI = 0
        self.currentgoal = Point()

        self.status = 'INITIALIZED'


    def odom_callback(self,msg):
        self.odom = msg.data
        self.odom_received = True

    def turbinespose_callback(self,msg):
        self.liste_turbines = msg.data
        self.turbines_received = True

    def phase_callback(self,msg):
        self.phase = msg.data

    def qrcode_callback(self,msg):
        if(msg.data != None):
            self.qrcode = msg.data
            self.qrcode_received = True

    def timer_callback(self):
        if(self.status == 'INITIALIZED'):
            if(self.turbines_received and self.phase == 1):
                self.status = 'SEARCHING'

        if(self.status == 'SEARCHING'):
            self.currentgoal = self.liste_turbines[self.turbinesI]
            if(self.qrcode_received):
                self.turbinesI+=1
                self.qr_publishers.publish(self.qrcode)
            elif():

            






def main(args=None):
    rclpy.init(args=args)

    Mission = Mission()

    rclpy.spin(Mission)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Mission.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
