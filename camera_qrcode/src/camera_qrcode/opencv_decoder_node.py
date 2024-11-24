#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import *
from std_msgs.msg import *
import cv2
import numpy as np

# Package to convert between ROS and OpenCV Images
from cv_bridge import CvBridge 

class OpenCvDecoder(Node):

    def __init__(self):
        super().__init__('opencv_decoder_node')
        self.get_logger().info('Hello world from opencv_decoder_node !')

        #Create publisher for qrcode data
        self.publisher_data = self.create_publisher(String, '/aquabot/qrcode_data', 5)

        # Create a subscriber on the topic "qrcode_angle"
        self.publisher_angle = self.create_publisher(Float64, '/aquabot/qrcode_angle', 10)


        # Create a subscriber on the topic "random_image"
        self.subscriber = self.create_subscription(Image, '/aquabot/sensors/cameras/main_camera_sensor/image_raw', self.image_callback, 10)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # Used to decode QR codes
        self.qr_decoder = cv2.QRCodeDetector()

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(msg)

        # Decode image
        dat,bbox,rectifiedImage = self.qr_decoder.detectAndDecode(current_frame)
        msg = String()
        angle = Float64()
        if len(dat) > 0:
            msg.data = dat
            self.publisher_data.publish(msg)
            #self.get_logger().info('Decoded data: ' + data)
            # Calculate the center of the bounding box
            if bbox is not None:
                bbox_center_x = (bbox[0][0][0] + bbox[0][1][0] + bbox[0][2][0] + bbox[0][3][0]) / 4 #bbox[0][0][0] = x du coin en bas à gauche
                bbox_center_y = (bbox[0][0][1] + bbox[0][1][1] + bbox[0][2][1] + bbox[0][3][1]) / 4 #bbox[0][0][1] = y du coin en bas à gauche

                # Get the dimensions of the image
                image_height, image_width = current_frame.shape[:2]

                # Calculate the position relative to the center of the camera
                relative_x = bbox_center_x - (image_width / 2)
                relative_y = bbox_center_y - (image_height / 2)

                #self.get_logger().info(f'QR code position relative to center: x={relative_x}, y={relative_y}')

                # Estimate the translation and rotation
                object_points = np.array([
                    [0, 0, 0], # Bottom-left corner
                    [0.5, 0, 0], # Bottom-right corner
                    [0.5, 0.5, 0], # Top-right corner
                    [0, 0.5, 0] # Top-left corner
                ], dtype=np.float32)

                image_points = np.array([
                    [bbox[0][0][0], bbox[0][0][1]], # Bottom-left corner
                    [bbox[0][1][0], bbox[0][1][1]], # Bottom-right corner
                    [bbox[0][2][0], bbox[0][2][1]], # Top-right corner
                    [bbox[0][3][0], bbox[0][3][1]] # Top-left corner
                ], dtype=np.float32)

                # Camera intrinsic parameters (replace fx, fy, cx, cy with actual values)
                fx = 429  # Example value
                fy = 429  # Example value
                cx = image_width / 2
                cy = image_height / 2

                camera_matrix = np.array([
                    [fx, 0, cx],
                    [0, fy, cy],
                    [0, 0, 1]
                ], dtype=np.float32)

                dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion

                success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)

                if success:
                    #self.get_logger().info(f'Translation vector: {tvec}')
                    #self.get_logger().info(f'Rotation vector: {rvec}')
                    #self.get_logger().info(f'Distance: {np.linalg.norm(tvec[0:2])}')
                    #self.get_logger().info(f'Angle: {np.arctan2(tvec[0], tvec[2])*180/np.pi}') #angle en degré
                    angle.data = float(np.arctan2(tvec[0], tvec[2])) #angle en radian
                    self.publisher_angle.publish(angle)
                    self.get_logger().info('qr code angle: "%s"' % angle.data)

        else:
            msg.data = 'null'
            self.publisher_data.publish(msg)
            angle.data = 10.0 # Valeur impossible pour dire que l'angle n'est pas bon
            self.publisher_angle.publish(angle)
            #self.get_logger().info('No QR code detected')

def main(args=None):
    rclpy.init(args=args)

    opencv_decoder_node = OpenCvDecoder()

    rclpy.spin(opencv_decoder_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    opencv_decoder_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
