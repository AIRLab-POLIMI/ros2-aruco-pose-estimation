
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

import cv2, math, time

class WebcamNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('webcam_node')
        self.publisher_ = self.create_publisher(Image, '/camera_feed', 100)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # Create a VideoCapture object
        # The argument '0' gets the default webcam.
        self.cap = cv2.VideoCapture(0)
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()


    def timer_callback(self):
        # Capture frame-by-frame
        ret, img = self.cap.read()
        img = cv2.resize(img, (360, 240))
        image_message = self.br.cv2_to_imgmsg(img)
        image_message.header.frame_id = 'camera_frame'
        # cv2.imshow("Image", img)
        # cv2.waitKey(1)
        if ret == True:
            self.publisher_.publish(image_message)
            self.get_logger().info('Publishing images')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    webcam_node = WebcamNode()

    rclpy.spin(webcam_node)

    # Destroy the node explicitly
    webcam_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()