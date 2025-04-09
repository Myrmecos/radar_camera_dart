import rclpy
import cv2 as cv
from rclpy.node import Node
from sensor.msg import Image
from cv_bridge import CvBridge


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '//hikrobot_image',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge() #initialize openCV bridge

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge_imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
            cv.imshow("image", cv_image)
            cv.waitKey(10)
        except Exception as e:
            self.get_logger().error(f"error processing image: {str(e)}")
        


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
    cv.destroyAllWindows()


if __name__ == '__main__':
    main()