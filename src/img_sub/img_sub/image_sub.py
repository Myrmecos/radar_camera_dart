import rclpy
import cv2 as cv
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from .imageprocessor import GLPosition
from ament_index_python.packages import get_package_share_directory
import yaml
import os

def drawline(image, pos, direction):
    thickness = 10
    color = (255, 0, 0)

    if direction==0: #horizontals
        start_point = (pos, 0)
        end_point = (pos, image.shape[1])
        cv.line(image, start_point, end_point, color, thickness)

    if direction==1:
        start_point (0, pos)
        end_point = (image.shape[0], pos)
        cv.line(image, start_point, end_point, color, thickness)

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/hikrobot_image',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge() #initialize openCV bridge
       
        yaml_path = "/home/astar/ros_ws/src/img_sub/config/cameraparam.yaml"
        self.glp = GLPosition(camera_param_path=yaml_path)
#image = cv.imread("img_dir/0.png")
# glp = GLPosition()
# pos = glp.find_green_light(image)
# #rel_pos = glp.pos_relative_to_center(pos)
# pos = (1024, 1280)
# print("relative position to center: ", pos)
# glp.get_camera_intrinsic_distortion("camparam.yaml")
# print("distortion coefficient: \n", glp.distort)
# print("intrinsic matrix: \n", glp.IM)
# print("angle relative to camera center: ", glp.get_GL_angle_relative(rel_pos))




    def listener_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
            image = cv.imread("/home/astar/ros_ws/src/img_sub/config/14.png")
            self.get_logger().error("checkpoint1")
            # process image
            pos = self.glp.find_green_light(image)
            self.get_logger().error("checkpoint2")
            self.get_logger().error(f"relative green light position to center:{pos}")

            if pos:
                angles = self.glp.get_GL_angle_relative(pos)
                print("angle relative to camera direction: (yaw, pitch)", angles) #rotation along xy plane, rotation at yz plane
                drawline(image, np.round(pos[0]), 0)
                drawline(image, np.round(pos[1]), 1)

            cv.imshow("image", image)
            cv.waitKey(10)
        except Exception as e:
            self.get_logger().error(f"error processing image: {str(e)}")
            import traceback
            traceback.print_stack(e)
        


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