import signal
import cv2
import rclpy
from rclpy.node import Node

from os.path import join as Path
from ament_index_python.packages import get_package_share_directory

startup_path = get_package_share_directory("startup")

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.declare_parameter('select_input', 'camera')
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.camera_callback)
        self.select_input = self.get_parameter('select_input').get_parameter_value().string_value
        self.cap = cv2.VideoCapture(0)
        self.i = 0

    def camera_callback(self):
        self.get_logger().debug('Input Mode: %s' %self.select_input)
        if self.cap.isOpened():
            ret, img = self.cap.read()
            if ret:
                cv2.imshow("Image", img)
                k = cv2.waitKey(1)
                if k == ord('s'):
                    cv2.imwrite(startup_path + '/images/img' + str(self.i) + '.png', img)
                    self.get_logger().info('image saved: %s' %startup_path)
                    self.i += 1
                elif k == ord('q'):
                    raise SystemExit
                    self.release_camera()

    def release_camera(self):
        self.get_logger().info('Release camera called.')
        self.cap.release()


def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    try:
        rclpy.spin(camera_node)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    finally:
        camera_node.release_camera()
        cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
