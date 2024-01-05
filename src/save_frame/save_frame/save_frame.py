import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from os.path import join as Path
from ament_index_python.packages import get_package_share_directory

startup_path = get_package_share_directory("startup")

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.declare_parameter('select_input', 'camera')
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        self.i = 0

    def camera_callback(self):
        my_param = self.get_parameter('select_input').get_parameter_value().string_value
        print(my_param)
        if self.cap.isOpened():
            ret, img = self.cap.read()
            if ret:
                # cv2.imshow("Image", img)
                k = cv2.waitKey(1)
                if k == ord('s'):
                    # save_file = Path(startup_path, "worlds", "calibration.world")
                    cv2.imwrite(startup_path + '/images/img' + str(self.i) + '.png', img)
                    print("image saved!", startup_path)
                    self.i += 1
                elif k == ord('q'):
                    self.release_camera()

    def release_camera(self):
        self.cap.release()
        print("Camera released and windows closed.")
        rclpy.shutdown()

    def run(self):
        while rclpy.ok():
            self.camera_callback()

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    try:
        camera_node.run()
    finally:
        camera_node.release_camera()
        cv2.destroyAllWindows() 

if __name__ == '__main__':
    main()
