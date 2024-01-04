import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        self.i = 0

    def camera_callback(self):
        if self.cap.isOpened():
            ret, img = self.cap.read()
            if ret:
                cv2.imshow("Image", img)
                k = cv2.waitKey(1)
                if k == ord('s'):
                    cv2.imwrite('/home/serkan/images/img' + str(self.i) + '.png', img)
                    print("image saved!")
                    self.i += 1
                elif k == ord('q'):
                    self.release_camera()

    def release_camera(self):
        self.cap.release()
        cv2.destroyAllWindows()
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

if __name__ == '__main__':
    main()
