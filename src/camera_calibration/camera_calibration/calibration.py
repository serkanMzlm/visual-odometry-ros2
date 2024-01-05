import glob
import pickle
import cv2 as cv
import numpy as np

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory

startup_path = get_package_share_directory("startup")
calibration_data_save_path = startup_path + "calibration_data/"

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        self.chessboard_size = (7, 7)
        self.frame_size = (720, 720)
        self.criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.objp = np.zeros((self.chessboard_size[0] * self.chessboard_size[1], 3), np.float32)
        self.objp[:,:2] = np.mgrid[0:self.chessboard_size[0],0:self.chessboard_size[1]].T.reshape(-1,2)
        self.size_of_chessboard_squares_mm = 20
        self.objp = self.objp * self.size_of_chessboard_squares_mm
        self.objpoints = [] 
        self.imgpoints = [] 
        self.images = glob.glob(startup_path + '/images/*.png')
        self.calibratetionCallback()
        cv.destroyAllWindows()
        self.safeData()
    def calibratetionCallback(self):
        if len(self.images) == 0:
            self.get_logger().warn('No images found in the images folder.')
            raise SystemExit
        for image in self.images:
            img = cv.imread(image)
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            ret, corners = cv.findChessboardCorners(gray, self.chessboard_size, None)
            if ret == True:
                self.objpoints.append(self.objp)
                corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), self.criteria)
                self.imgpoints.append(corners)

                # Draw and display the corners
                cv.drawChessboardCorners(img, self.chessboard_size, corners2, ret)
                cv.imshow('img', img)
                cv.waitKey(1000)
    def safeData(self):
        ret, cameraMatrix, dist, rvecs, tvecs = cv.calibrateCamera(self.objpoints, self.imgpoints, self.frame_size, None, None)
        # Save the camera calibration result for later use (we won't worry about rvecs / tvecs)
        pickle.dump((cameraMatrix, dist), open(startup_path + "/calibration_data/calibration.pkl", "wb" ))
        pickle.dump(cameraMatrix, open(startup_path + "/calibration_data/cameraMatrix.pkl", "wb" ))
        pickle.dump(dist, open(startup_path + "/calibration_data/dist.pkl", "wb" ))

def main(args=None):
    rclpy.init(args=args)
    calibrate_node = CalibrationNode()

if __name__ == '__main__':
    main()