import numpy as np
import cv2
import cv2.aruco as aruco
import sys
import rospy
import glob
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# convert ros_img messages into cv2_img
class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2",Image)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)

    def callback(self,data):
      try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      except CvBridgeError as e:
          print(e)

     if np.all(ids != None):
         rvec, tvec,_ = aruco.estimatePoseSingleMarkers(corners[0], 0.05, mtx, dist)
         topleftX = corners[0][0][0][0]
         topleftY = corners[0][0][0][1]

         toprightX = corners[0][0][1][0]
         toprightY = corners[0][0][1][1]

         bottomleftX = corners[0][0][2][0]
         bottomlextY = corners[0][0][2][1]

         bottomrightX = corners[0][0][3][0]
         bottomrightY = corners[0][0][3][1]

         # expected information
         distance = tvec[0][0][2]
         midpointX = (topleftX  + bottomrightX)/2
         midpointY = (topleftY + bottomrightY)/2

         aruco.drawAxis(cv_image, mtx, dist, rvec[0], tvec[0], 0.1)
         aruco.drawDetectedMarkers(cv_image, corners)

         # draw ID
         font = cv2.FONT_HERSHEY_SIMPLEX
         cv2.putText(cv_image, "Id: " + str(ids), (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)

      try:
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      except CvBridgeError as e:
          print(e)

def main(args):
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6*7,3), np.float32)
    objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    
    images = glob.glob('calib_images/*.jpg')

    for fname in images:
        img = cv2.imread(fname)
          gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)

          # Find the chess board corners
          ret, corners = cv2.findChessboardCorners(gray, (7,6),None)
          print(corners)
          print(objpoints)

          # If found, add object points, image points (after refining them)
          if ret == True:
              objpoints.append(objp)
              corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
              imgpoints.append(corners2)

              # Draw and display the corners
              img = cv2.drawChessboardCorners(img, (7,6), corners2,ret)

      ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

      aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
      parameters = aruco.DetectorParameters_create()
      corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
    

