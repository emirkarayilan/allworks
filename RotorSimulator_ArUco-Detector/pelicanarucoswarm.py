#!/usr/bin/env python3
import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

class PelicanArucoDetector:
    def __init__(self):
        rospy.init_node('pelican_aruco_detector', anonymous=True)

        # ROS publishers and subscribers
        self.image_sub = rospy.Subscriber('/pelican/vi_sensor/camera_depth/camera/image_raw', Image, self.image_callback)
        self.pose_pub = rospy.Publisher('/pelican/aruco_position', PoseStamped, queue_size=10)
        self.detected_pub = rospy.Publisher('/pelican/aruco_detected', Bool, queue_size=10)  # <== yeni

        self.bridge = CvBridge()
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()

       
        self.marker_length = 0.2

       
        self.camera_matrix = np.array([
            [205.46963709898583, 0.0, 320.5],
            [0.0, 205.46963709898583, 240.5],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        self.dist_coeffs = np.zeros((5, 1))
        self.aruco_found = False

    def image_callback(self, msg):
        if self.aruco_found:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr("CV bridge error: %s", e)
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            rospy.loginfo("ArUco Marker Hummingbird tarafından bulundu.")
            self.aruco_found = True

            
            aruco.drawDetectedMarkers(cv_image, corners, ids)

            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_length,
                                                              self.camera_matrix, self.dist_coeffs)
            
            tvec = tvecs[0][0]
            rvec = rvecs[0][0]

            
            aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)

           
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "pelican/vi_sensor/camera_link"
            pose_msg.pose.position.x = tvec[0]
            pose_msg.pose.position.y = tvec[1]
            pose_msg.pose.position.z = tvec[2]

            self.pose_pub.publish(pose_msg)
            self.detected_pub.publish(True)

        cv2.imshow("Pelican Kamera - ArUco Tespiti", cv_image)
        cv2.waitKey(1)

    def __del__(self):
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        detector = PelicanArucoDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
