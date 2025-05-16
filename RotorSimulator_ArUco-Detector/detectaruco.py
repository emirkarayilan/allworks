#!/usr/bin/env python3
import rospy
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class MultiDroneArucoViewer:
    def __init__(self):
        rospy.init_node("multi_drone_aruco_viewer")

        self.bridge = CvBridge()
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()

        self.drones = ["pelican", "hummingbird"]
        self.subscribers = []

        for drone_id in self.drones:
            topic = f"/{drone_id}/vi_sensor/camera_depth/camera/image_raw"
            sub = rospy.Subscriber(topic, Image, self.image_callback, callback_args=drone_id)
            self.subscribers.append(sub)

        rospy.loginfo("Aruco viewer başlatıldı.")
        rospy.spin()
        cv2.destroyAllWindows()

    def image_callback(self, msg, drone_id):
        try:
            # Depth görüntü olarak alıyoruz (mono16 format)
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Normalize edip uint8 yapıyoruz ki OpenCV işlem yapabilsin
            normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
            gray = normalized.astype(np.uint8)

        except Exception as e:
            rospy.logerr(f"[{drone_id}] CV Bridge hatası: {e}")
            return

        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            aruco.drawDetectedMarkers(gray, corners, ids)
            rospy.loginfo(f"[{drone_id}] ArUco tespit edildi: ID'ler = {ids.flatten()}")
        else:
            rospy.loginfo_throttle(5, f"[{drone_id}] ArUco yok.")

        # Görüntüyü göster
        cv2.imshow(f"Aruco - {drone_id}", gray)
        cv2.waitKey(1)

if __name__ == "__main__":
    try:
        MultiDroneArucoViewer()
    except rospy.ROSInterruptException:
        pass

