#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import copy

class FireflyLander:
    def __init__(self):
        rospy.init_node("firefly_lander", anonymous=True)

        self.target_pose = None
        self.landed = False

        self.pub = rospy.Publisher('/firefly/command/pose', PoseStamped, queue_size=1)
        rospy.Subscriber("/aruco/position_world", PoseStamped, self.aruco_callback)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.update_position)

        rospy.loginfo("Firefly iniş node'u başlatıldı. Hedef bekleniyor...")
        
    def aruco_callback(self, msg):
        if self.landed:
            return

        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = "world"
        self.target_pose.pose = copy.deepcopy(msg.pose)
        self.target_pose.pose.position.z = 0.0
        self.target_pose.pose.orientation.w = 1.0

        rospy.loginfo(f"ArUco hedefi alındı: x={self.target_pose.pose.position.x:.2f}, y={self.target_pose.pose.position.y:.2f}")
        rospy.loginfo("Firefly iniş için yönlendiriliyor...")

    def update_position(self, event):
        if self.landed or self.target_pose is None:
            return

        self.target_pose.header.stamp = rospy.Time.now()
        self.pub.publish(self.target_pose)

        rospy.loginfo_once("İniş pozisyonu yayınlanıyor...")

        rospy.Timer(rospy.Duration(5), self.stop_publishing, oneshot=True)

    def stop_publishing(self, event):
        self.landed = True
        rospy.loginfo("Firefly hedefe yönlendirildi. Yayın durduruldu.")

if __name__ == "__main__":
    try:
        FireflyLander()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

