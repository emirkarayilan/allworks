#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import random

class ArucoManager:
    def __init__(self):
        rospy.init_node("aruco_manager", anonymous=True)

        self.received = False
        self.offset_x = random.uniform(4.0, 6.0)
        self.offset_y = random.uniform(2.0, 5.0)

        rospy.Subscriber("/pelican/aruco_position", PoseStamped, self.aruco_callback, callback_args="pelican")
        rospy.Subscriber("/hummingbird/aruco_position", PoseStamped, self.aruco_callback, callback_args="hummingbird")

        self.pub = rospy.Publisher("/aruco/position_world", PoseStamped, queue_size=1)

        rospy.loginfo("ArUco Manager node çalışıyor. İlk tespiti bekliyor...")
    
    def aruco_callback(self, msg, drone_name):
        if self.received:
            return

        new_pose = PoseStamped()
        new_pose.header.stamp = rospy.Time.now()
        new_pose.header.frame_id = "world"

        new_pose.pose.position.x = msg.pose.position.x + self.offset_x
        new_pose.pose.position.y = msg.pose.position.y + self.offset_y
        new_pose.pose.position.z = 0.0001

        new_pose.pose.orientation = msg.pose.orientation

        self.pub.publish(new_pose)
        rospy.loginfo(f"[{drone_name.upper()}] tarafından tespit edilen ArUco konumu Firefly'a gönderildi: x={new_pose.pose.position.x:.2f}, y={new_pose.pose.position.y:.2f}")

        self.received = True

if __name__ == "__main__":
    try:
        ArucoManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

