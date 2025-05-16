#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import threading
import time

class GridScanner:
    def __init__(self, uav_name, xmin, xmax, ymin, ymax, step, altitude):
        self.uav_name = uav_name
        self.pose_pub = rospy.Publisher(f"/{uav_name}/command/pose", PoseStamped, queue_size=10)
        self.status_pub = rospy.Publisher("/swarm/status", String, queue_size=10)
        self.aruco_sub = rospy.Subscriber(f"/{uav_name}/aruco_position", PoseStamped, self.aruco_callback)
        self.swarm_status_sub = rospy.Subscriber("/swarm/status", String, self.status_callback)

        self.normal_rate = 0.5
        self.fast_rate = 2.8
        self.rate = rospy.Rate(self.normal_rate)

        self.altitude = altitude
        self.grid = self.generate_grid(xmin, xmax, ymin, ymax, step)
        self.current_index = 0
        self.swarm_complete = False
        self.all_drones_done = set()
        self.total_drones = rospy.get_param("~total_drones", 2)

        self.aruco_detected = False

    def generate_grid(self, xmin, xmax, ymin, ymax, step):
        x_vals = list(self._frange(xmin, xmax, step))
        y_vals = list(self._frange(ymin, ymax, step))

        grid = []
        for i, y in enumerate(y_vals):
            row = [(x, y) for x in x_vals] if i % 2 == 0 else [(x, y) for x in reversed(x_vals)]
            grid.extend(row)
        return grid

    def _frange(self, start, stop, step):
        while start <= stop:
            yield round(start, 2)
            start += step

    def status_callback(self, msg):
        if msg.data == "aruco_found" and not self.aruco_detected:
            rospy.loginfo(f"{self.uav_name} diğer İHA ArUco buldu, hızlıca bitiriyor!")
            self.aruco_detected = True
            self.rate = rospy.Rate(self.fast_rate)
            return

        drone_name = msg.data
        self.all_drones_done.add(drone_name)
        rospy.loginfo(f"{self.uav_name} durumu aldı: {drone_name} tamamlandı")
        if len(self.all_drones_done) >= self.total_drones:
            self.swarm_complete = True

    def aruco_callback(self, msg):
        if not self.aruco_detected:
            rospy.loginfo(f"{self.uav_name} ArUco bulundu, görev tamamlandı.")
            self.aruco_detected = True
            self.status_pub.publish("aruco_found")  # Tüm swarm'a haber ver
            self.status_pub.publish(self.uav_name)  # Kendi görevi de tamamlandı
            self.rate = rospy.Rate(self.fast_rate)
    def scan(self):
        rospy.loginfo(f"{self.uav_name} taramaya başladı...")
        while not rospy.is_shutdown() and self.current_index < len(self.grid):
            x, y = self.grid[self.current_index]
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = self.altitude
            pose.pose.orientation.w = 1.0
            self.pose_pub.publish(pose)
            rospy.loginfo(f"{self.uav_name} hedef: ({x}, {y}, {self.altitude})")
            self.current_index += 1
            self.rate.sleep()

        if not self.aruco_detected:
            self.status_pub.publish(self.uav_name)
            rospy.loginfo(f"{self.uav_name} taramayı tamamladı.")
        else:
            rospy.loginfo(f"{self.uav_name} Aruco sonrası tarama tamamladı.")

        self.slow_landing()

    def slow_landing(self):
        rospy.loginfo(f"{self.uav_name} yavaş inişe geçiyor...")
        final_x, final_y = self.grid[-1]
        current_z = self.altitude
        landing_target_z = 0.75
        descent_step = 0.2

        while current_z > landing_target_z and not rospy.is_shutdown():
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = final_x
            pose.pose.position.y = final_y
            pose.pose.position.z = current_z
            pose.pose.orientation.w = 1.0
            self.pose_pub.publish(pose)
            rospy.loginfo(f"{self.uav_name} iniş yüksekliği: {current_z:.2f}")
            current_z -= descent_step
            self.rate.sleep()

        final_pose = PoseStamped()
        final_pose.header.stamp = rospy.Time.now()
        final_pose.header.frame_id = "map"
        final_pose.pose.position.x = final_x
        final_pose.pose.position.y = final_y
        final_pose.pose.position.z = landing_target_z
        final_pose.pose.orientation.w = 1.0
        for _ in range(10):
            self.pose_pub.publish(final_pose)
            self.rate.sleep()

        rospy.loginfo(f"{self.uav_name} inişi tamamladı.")
        
        rospy.set_param("/gridscan_done", True)


def main():
    rospy.init_node("swarm_grid_scanner")

    try:
        x_length = float(input("Tarama alanının X uzunluğunu girin (m): "))
        y_length = float(input("Tarama alanının Y uzunluğunu girin (m): "))
    except ValueError:
        print("Geçersiz giriş. Sayı girmeniz gerekiyor.")
        return

    step = 1.0
    altitude = 1.3
    half_x = x_length / 2.0

    hummingbird = GridScanner("hummingbird", 0, half_x, 0, y_length, step, altitude)
    pelican = GridScanner("pelican", half_x, x_length, 0, y_length, step, altitude)

    thread1 = threading.Thread(target=hummingbird.scan)
    thread2 = threading.Thread(target=pelican.scan)

    thread1.start()
    thread2.start()

    thread1.join()
    thread2.join()

if __name__ == "__main__":
    main()

