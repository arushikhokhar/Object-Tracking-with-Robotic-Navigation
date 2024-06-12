#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String

SAFE_DISTANCE = 0.5  
FOLLOW_DISTANCE = 1.5 
SEARCH_ROTATION_SPEED = 0.8 
OBSTACLE_AVOIDANCE_SPEED = 0.2  
OBSTACLE_AVOIDANCE_TURN_SPEED = 0.5  
SEARCH_DURATION = 5  
BACKUP_SPEED = -0.2  
BACKUP_DURATION = 3  
TURN_DURATION = 3  
AVOIDANCE_TIMEOUT = 10  

class TargetFollower:
    def __init__(self):
        rospy.init_node('target_follower', anonymous=True)
        
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.status_pub = rospy.Publisher('/robot_status', String, queue_size=10)
        
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/target_detection_info', Float32MultiArray, self.centroid_callback)
        
        self.closest_distance = float('inf')
        self.closest_angle = 0
        self.centroid_x = None
        self.centroid_y = None
        self.image_width = None
        self.image_height = None
        self.twist = Twist()
        self.target_detected = False
        self.lidar_ranges = []
        self.last_detection_time = rospy.Time.now()
        self.state = "searching" 
        self.state_start_time = rospy.Time.now()
        self.recovery_start_time = rospy.Time.now()

    def scan_callback(self, data):
        self.closest_distance = min(data.ranges)
        self.closest_angle = data.ranges.index(self.closest_distance)
        self.closest_angle = (self.closest_angle * data.angle_increment * 180 / np.pi) - 180

        self.lidar_ranges = data.ranges

        self.state_machine()
    
    def centroid_callback(self, data):
        if len(data.data) == 4:
            self.centroid_x = data.data[0]
            self.centroid_y = data.data[1]
            self.image_width = data.data[2]
            self.image_height = data.data[3]
            self.target_detected = True
            self.last_detection_time = rospy.Time.now()
            if self.state != "avoiding_obstacle" and self.state != "recovery":
                self.state = "following"
                self.state_start_time = rospy.Time.now()
        else:
            self.centroid_x = None
            self.centroid_y = None
            self.image_width = None
            self.image_height = None
            self.target_detected = False

    def state_machine(self):
        if self.closest_distance < SAFE_DISTANCE:
            self.state = "avoiding_obstacle"
            self.state_start_time = rospy.Time.now()
        
        if self.state == "following":
            self.follow_target()
        elif self.state == "avoiding_obstacle":
            self.avoid_obstacle()
        elif self.state == "recovery":
            self.recover()
        elif self.state == "searching":
            self.search()

        self.cmd_vel_pub.publish(self.twist)

    def follow_target(self):
        self.twist = Twist()

        if self.target_detected:
            self.status_pub.publish("Following target")

            image_center_x = self.image_width / 2  
            error_x = self.centroid_x - image_center_x
            self.twist.angular.z = -0.002 * error_x  

            self.twist.linear.x = min(0.5, 0.5 * (self.closest_distance - SAFE_DISTANCE))
        else:
            self.state = "searching"
            self.state_start_time = rospy.Time.now()

    def avoid_obstacle(self):
        self.status_pub.publish("Avoiding obstacle")

        self.twist.linear.x = BACKUP_SPEED
        if self.closest_angle > 0:
            self.twist.angular.z = -OBSTACLE_AVOIDANCE_TURN_SPEED 
        else:
            self.twist.angular.z = OBSTACLE_AVOIDANCE_TURN_SPEED  

        if rospy.Time.now() - self.state_start_time > rospy.Duration(AVOIDANCE_TIMEOUT):
            self.state = "recovery"
            self.state_start_time = rospy.Time.now()

    def recover(self):
        self.status_pub.publish("Recovery mode")

        free_angle = self.find_free_direction()

        if rospy.Time.now() - self.state_start_time < rospy.Duration(BACKUP_DURATION):
            self.twist.linear.x = BACKUP_SPEED
            self.twist.angular.z = free_angle * OBSTACLE_AVOIDANCE_TURN_SPEED / 180  
        else:
            self.state = "searching"
            self.state_start_time = rospy.Time.now()

    def search(self):
        self.status_pub.publish("Searching for target")

        self.twist.linear.x = 0.0
        self.twist.angular.z = SEARCH_ROTATION_SPEED

        if self.closest_distance < SAFE_DISTANCE:
            self.state = "avoiding_obstacle"
            self.state_start_time = rospy.Time.now()

    def find_free_direction(self):
        max_distance = max(self.lidar_ranges)
        max_index = self.lidar_ranges.index(max_distance)
        free_angle = (max_index * len(self.lidar_ranges) * 180 / np.pi) - 180
        return free_angle

if __name__ == '__main__':
    try:
        follower = TargetFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
