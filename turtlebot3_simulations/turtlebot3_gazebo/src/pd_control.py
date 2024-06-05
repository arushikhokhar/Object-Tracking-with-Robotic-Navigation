#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import numpy as np

class Controller:
    def __init__(self) -> None:
        self.move = Twist()
        self.freeze = Twist()

        self.pub_target = rospy.Publisher('/target', String, queue_size=10)
        self.set_model_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # PD Controller parameters for angular control
        self.kp_ang = 0.01  # Proportional gain
        self.kd_ang = 0.002  # Derivative gain

        # PD Controller parameters for linear control
        self.kp_lin = 0.002  # Proportional gain
        self.kd_lin = 0.0002  # Derivative gain

        self.previous_error_ang = 0.0
        self.previous_error_lin = 0.0

        self.previous_time = rospy.Time.now()

        self.dead_zone = 10 
        self.angular_dead_zone = 5  

        self.max_angular_speed = 0.4
        self.max_linear_speed = 0.3

        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.target_detection_subscriber = rospy.Subscriber('/target_detection_info', Float32MultiArray, self.target_detection_callback)

        self.target_detected = False
        self.target_last_seen = rospy.Time.now()
        self.x_center = 0
        self.y_center = 0
        self.image_width = 640
        self.image_height = 480

        self.searching = False
        self.searching_start_time = rospy.Time.now()
        self.searching_duration = rospy.Duration(30)

        self.min_safe_distance = 0.5
        self.obstacle_detected = False

        self.window_size = 5
        self.x_center_history = []
        self.y_center_history = []

        self.state = 'IDLE'

        self.position_change_threshold = 5

    def target_detection_callback(self, msg: Float32MultiArray):
        if len(msg.data) == 4:
            self.target_detected = True
            self.target_last_seen = rospy.Time.now()
            self.x_center_history.append(msg.data[0])
            self.y_center_history.append(msg.data[1])
            if len(self.x_center_history) > self.window_size:
                self.x_center_history.pop(0)
                self.y_center_history.pop(0)
            self.x_center = np.mean(self.x_center_history)
            self.y_center = np.mean(self.y_center_history)
            self.image_width = msg.data[2]
            self.image_height = msg.data[3]
            self.searching = False
        else:
            self.target_detected = False


    def run(self) -> None:
        rate = rospy.Rate(10)  # 10 Hz
        last_position = None
        try:
            while not rospy.is_shutdown():
                if self.target_detected or (rospy.Time.now() - self.target_last_seen < rospy.Duration(1)):
                    rospy.loginfo("target detected: Adjusting robot")

                    self.pub_target.publish("Target Detected!")

                    if self.x_center is not None and self.y_center is not None:
                        image_center_x = self.image_width / 2
                        error_x = self.x_center - image_center_x

                        desired_distance = self.image_height / 2  # Assume target should be centered vertically
                        error_y = desired_distance - self.y_center

                        position_change = False
                        if last_position:
                            position_change = abs(last_position[0] - self.x_center) > self.position_change_threshold or abs(last_position[1] - self.y_center) > self.position_change_threshold
                        last_position = (self.x_center, self.y_center)

                        if abs(error_x) < self.dead_zone:
                            error_x = 0
                        if abs(error_y) < self.dead_zone:
                            error_y = 0

                        current_time = rospy.Time.now()
                        delta_time = (current_time - self.previous_time).to_sec()

                        derivative_error_ang = (error_x - self.previous_error_ang) / delta_time if delta_time > 0 else 0.0
                        angular_z = self.kp_ang * error_x + self.kd_ang * derivative_error_ang
                        derivative_error_lin = (error_y - self.previous_error_lin) / delta_time if delta_time > 0 else 0.0
                        linear_x = self.kp_lin * error_y + self.kd_lin * derivative_error_lin
                        angular_z = max(min(angular_z, self.max_angular_speed), -self.max_angular_speed)
                        linear_x = max(min(linear_x, self.max_linear_speed), -self.max_linear_speed)

                        self.move.angular.z = angular_z
                        self.move.linear.x = linear_x

                        rospy.loginfo(f"Error X: {error_x}, Derivative Error Angular: {derivative_error_ang}, Angular Z: {angular_z}")
                        rospy.loginfo(f"Error Y: {error_y}, Derivative Error Linear: {derivative_error_lin}, Linear X: {linear_x}")

                        self.previous_error_ang = error_x
                        self.previous_error_lin = error_y
                        self.previous_time = current_time

                        self.cmd_vel_publisher.publish(self.move)

                        if not position_change:
                            self.move.angular.z = 0.0
                            self.cmd_vel_publisher.publish(self.move)

                else:
                    if not self.searching:
                        rospy.loginfo("No target detected: Starting search pattern")
                        self.searching = True
                        self.searching_start_time = rospy.Time.now()

                    if rospy.Time.now() - self.searching_start_time < self.searching_duration:
                        # Rotate in place to search for the target
                        self.move.angular.z = self.max_angular_speed
                        self.move.linear.x = 0.0
                        self.cmd_vel_publisher.publish(self.move)
                    else:
                        rospy.loginfo("Search duration exceeded: Freezing the robot")
                        self.cmd_vel_publisher.publish(self.freeze)

                rate.sleep()

        except rospy.exceptions.ROSInterruptException:
            pass

if __name__ == "__main__":
    rospy.init_node("controller", anonymous=True)
    controller = Controller()
    controller.run()
