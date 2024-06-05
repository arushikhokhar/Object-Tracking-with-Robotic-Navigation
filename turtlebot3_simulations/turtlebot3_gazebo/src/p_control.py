#!/usr/bin/python3

# ROS
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import numpy as np

class Controller:
    def __init__(self) -> None:
        self.move = Twist()
        self.move.linear.x = 0.3
        self.freeze = Twist()

        # The "p" parameter of p-controller
        self.angular_vel_coef = 1

        self.target_detection_subscriber = rospy.Subscriber('/target_detection_info', Float32MultiArray, self.target_detection_callback)
        
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.target_detected = False
        self.x_center = 0
        self.image_width = 640 

    def target_detection_callback(self, msg):
        if len(msg.data) == 4:
            self.target_detected = True
            self.x_center = msg.data[0]
            self.image_width = msg.data[2]
        else:
            self.target_detected = False

    def run(self) -> None:
        try:
            while not rospy.is_shutdown():
                if self.target_detected:
                    error = (self.x_center - self.image_width / 2) / (self.image_width / 2)
                    angular_z = -self.angular_vel_coef * error

                    self.move.angular.z = angular_z
                    self.cmd_vel_publisher.publish(self.move)
                else:
                    self.cmd_vel_publisher.publish(self.freeze)

                rospy.sleep(0.1)  

        except rospy.ROSInterruptException:
            pass
                

if __name__ == "__main__":
    rospy.init_node("controller", anonymous=True)
    
    controller = Controller()
    controller.run()
