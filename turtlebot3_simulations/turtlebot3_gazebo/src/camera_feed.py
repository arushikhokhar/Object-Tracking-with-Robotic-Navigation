#!/usr/bin/env python3

import copy
import cv2
import numpy as np
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String

class ImageProcessor:
    def __init__(self):
        self.image_msg = Image()

        self.camera_subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, self.camera_listener)
        self.model = YOLO("yolov8n.pt")
        self.results = None

        self.cv2_frame_size = (400, 320)
        cv2.namedWindow("robot_view", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("robot_view", *self.cv2_frame_size)

        self.target_detection_publisher = rospy.Publisher('/target_detection_info', Float32MultiArray, queue_size=10)
        self.robot_action_subscriber = rospy.Subscriber('/robot_action', String, self.robot_action_listener)
        self.robot_state_subscriber = rospy.Subscriber('/robot_state', String, self.robot_state_listener)

        self.robot_action = ""
        self.robot_state = "IDLE"
        self.update_view()

    def camera_listener(self, msg):
        self.image_msg = copy.deepcopy(msg)

    def robot_action_listener(self, msg):
        self.robot_action = msg.data

    def robot_state_listener(self, msg):
        self.robot_state = msg.data

    def get_target_detection_info(self):
        if self.results is not None and len(self.results) > 0:
            for result in self.results:
                for box in result.boxes:
                    cls = int(box.data.cpu().numpy()[0][5])
                    if self.model.names[cls] == 'person':                                                              #can be changed for other targets as well
                        x_center = (box.data.cpu().numpy()[0][0] + box.data.cpu().numpy()[0][2]) / 2
                        y_center = (box.data.cpu().numpy()[0][1] + box.data.cpu().numpy()[0][3]) / 2
                        return True, x_center, y_center
        return False, None, None

    def update_view(self):
        try:
            while not rospy.is_shutdown():
                if len(self.image_msg.data) == 0:
                    continue

                height = self.image_msg.height
                width = self.image_msg.width
                channels = 3

                self.image_np = np.frombuffer(self.image_msg.data, dtype=np.uint8)
                self.image_np = self.image_np.reshape((height, width, channels))

                frame = copy.deepcopy(self.image_np)

                self.results = self.model(frame)

                target_detected, x_center, y_center = self.get_target_detection_info()
                detection_info = Float32MultiArray()
                if target_detected:
                    detection_info.data = [x_center, y_center, width, height]
                else:
                    detection_info.data = []

                self.target_detection_publisher.publish(detection_info)

                annotator = Annotator(frame)
                for result in self.results:
                    for box in result.boxes:
                        x1, y1, x2, y2, conf, cls = box.data.cpu().numpy()[0]
                        label = f"{self.model.names[int(cls)]} {conf:.2f}"
                        annotator.box_label([x1, y1, x2, y2], label, color=(0, 255, 0))

                frame = annotator.result()

                # Add additional annotations
                if target_detected:
                    cv2.circle(frame, (int(x_center), int(y_center)), 5, (0, 0, 255), -1)
                    cv2.putText(frame, "target detected", (int(x_center) - 50, int(y_center) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, f"State: {self.robot_state}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                cv2.imshow("robot_view", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))

                cv2.waitKey(1)

        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    rospy.init_node('camera_feed', anonymous=True)
    ImageProcessor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()