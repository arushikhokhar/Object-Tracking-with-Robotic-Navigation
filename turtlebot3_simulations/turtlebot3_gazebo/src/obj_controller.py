#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState, ModelStates
from pynput import keyboard
import math

class ModelController:
    def __init__(self):
        self.current_state = None
        self.model_name = 'person_standing'                                     #can be changed to any model name
        
        rospy.init_node('model_service_node', anonymous=True)
        
        self.pub_model = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)        
        self.state_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.state_callback)        
        self.rate = rospy.Rate(10)        
        rospy.wait_for_service('/gazebo/set_model_state')        
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)        
        self.key = 'q'

        self.print_instructions()

    def print_instructions(self):
        print("Control Instructions:")
        print("W: Move forward")
        print("S: Move backward")
        print("A: Move left")
        print("D: Move right")
        print("Q: Rotate counter-clockwise")
        print("E: Rotate clockwise")
        print("Press 'Esc' to exit.")

    def state_callback(self, msg):
        try:
            index = msg.name.index(self.model_name)
            self.current_state = msg.pose[index]
        except ValueError:
            rospy.logerr(f"Model {self.model_name} not found in /gazebo/model_states")

    def run(self):
        while not rospy.is_shutdown():
            with keyboard.Events() as events:
                event = events.get(1e6)
                if isinstance(event, keyboard.Events.Press):
                    key = event.key
                    if key == keyboard.KeyCode.from_char('s'):
                        self.key = 's'
                    elif key == keyboard.KeyCode.from_char('w'):
                        self.key = 'w'
                    elif key == keyboard.KeyCode.from_char('a'):
                        self.key = 'a'
                    elif key == keyboard.KeyCode.from_char('d'):
                        self.key = 'd'
                    elif key == keyboard.KeyCode.from_char('q'):
                        self.key = 'q'
                    elif key == keyboard.KeyCode.from_char('e'):
                        self.key = 'e'
                    elif key == keyboard.Key.esc:
                        return  
            if self.current_state is None:
                rospy.logwarn("Waiting for the initial state of the model...")
                self.rate.sleep()
                continue

            state_msg = ModelState()
            state_msg.model_name = self.model_name
            state_msg.pose = self.current_state

            if self.key == 'w':
                state_msg.pose.position.x += 0.07
            elif self.key == 's':
                state_msg.pose.position.x -= 0.07
            elif self.key == 'd':
                state_msg.pose.position.y -= 0.07
            elif self.key == 'a':
                state_msg.pose.position.y += 0.07

            if self.key == 'q':
                rotation_angle = math.pi / 12  
            elif self.key == 'e':
                rotation_angle = -math.pi / 12 
            else:
                rotation_angle = 0

            current_yaw = 2 * math.atan2(state_msg.pose.orientation.z, state_msg.pose.orientation.w)
            current_yaw += rotation_angle
            state_msg.pose.orientation.z = math.sin(current_yaw / 2)
            state_msg.pose.orientation.w = math.cos(current_yaw / 2)

            try:
                resp = self.set_state(state_msg)
                if resp.success:
                    self.current_state = state_msg.pose
                else:
                    rospy.logerr(f"Failed to set model state: {resp.status_message}")
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")

            self.rate.sleep()

if __name__ == '__main__':
    try:
        model_controller = ModelController()
        model_controller.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"An error occurred: {e}")
