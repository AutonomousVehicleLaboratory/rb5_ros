#!/usr/bin/env python3

"""
Copyright 2023, UC San Diego, Contextual Robotics Institute

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the “Software”), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

"""
import rospy
import cv2
from megapi import *
from std_msgs.msg import String
from sensor_msgs.msg import Joy


class MegaPiController:
    def __init__(self):
        self.bot = MegaPi()
        self.bot.start('/dev/ttyUSB0')

        return

    def move(self, direction, speed):
        # port1: front right (wheel 1)
        # port2: rear left (wheel 0)
        # port9: rear right (wheel 3)
        # port10: front left (wheel 2)

        if direction == "left":
            rospy.loginfo("Moving left")
            # fl wheel
            self.bot.motorRun(10, speed)

            # fr wheel
            self.bot.motorRun(1, speed)

            # rl wheel
            self.bot.motorRun(2, -speed)

            # rr wheel
            self.bot.motorRun(9, -speed)
            return
        elif direction == "right":
            rospy.loginfo("Moving right")
            # fl wheel
            self.bot.motorRun(10, -speed)

            # fr wheel
            self.bot.motorRun(1, -speed)

            # rl wheel
            self.bot.motorRun(2, speed)

            # rr wheel
            self.bot.motorRun(9, speed)
            return
        elif direction == "forward":
            rospy.loginfo("Moving forward")
            # fl wheel
            self.bot.motorRun(10, -speed)

            # fr wheel
            self.bot.motorRun(1, speed)

            # rl wheel
            self.bot.motorRun(2, -speed)

            # rr wheel
            self.bot.motorRun(9, speed)


            return
        elif direction == "reverse":
            rospy.loginfo("Moving in reverse")
            # fl wheel
            self.bot.motorRun(10, speed)

            # fr wheel
            self.bot.motorRun(1, -speed)

            # rl wheel
            self.bot.motorRun(2, speed)

            # rr wheel
            self.bot.motorRun(9, -speed)

            return
        elif direction == "ccwise":
            # TODO: Counter clockwise
            rospy.loginfo("Moving counter clockwise")
            # fl wheel
            self.bot.motorRun(10, speed)

            # fr wheel
            self.bot.motorRun(1, speed)

            # rl wheel
            self.bot.motorRun(2, speed)

            # rr wheel
            self.bot.motorRun(9, speed)
        elif direction == "cwise":
            # TODO: clockwise
            rospy.loginfo("Moving clockwise")
            # fl wheel
            self.bot.motorRun(10, -speed)

            # fr wheel
            self.bot.motorRun(1, -speed)

            # rl wheel
            self.bot.motorRun(2, -speed)

            # rr wheel
            self.bot.motorRun(9, -speed)

        else:
            rospy.loginfo("Stopping")
            # fl wheel
            self.bot.motorRun(10, 0)

            # fr wheel
            self.bot.motorRun(1, 0)

            # rl wheel
            self.bot.motorRun(2, 0)

            # rr wheel
            self.bot.motorRun(9, 0)
            return




    def move_joy(self, joy_cmd):
        if joy_cmd.buttons[5]:
            if joy_cmd.axes[0] > 0.0:
                # left
                self.move("left", 30)
            elif joy_cmd.axes[0] < 0.0:
                # right
                self.move("right", 30)
            elif joy_cmd.axes[1] > 0.0:
                # forward
                self.move("forward", 30)
            elif joy_cmd.axes[1] < 0.0:
                # reverse
                self.move("reverse", 30)
            elif joy_cmd.axes[3] < 0.0:
                # turn clock-wise 
                self.move("cwise", 30)
            elif joy_cmd.axes[3] > 0.0:
                # turn counter clock-wise 
                self.move("ccwise", 30)
            elif abs(joy_cmd.axes[0]) <= 0.1 and abs(joy_cmd.axes[1]) <= 0.0:
                self.move("stop", 0)
        else:
            self.move("stop", 0)



    def move_dir(self, str_cmd):
        
        #if direction == "left":
        #elif direction == "right":
        #elif direction == 
        return    

if __name__ == "__main__":
    ctrl = MegaPiController()
    rospy.init_node('rb5_controller')
    rospy.Subscriber('/rb5/ctrl_cmd_str', String, ctrl.move_dir) 
    rospy.Subscriber('/joy', Joy, ctrl.move_joy, queue_size=1) 
    
    rospy.spin()
