#!/usr/bin/env python
""" MegaPi Controller ROS Wrapper"""
import rospy

from sensor_msgs.msg import Joy
from mpi_control import MegaPiController


class MegaPiControllerNode:
    def __init__(self, verbose=True, debug=False):
        self.mpi_ctrl = MegaPiController(port='/dev/ttyUSB0', verbose=verbose)
        self.v_max_default_straight = 100
        self.v_max_default_slide = 100
        self.v_max_default_rotate = 50
        self.reset_v_max()
        self.verbose = verbose
        self.debug = debug
        self.state = "run"
    

    def reset_v_max(self):
        self.v_max_straight = self.v_max_default_straight
        self.v_max_slide = self.v_max_default_slide
        self.v_max_rotate = self.v_max_default_rotate


    def joy_callback(self, joy_cmd):
        if self.debug:
            print('buttons:', joy_cmd.buttons)
            print('axes:', [round(axe,2) for axe in joy_cmd.axes])
        if joy_cmd.buttons[4] == 1:
            if self.state == "run":
                self.state = "stop"
            elif self.state == "stop":
                self.state = "run"
        if self.state == "stop":
            print('Vehicle in the stop state.')
            return
        
        if joy_cmd.buttons[5] == 1:
            print('Reset max speed')
            self.reset_v_max()

        v_straight = 0
        v_slide = 0
        v_rotate = 0

        v_slide = self.v_max_slide * joy_cmd.axes[0]
        v_straight = self.v_max_straight * joy_cmd.axes[1]
        v_rotate = self.v_max_rotate * joy_cmd.axes[2]

        if joy_cmd.axes[4] > 0:
            self.v_max_slide -= 10
        elif joy_cmd.axes[4] < 0:
            self.v_max_slide += 10
        if joy_cmd.axes[5] > 0:
            self.v_max_straight += 10
        elif joy_cmd.axes[5] < 0:
            self.v_max_straight -= 10
        if joy_cmd.buttons[1] == 1:
            self.v_max_rotate += 10
        elif joy_cmd.buttons[3] == 1:
            self.v_max_rotate -= 10

        if self.verbose:
            print('state: ' + self.state +
                  ' v_straight: ' + repr(int(round(v_straight, 2))) + '/' + repr(self.v_max_straight) +
                  ' v_slide: '+ repr(int(round(v_slide, 2))) + '/' + repr(self.v_max_slide) +
                  ' v_rotate: ' + repr(int(round(v_rotate, 2))) + '/' + repr(self.v_max_rotate))

        if abs(joy_cmd.axes[2]) <= 0.1:
            if abs(joy_cmd.axes[0]) <= 0.1 and abs(joy_cmd.axes[1]) <= 0.1:
                self.mpi_ctrl.carStop()
            elif abs(joy_cmd.axes[0]) <= 0.1:
                self.mpi_ctrl.carStraight(v_straight)
            elif abs(joy_cmd.axes[1]) <= 0.1:
                self.mpi_ctrl.carSlide(v_slide)
            else:
                self.mpi_ctrl.carMixed(v_straight, 0, v_slide)
        else:
            if abs(joy_cmd.axes[0]) <= 0.1 and abs(joy_cmd.axes[1]) <= 0.1:
                self.mpi_ctrl.carRotate(v_rotate)
            else:
                self.mpi_ctrl.carMixed(v_straight, v_rotate, v_slide)
        

if __name__ == "__main__":
    mpi_ctrl_node = MegaPiControllerNode()
    rospy.init_node('megapi_controller')
    rospy.Subscriber('/joy', Joy, mpi_ctrl_node.joy_callback, queue_size=1) 
    
    rospy.spin()
