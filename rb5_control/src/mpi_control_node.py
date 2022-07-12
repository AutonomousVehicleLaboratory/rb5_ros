#!/usr/bin/env python
""" MegaPi Controller ROS Wrapper"""
import rospy

from sensor_msgs.msg import Joy
from mpi_control import MegaPiController


class MegaPiControllerNode:
    def __init__(self, verbose=True):
        self.mpi_ctrl = MegaPiController(port='/dev/ttyUSB0', verbose=verbose)
        self.v_default = 50
        self.v_slide = self.v_default
        self.v_straight = self.v_default
        self.v_rotate = self.v_default
        self.v_max = 100
        self.verbose = verbose


    def joy_callback(self, joy_cmd):
        if self.verbose:
            print('buttons:', joy_cmd.buttons)
            print('axes:', joy_cmd.axes)

        v_straight = 0
        v_slide = 0
        v_rotate = 0

        v_slide = self.v_max * joy_cmd.axes[0]
        v_straight = self.v_max * joy_cmd.axes[1]
        v_rotate = self.v_max * joy_cmd.axes[2]

        if joy_cmd.axes[3] > 0:
            self.v_max += 10
        elif joy_cmd.axes[3] < 0:
            self.v_max -= 10

        if self.verbose:
            print('v_max:', self.vmax, 'v_straight:', v_straight, 'v_rotate:', v_rotate, 'v_slide', v_slide)

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