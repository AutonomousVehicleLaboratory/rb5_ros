#!/usr/bin/env python
""" MegaPi Controller ROS Wrapper"""
import rospy

from sensor_msgs.msg import Joy
from mpi_control import MegaPiController


class MegaPiControllerNode:
    def __init__(self):
        self.mpi_ctrl = MegaPiController(port='/dev/ttyUSB0', verbose=True)
        self.v_default = 50
        self.v_slide = self.v_default
        self.v_straight = self.v_default
        self.v_rotate = self.v_default
        self.v_max = 100


    def joy_callback(self, joy_cmd):
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

        # if joy_cmd.axes[0] != 0.0:
        #     v_slide =  math.copysign(self.v_slide, joy_cmd.axes[0])
        # if joy_cmd.axes[1] != 0.0:
        #     v_straight = math.copysign(self.v_straight, joy_cmd.axes[1])
        # if joy_cmd.axes[2] != 0.0:
        #     v_rotate = math.copysign(self.v_rotate, joy_cmd.axes[2])
        
        
        # print('v_straight:', v_straight, 'v_rotate:', v_rotate, 'v_slide', v_slide)


        # self.mpi_ctrl.carMixed(v_straight, v_rotate, v_slide)
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
        
        # if joy_cmd.axes[0] > 0.0 and abs(joy_cmd.axes[1]) <= 0.1:
        #     self.mpi_ctrl.carSlide(50)
        # elif joy_cmd.axes[0] < 0.0:
        #     self.mpi_ctrl.carSlide(-50)
        # elif joy_cmd.axes[1] > 0.0:
        #     self.mpi_ctrl.carStraight(50)
        # elif joy_cmd.axes[1] < 0.0:
        #     self.mpi_ctrl.carStraight(-50)
        # elif joy_cmd.buttons[1] > 0.0:
        #     self.mpi_ctrl.carRotate(50)
        # elif joy_cmd.buttons[3] > 0.0:
        #     self.mpi_ctrl.carRotate(-50)
        # elif abs(joy_cmd.axes[0]) <= 0.1 and abs(joy_cmd.axes[1]) <= 0.1:
        #     self.mpi_ctrl.carStop()
        


if __name__ == "__main__":
    mpi_ctrl_node = MegaPiControllerNode()
    rospy.init_node('rb5_megapi_controller')
    rospy.Subscriber('/joy', Joy, mpi_ctrl_node.joy_callback, queue_size=1) 
    
    rospy.spin()