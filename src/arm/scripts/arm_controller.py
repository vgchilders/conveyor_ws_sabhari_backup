#!/usr/bin/env python3

import os
import rospy
import serial
import time
from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import *
from dynamixel_sdk_examples.msg import *
from tool_dynamixel import DynamixelMotor

from std_msgs.msg import Float64, String, Int32
from arm.srv import dynamixel_srv, stepper_srv

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# Dynamixel parameters

# Protocol version
PROTOCOL_VERSION    = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DynamixelBAUDRATE   = 1000000           # Dynamixel default baudrate : 57600
DynamixelDEVICENAME = '/dev/ttyUSB0'    # Check which port is being used on your controller

dynamixelPortHandler = PortHandler(DynamixelDEVICENAME)
dynamixelPacketHandler = PacketHandler(PROTOCOL_VERSION)

# Teensy parameters
TeensyDEVICENAME    = '/dev/ttyACM0'
TeensyBAUDRATE      = 115200*2

teensyPortHandler = serial.Serial(port=TeensyDEVICENAME, baudrate=TeensyBAUDRATE, timeout=1)

class Dynamixel:
    link_a_start = 1300; link_a_end = 1900
    link_b_start = 2800; link_b_end = 2200
    z_axis_min = 0; z_axis_max = 19

    gripper_start = 2000; gripper_end = 1360
    gripper_min = 0; gripper_max = 10
    
    def __init__(self,Link1_ID,Link2_ID,Gripper_ID):
        self.link_a = DynamixelMotor(Link1_ID, dynamixelPortHandler, dynamixelPacketHandler)
        self.link_b = DynamixelMotor(Link2_ID, dynamixelPortHandler, dynamixelPacketHandler)
        # self.gripper = DynamixelMotor(Gripper_ID, dynamixelPortHandler, dynamixelPacketHandler)

        # Enable torque
        self.link_a.enable_torque()  
        self.link_b.enable_torque()  
        # self.gripper.enable_torque()

        self.setup_services()
        self.setup_publishers()

        print("arm_controller:Dynamixel ready to control")

    def setup_services(self):
        rospy.Service('arm/z_axis_set', dynamixel_srv, self.handle_z_axis)
        rospy.Service('arm/gripper_set', dynamixel_srv, self.handle_gripper)

    def setup_publishers(self):
        self.pub_z_axis = rospy.Publisher('/arm/z_axis', Float64, queue_size=1)
        # self.pub_gripper = rospy.Publisher('/arm/gripper', Float64, queue_size=1)

    def handle_z_axis(self,req):
        input = req.data
        input = max(self.z_axis_min,input); input = min(self.z_axis_max,input)
        link_a_val = self.link_a_start + \
                     (self.link_a_end-self.link_a_start)*input/(self.z_axis_max-self.z_axis_min)
        link_b_val = self.link_b_start + \
                     (self.link_b_end-self.link_b_start)*input/(self.z_axis_max-self.z_axis_min)
        while not self.link_a.set_goal_position(int(link_a_val)): pass
        while not self.link_b.set_goal_position(int(link_b_val)): pass
        self.pub_z_axis.publish(int(input))
        return True

    def handle_gripper(self,req):
        # input = req.data
        # input = max(self.gripper_min,input); input = min(self.gripper_max,input)
        # gripper_val = self.gripper_start + \
        #               (self.gripper_end-self.gripper_start)*input/(self.gripper_max-self.gripper_min)
        # while not self.gripper.set_goal_position(int(gripper_val)): pass
        # self.pub_gripper.publish(int(input))
        return True

class Teensy:
    x_axis_min = 0; x_axis_max = 1610
    y_axis_min = 0; y_axis_max = 900
    
    def __init__(self):
        self.setup_services()
        self.setup_publishers()
        print("arm_controller:Stepper motors ready to control")

    def setup_services(self):
        rospy.Service('arm/xy_axis_set', stepper_srv, self.handle_xy_axis)
        rospy.Service('arm/suction_gripper', dynamixel_srv, self.handle_gripper)

    def setup_publishers(self):
        self.pub_xy_axis = rospy.Publisher('/arm/xy_axis', String, queue_size=1)
    

    def handle_xy_axis(self,req):
        # 0 for xy movement
        data = "<0,-99,-99>"; x = 0; y = 0 # Default values for homing
        if(not(req.data1 == -99 and req.data2 == -99)):
            x = req.data1; x = max(self.x_axis_min,x); x = min(self.x_axis_max,x); x = int(x)
            y = req.data2; y = max(self.y_axis_min,y); y = min(self.y_axis_max,y); y = int(y)
            data = "<0,"+str(-x)+","+str(y)+">" # Inverting x due to the nature of home position
        teensyPortHandler.write(bytes(data, 'utf-8'))
        time.sleep(0.05)
        self.pub_xy_axis.publish(str(x)+","+str(y))
        return True
    
    def handle_gripper(self, req):
        data =  "<1,"+str(req.data)+">"
        teensyPortHandler.write(bytes(data, 'utf-8'))
        time.sleep(0.05)
        return True;


def controller_node():
    rospy.init_node('arm_controller_node')
    Dynamixel(1,2,3)
    Teensy() 
    print("arm_controller:Setup Finished")
    rospy.spin()
    
def main():
    # Open port
    try:
       dynamixelPortHandler.openPort()
       print("arm_controller:Succeeded to open the port for dynamixel")
    except:
        print("arm_controller:Failed to open the port for dynamixel")
        print("arm_controller:Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    try:
        dynamixelPortHandler.setBaudRate(DynamixelBAUDRATE)
        print("arm_controller:Succeeded to change the baudrate for dynamixel")
    except:
        print("arm_controller:Failed to change the baudrate for dynamixel")
        print("arm_controller:Press any key to terminate...")
        getch()
        quit()

    if teensyPortHandler.isOpen():
        print("arm_controller:Succeeded to open the port for teesy")
    else:
        print("arm_controller:Failed to open the port for teensy")
        print("arm_controller:Press any key to terminate...")
        getch()
        quit()

    controller_node()

if __name__ == '__main__':
    main()