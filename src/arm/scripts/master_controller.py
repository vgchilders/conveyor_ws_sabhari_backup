
from tkinter import wantobjects
import rospy
from geometry_msgs.msg import Pose
from arm.srv import stepper_srv
from time import sleep
import numpy as np
from arm_controller import Dynamixel
from std_msgs.msg import Int32

BELT_SPEED_MM_SEC = 83.369565217/2

ARM_PICKUP_TIME = 1
RETURN_TIME = 2
MOVE_ARM_TIME = .5
GRAB_TIME = .5

ARM_UP = 0
ARM_DOWN = 15


class Master():
    def __init__(self):
        self.setup_subscribers()
        self.setup_services()
    
    def setup_services(self):
        self.srv_set_xy_axis = rospy.ServiceProxy('arm/xy_axis_set', stepper_srv)
        # self.srv_set_z_axis = rospy.ServiceProxy('arm/z_axis_set', dynamixel_srv)
        # self.srv_set_gripper = rospy.ServiceProxy('arm/gripper_set', dynamixel_srv)
    
    def setup_subscribers(self):
        rospy.Subscriber("/target_location", Pose, callback=self.wait_for_object, queue_size=1)
        rospy.Subscriber("/home", Int32, callback=self.home_robot, queue_size=1)

    def wait_for_object(self,req):
        wait_time = (req.position.y/BELT_SPEED_MM_SEC)-ARM_PICKUP_TIME
        print(wait_time)

        if (wait_time - ARM_PICKUP_TIME > 0):
            # Move to object
            self.move_arm(req.position.x,req.position.y)
            sleep(wait_time)

            # Arm down
            # self.srv_set_z_axis(ARM_DOWN)
            # sleep(MOVE_ARM_TIME)

            # Grip object
            # self.srv_set_gripper(Dynamixel.gripper_max)
            # sleep(GRAB_TIME)

            # Arm up
            # self.srv_set_z_axis(ARM_UP)
            # sleep(MOVE_ARM_TIME)

            # Move to dropoff
            self.move_arm(0,0)
            sleep(RETURN_TIME)
            
            # Let go of object
            # self.srv_set_gripper(Dynamixel.gripper_min)
            # sleep(GRAB_TIME)



    def move_arm(self, x, y):
        
        # Arm Y always set to 0 for now
        y = 10
        
        belt_diam = 3.175
        belt_circumfrence = 2 * np.pi * (belt_diam/2)

        x_stepper= round((x*20)/belt_circumfrence)
        y_stepper= round((y*20)/belt_circumfrence)

        print("Moving X,Y-Axis to ",x_stepper, ",",y_stepper)
        self.srv_set_xy_axis(x_stepper,y_stepper)
    
    def home_robot(self, req):
        print("Homing X,Y,Z Axes")
        # self.srv_set_z_axis(Dynamixel.z_axis_min)
        # self.srv_set_gripper(Dynamixel.gripper_min)
        self.srv_set_xy_axis(-99,-99)



if __name__ == '__main__':
	rospy.init_node('master_controller', anonymous=True)
	Master()
	rospy.spin()