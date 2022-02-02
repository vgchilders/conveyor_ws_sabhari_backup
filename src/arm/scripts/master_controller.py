
import rospy
from geometry_msgs.msg import Pose
from arm.srv import stepper_srv
from time import sleep
import numpy as np
BELT_SPEED_CM_SEC = 8.369565217
ARM_PICKUP_TIME = 1
RETURN_TIME=2
class Master():
    def __init__(self):
        self.running = False
        rospy.Subscriber("/target_location",
                Pose, callback=self.wait_for_object, queue_size=1)
        self.srv_set_xy_axis = rospy.ServiceProxy('arm/xy_axis_set', stepper_srv)
    def wait_for_object(self,req):
        if self.running == False:
            self.running = True
        
        self.move_arm(req.position.x,req.position.y)
        #print(str(req.position.y/BELT_SPEED_CM_SEC)-ARM_PICKUP_TIME))
        #sleep((req.position.y/BELT_SPEED_CM_SEC)-ARM_PICKUP_TIME)
        sleep(15000)
        #pickup

        self.move_arm(0,0)
        sleep(RETURN_TIME)
        #Let go of object


        self.running= False
    def move_arm(self, x, y):
        
        # Arm Y always set to 0 for now
        y = 0
        
        belt_diam = 3.175
        belt_circumfrence = 2 * np.pi * (belt_diam/2)

        x_stepper= round((x*20)/belt_circumfrence)
        y_stepper= round((y*20)/belt_circumfrence)

        print("Moving X,Y-Axis to ",x_stepper, ",",y_stepper)
        self.srv_set_xy_axis(x_stepper,y_stepper)



if __name__ == '__main__':
	rospy.init_node('master_controller', anonymous=True)
	Master()
	rospy.spin()