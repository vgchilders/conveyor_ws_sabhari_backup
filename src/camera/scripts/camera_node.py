import random
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, Point
import numpy as np
import cv2
import pyrealsense2 as rs
from arm.srv import stepper_srv
import object_detection_model as odm
import trash_item
import message_filters

BELT_SPEED = 5
CONFIDENCE_THRESHOLD = 10
Y_THRESHOLD = 400

class Camera:
    def __init__(self):
        self.cameraInfo = CameraInfo()
        self.bridge = CvBridge()
        self.trash_items = []
        self.trash_items_shown = []
        self.classifier = odm.ObjectDetectionModel()

        # Services
        self.srv_set_xy_axis = rospy.ServiceProxy('arm/xy_axis_set', stepper_srv)

        # Publishers
        self.target_location_pub = rospy.Publisher("/target_location", Pose, queue_size=1)

        print("Camera Init")
        
    def update_camera_info(self, caminfo):
        self.cameraInfo = caminfo

    def new_image_recieved(self, depth_image, color_img):
        cv_image = self.bridge.imgmsg_to_cv2(color_img, "bgr8")
        print("New image!")

        # find trash objects in image
        new_trash_items = self.classifier.classify(cv_image)
        print("new items: {0}".format(len(new_trash_items)))

        # update previously detected trash item list with belt speed
        for trash in self.trash_items:
            trash.y += BELT_SPEED
            if trash.y > Y_THRESHOLD:
                self.trash_items.remove(trash)
        print("Updated old list")

        # check if any new trash objects match existing trash objects
        for new_trash in new_trash_items:
            for existing_trash in self.trash_items:
                if new_trash.compare_item(existing_trash):
                    existing_trash.update_item(new_trash)
                    break
            else:
                self.trash_items.append(new_trash)
        print("Updated new list")

        # select confident trash item closest to the arm (the one with the highest y pos)
        target_trash = None
        curr_y = 0
        for trash in self.trash_items:
            if trash.conf > CONFIDENCE_THRESHOLD and trash.y > curr_y:
                target_trash = trash
                curr_y = trash.y
        print("Select trash")

        # convert & send target position to arm
        if target_trash:
            target_pixel = [target_trash.x, target_trash.y]
            self.send_target_location(target_pixel, depth_image)

        return cv_image


    def send_target_location(self, pixel, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "passthrough")

        x = int(pixel[0])
        y = int(pixel[1])
        z = cv_image[y, x]

        location = self.depth_to_pos(x, y, z, self.cameraInfo)

        print(location)
        target = Pose()
        target.position.x = location[1]
        target.position.y = location[0]
        target.position.z = location[2]

        self.target_location_pub.publish(target)
        self.move_arm(location[1], location[0])


    def move_arm(self, x, y):
        
        # Arm Y always set to 0 for now
        y = 0
        
        belt_diam = 3.175;
        belt_circumfrence = 2 * np.pi * (belt_diam/2)

        x_stepper= round((x*20)/belt_circumfrence)
        y_stepper= round((y*20)/belt_circumfrence)

        print("Moving X,Y-Axis to ",x_stepper, ",",y_stepper)
        # self.srv_set_xy_axis(x_stepper,y_stepper)


    def depth_to_pos(self, x, y, depth, cameraInfo):
        intrinsics = rs.intrinsics()
        intrinsics.width = cameraInfo.width
        intrinsics.height = cameraInfo.height
        intrinsics.ppx = cameraInfo.K[2]
        intrinsics.ppy = cameraInfo.K[5]
        intrinsics.fx = cameraInfo.K[0]
        intrinsics.fy = cameraInfo.K[4]
        # _intrinsics.model = cameraInfo.distortion_model
        intrinsics.model = rs.distortion.brown_conrady
        intrinsics.coeffs = [i for i in cameraInfo.D]

        result = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], depth)
        # result[0]: right, result[1]: down, result[2]: forward
        return -result[0]+406, -result[1]+319+120, result[2]
        #return -result[0]+398, result[1]+283.423+120, result[2]




if __name__ == '__main__':
	rospy.init_node('camera', anonymous=True)
	Camera()
	rospy.spin()

