import random
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, Point
import numpy as np
import cv2
import pyrealsense2 as rs
from arm.srv import stepper_srv


class Camera:
	
	def __init__(self):
		self.cameraInfo = CameraInfo()
		self.bridge = CvBridge()

		self.raw_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw",
						Image, callback=self.update_raw_image, queue_size=1)
		rospy.Subscriber("/camera/aligned_depth_to_color/camera_info",
						CameraInfo, callback=self.update_camera_info, queue_size=1)
		self.srv_set_xy_axis = rospy.ServiceProxy('arm/xy_axis_set', stepper_srv)
		self.target_location_pub = rospy.Publisher("/target_location", Pose, queue_size=1)
		self.sub_pixel_pos = rospy.Subscriber("/gui_pixel_pos", Point, callback=self.send_target_location, queue_size=1)

		self.raw_image = Image()


	def update_camera_info(self, caminfo):
		self.cameraInfo = caminfo
		# print(cameraInfo)

	def update_raw_image(self, ros_image):
		self.raw_image = ros_image

	def send_target_location(self, pixel):
		depth_image = self.bridge.imgmsg_to_cv2(self.raw_image, "passthrough")

		x = int(pixel.x)
		y = int(pixel.y)
		z = depth_image[y, x]

		location = self.depth_to_pos(x, y, z, self.cameraInfo)

		print(location)
		target = Pose()
		target.position.x = location[1]
		target.position.y = location[0]
		target.position.z = location[2]

		self.target_location_pub.publish(target)
		self.move_arm(location[1], location[0])


	def select_target_location(self, ros_image):
		depth_image = self.bridge.imgmsg_to_cv2(ros_image, "passthrough")

		pixels = self.run_model(ros_image)

		for pixel in pixels:
			z = depth_image[pixel[1], pixel[0]]

			location = self.depth_to_pos(
				pixel[0], pixel[1], z, self.cameraInfo)
			print(location)
			target = Pose()
			target.position.x = location[1]
			target.position.y = location[0]
			target.position.z = location[2]

			self.target_location_pub.publish(target)
			self.move_arm(location[1], location[0])
			self.raw_sub.unregister() # TODO look at this


	def run_model(self, ros_image): #roslaunch realsense2_camera rs_camera.launch filters:=pointcloud
		x = 106#random.randrange(640)
		y = 421	#random.randrange(480)

		return [[x, y]]


	def move_arm(self, x, y):
		y = 0 #TODO change this
		belt_diam = 3.175; belt_circumfrence = 2 * np.pi *(belt_diam/2)
		print(x)
		x_stepper= round((x*20)/belt_circumfrence)
		y_stepper= round((y*20)/belt_circumfrence)
		print("Moving X,Y-Axis to ",x_stepper, ",",y_stepper)
		self.srv_set_xy_axis(x_stepper,y_stepper)


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

