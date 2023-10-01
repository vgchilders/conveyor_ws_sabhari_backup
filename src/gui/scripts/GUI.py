#!/usr/bin/env python3

import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from GUI_Template import Ui_Form
import cv2

import rospy
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Float64, String
from cv_bridge import CvBridge
from arm.srv import dynamixel_srv, stepper_srv
from geometry_msgs.msg import Point
from time import sleep
from arm_controller import Dynamixel, Teensy

belt_diam = 3.175; belt_circumfrence = 2 * np.pi *(belt_diam/2)
x_axis_min = 0; # x_axis_max = belt_circumfrence * (Teensy.x_axis_max/200)
y_axis_min = 0; # y_axis_max = belt_circumfrence * (Teensy.y_axis_max/200)
x_axis_max = 80
y_axis_max = 44

class GUI(Ui_Form):

    def __init__(self, Form):
        self.setupUi(Form)
        self.Img_Frame.mousePressEvent = self.pubMousePos
        Form.setWindowTitle("Conveyor Belt GUI")
        self.setup_signals()
        self.setup_SpinBox_Limits()
        self.setup_subscribers()
        self.setup_publishers()
        self.setup_services()
        self.br = CvBridge()

    def setup_signals(self):
        self.XY_Button.clicked.connect(self.XY_Button_Clicked)
        self.Z_Button.clicked.connect(self.Z_Button_Clicked)

        self.Grip_Open.clicked.connect(self.Grip_Open_Clicked)
        self.Grip_Close.clicked.connect(self.Grip_Close_Clicked)
        
        self.Home_Button.clicked.connect(self.Home_Button_Clicked)
        
        self.Img_RGB_Button.clicked.connect(self.RGB_Button_clicked)
        self.Img_Depth_Button.clicked.connect(self.Depth_Button_clicked)

    def setup_SpinBox_Limits(self):
        self.XY_X_SpinBox.setMinimum(x_axis_min)
        self.XY_X_SpinBox.setMaximum(x_axis_max)

        self.XY_Y_SpinBox.setMinimum(y_axis_min)
        self.XY_Y_SpinBox.setMaximum(y_axis_max)

        self.Z_SpinBox.setMinimum(Dynamixel.z_axis_min)
        self.Z_SpinBox.setMaximum(Dynamixel.z_axis_max)
        
    def setup_subscribers(self):
        self.sub_xy_axis = rospy.Subscriber("/arm/xy_axis", String, self.callback_xy_axis)
        self.sub_z_axis = rospy.Subscriber("/arm/z_axis", Float64, self.callback_z_axis)
        self.sub_grip = rospy.Subscriber("/arm/gripper", Float64, self.callback_gripper)

        self.sub_rgb = rospy.Subscriber("/camera/color/image_raw", Image, self.callback_rgb_image)
        self.sub_depth = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.callback_depth_image)
        #self.sub_points = rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.callback_point_cloud)
        #self.sub_aligned = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.callback_alligned)

    def disconnect_subscribers(self):
        self.sub_xy_axis.unregister()
        self.sub_z_axis.unregister()
        self.sub_grip.unregister()

        self.sub_rgb.unregister()
        self.sub_depth.unregister()
        #self.sub_points.unregister
        #self.sub_aligned.unregister

    def setup_publishers(self):
        self.pub_pixel_loc = rospy.Publisher("/gui_pixel_pos", Point, queue_size=1)

    def setup_services(self):
        self.srv_set_xy_axis = rospy.ServiceProxy('arm/xy_axis_set', stepper_srv)
        self.srv_set_z_axis = rospy.ServiceProxy('arm/z_axis_set', dynamixel_srv)
        self.srv_set_gripper = rospy.ServiceProxy('arm/suction_gripper', dynamixel_srv)

    def callback_xy_axis(self,data):
        self.XY_Val.setText(data.data)
    
    def callback_z_axis(self,data):
        self.Z_Val.setText(str(data.data))
    
    def callback_gripper(self,data):
        self.Grip_Val.setText(str(data.data))

    def callback_rgb_image(self,data):
        if self.Img_RGB_Button.isChecked():
            frame = self.br.imgmsg_to_cv2(data,"bgr8")
            frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
            frame = cv2.resize(frame,(640,480))
            self.Img_Frame.setPixmap(QtGui.QPixmap(QtGui.QImage(frame,frame.shape[1],frame.shape[0],frame.strides[0],QtGui.QImage.Format_RGB888)))

    def pubMousePos(self, event):
        x = event.pos().x()
        y = event.pos().y()

        print("GUI:x: {0}, y: {1}, evebt: {2}".format(x, y, event))
        
        point = Point(x, y, 0)
        self.pub_pixel_loc.publish(point)

    def callback_depth_image(self,data):
        if self.Img_Depth_Button.isChecked():
            frame = self.br.imgmsg_to_cv2(data,"passthrough")
            frame = cv2.convertScaleAbs(frame,alpha=0.05)
            frame = cv2.cvtColor(frame,cv2.COLOR_GRAY2RGB)
            frame = cv2.resize(frame,(640,480))
            print(frame[0])
            self.Img_Frame.setPixmap(QtGui.QPixmap(QtGui.QImage(frame,frame.shape[1],frame.shape[0],frame.strides[0],QtGui.QImage.Format_RGB888)))
    def callback_point_cloud(self,data):
        print(data.height,data.width)
    def callback_alligned(self,data):
        #print(data.height,data.width)
        #print(data.data[0][0])
        return
    def XY_Button_Clicked(self):
        print("GUI:Moving X,Y-Axis to ",self.XY_X_SpinBox.value(), ",",self.XY_Y_SpinBox.value())
        print("GUI:X axis max: ",x_axis_max, ", Y axis max: ",y_axis_max)

        print("GUI:Moving X,Y-Axis to ",self.XY_X_SpinBox.value()*200, ",",(self.XY_X_SpinBox.value()*200)/belt_circumfrence)
        ##TODO ADD CONVERSION
        x_stepper= round((self.XY_X_SpinBox.value()*200)/belt_circumfrence)
        y_stepper= round((self.XY_Y_SpinBox.value()*200)/belt_circumfrence)
        print("GUI:Moving X,Y-Axis to ",x_stepper, ",",y_stepper)
        self.srv_set_xy_axis(x_stepper,y_stepper)
  
    def Z_Button_Clicked(self):
        print("GUI:Moving Z-Axis to ",self.Z_SpinBox.value())
        self.srv_set_z_axis(self.Z_SpinBox.value())
    
    def Grip_Open_Clicked(self):
        print("GUI:Opening Gripper")
        self.srv_set_gripper(0)
    
    def Grip_Close_Clicked(self):
        print("GUI:Closing Gripper")
        self.srv_set_gripper(1)
    
    def Home_Button_Clicked(self):
        print("GUI:Homing X,Y,Z Axes")
        self.srv_set_z_axis(Dynamixel.z_axis_min)
        self.srv_set_gripper(0)
        self.srv_set_xy_axis(-99,-99)
        # sleep(5)
        # reps = 0
        # t1 = 2.5
        # t2 = .2
        # t3 = .3
        # while(reps < 10):
        #     self.srv_set_xy_axis(802,401)
        #     sleep(t1)
        #     self.srv_set_z_axis(Dynamixel.z_axis_max)
        #     sleep(t2)
        #     self.srv_set_gripper(1)
        #     sleep(t3)
        #     self.srv_set_z_axis(Dynamixel.z_axis_min)
        #     sleep(t2)
        #     self.srv_set_xy_axis(10,10)
        #     sleep(t1)
        #     self.srv_set_z_axis(Dynamixel.z_axis_max)
        #     sleep(t2)
        #     self.srv_set_gripper(0)
        #     sleep(t3)
        #     self.srv_set_z_axis(Dynamixel.z_axis_min)
        #     sleep(t1)
        #     self.srv_set_z_axis(Dynamixel.z_axis_max)
        #     sleep(t2)
        #     self.srv_set_gripper(1)
        #     sleep(t3)
        #     self.srv_set_z_axis(Dynamixel.z_axis_min)
        #     sleep(t2)
        #     self.srv_set_xy_axis(802,401)
        #     sleep(t1)
        #     self.srv_set_z_axis(Dynamixel.z_axis_max)
        #     sleep(t2)
        #     self.srv_set_gripper(0)
        #     sleep(t3)
        #     self.srv_set_z_axis(Dynamixel.z_axis_min)
        #     sleep(t2)
        #     self.srv_set_xy_axis(10,10)
        #     sleep(t1)
        # sleep(5)
        # while(True):
        #     print("GUI:Running test movements")
        #     self.srv_set_xy_axis(1584,862)
        #     sleep(5)
        #     self.srv_set_xy_axis(10,10)
        #     sleep(5)


    def RGB_Button_clicked(self):
        print("GUI:RGB Video Stream selected.")
    
    def Depth_Button_clicked(self):
        print("GUI:Depth Video Stream selected.")

    def stop(self):
        print("GUI:Exiting")
        self.disconnect_subscribers()

if __name__ == "__main__":
    rospy.init_node('listener', anonymous=True)
    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = GUI(Form)  
    Form.show()
    app.exec_()
    ui.stop()
