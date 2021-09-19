#!/usr/bin/env python3

import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from GUI_Template import Ui_Form
import cv2

import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String
from cv_bridge import CvBridge
from arm.srv import dynamixel_srv, stepper_srv
from arm_controller import Dynamixel, Teensy

class GUI(Ui_Form):
    belt_diam = 1.25
    x_axis_min = 0; x_axis_max = 2 * np.pi *(belt_diam/2) * (Teensy.x_axis_max/200)
    y_axis_min = 0; y_axis_max = 2 * np.pi *(belt_diam/2) * (Teensy.y_axis_max/200)

    def __init__(self, Form):
        self.setupUi(Form)
        Form.setWindowTitle("Conveyor Belt GUI")
        self.setup_signals()
        self.setup_SpinBox_Limits()
        self.setup_subscribers()
        self.setup_services()
        self.br = CvBridge()

    def setup_signals(self):
        self.XY_Button.clicked.connect(self.XY_Button_Clicked)
        self.Z_Button.clicked.connect(self.Z_Button_Clicked)

        self.Grip_Button.clicked.connect(self.Grip_Button_Clicked)
        self.Grip_Open.clicked.connect(self.Grip_Open_Clicked)
        self.Grip_Close.clicked.connect(self.Grip_Close_Clicked)
        
        self.Home_Button.clicked.connect(self.Home_Button_Clicked)
        
        self.Img_RGB_Button.clicked.connect(self.RGB_Button_clicked)
        self.Img_Depth_Button.clicked.connect(self.Depth_Button_clicked)

    def setup_SpinBox_Limits(self):
        self.XY_X_SpinBox.setMinimum(Teensy.x_axis_min)
        self.XY_X_SpinBox.setMaximum(Teensy.x_axis_max)

        self.XY_Y_SpinBox.setMinimum(Teensy.y_axis_min)
        self.XY_Y_SpinBox.setMaximum(Teensy.y_axis_max)


        # self.XY_X_SpinBox.setMinimum(x_axis_min)
        # self.XY_X_SpinBox.setMaximum(x_axis_max)

        # self.XY_Y_SpinBox.setMinimum(y_axis_min)
        # self.XY_Y_SpinBox.setMaximum(y_axis_max)

        self.Z_SpinBox.setMinimum(Dynamixel.z_axis_min)
        self.Z_SpinBox.setMaximum(Dynamixel.z_axis_max)
        
        self.Grip_SpinBox.setMinimum(Dynamixel.gripper_min)
        self.Grip_SpinBox.setMaximum(Dynamixel.gripper_max)

    def setup_subscribers(self):
        self.sub_xy_axis = rospy.Subscriber("/arm/xy_axis", String, self.callback_xy_axis)
        self.sub_z_axis = rospy.Subscriber("/arm/z_axis", Float64, self.callback_z_axis)
        self.sub_grip = rospy.Subscriber("/arm/gripper", Float64, self.callback_gripper)

        self.sub_rgb = rospy.Subscriber("/camera/color/image_raw", Image, self.callback_rgb_image)
        self.sub_depth = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.callback_depth_image)

    def disconnect_subscribers(self):
        self.sub_xy_axis.unregister()
        self.sub_z_axis.unregister()
        self.sub_grip.unregister()

        self.sub_rgb.unregister()
        self.sub_depth.unregister()

    def setup_services(self):
        self.srv_set_xy_axis = rospy.ServiceProxy('arm/xy_axis_set', stepper_srv)
        self.srv_set_z_axis = rospy.ServiceProxy('arm/z_axis_set', dynamixel_srv)
        self.srv_set_gripper = rospy.ServiceProxy('arm/gripper_set', dynamixel_srv)

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

    def callback_depth_image(self,data):
        if self.Img_Depth_Button.isChecked():
            frame = self.br.imgmsg_to_cv2(data,"passthrough")
            frame = cv2.convertScaleAbs(frame,alpha=0.05)
            frame = cv2.cvtColor(frame,cv2.COLOR_GRAY2RGB)
            frame = cv2.resize(frame,(640,480))
            self.Img_Frame.setPixmap(QtGui.QPixmap(QtGui.QImage(frame,frame.shape[1],frame.shape[0],frame.strides[0],QtGui.QImage.Format_RGB888)))

    def XY_Button_Clicked(self):
        print("Moving X,Y-Axis to ",self.XY_X_SpinBox.value(), ",",self.XY_Y_SpinBox.value())
        ##TODO ADD CONVERSION
        self.srv_set_xy_axis(self.XY_X_SpinBox.value(),self.XY_Y_SpinBox.value())
  
    def Z_Button_Clicked(self):
        print("Moving Z-Axis to ",self.Z_SpinBox.value())
        self.srv_set_z_axis(self.Z_SpinBox.value())
    
    def Grip_Button_Clicked(self):
        print("Moving Gripper to ",self.Grip_SpinBox.value())
        self.srv_set_gripper(self.Grip_SpinBox.value())
    
    def Grip_Open_Clicked(self):
        print("Opening Gripper")
        self.srv_set_gripper(Dynamixel.gripper_min)
    
    def Grip_Close_Clicked(self):
        print("Closing Gripper")
        self.srv_set_gripper(Dynamixel.gripper_max)
    
    def Home_Button_Clicked(self):
        print("Homing X,Y,Z Axes")
        self.srv_set_z_axis(Dynamixel.z_axis_min)
        self.srv_set_gripper(Dynamixel.gripper_min)
        self.srv_set_xy_axis(-99,-99)

    def RGB_Button_clicked(self):
        print("RGB Video Stream selected.")
    
    def Depth_Button_clicked(self):
        print("Depth Video Stream selected.")

    def stop(self):
        print("Exiting")
        self.disconnect_subscribers()

if __name__ == "__main__":
    rospy.init_node('listener', anonymous=True)
    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = GUI(Form)  
    Form.show()
    app.exec_()
    ui.stop()
