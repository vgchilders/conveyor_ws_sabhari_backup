import sys
import random as rd
import numpy as np

import trash_item
from camera_node import Camera

from PyQt5.QtWidgets import *
from PyQt5.QtCore    import *
from PyQt5.QtGui     import *

import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy
import message_filters
from std_msgs.msg import Int32
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, Point

import pyrealsense2 as rs
from arm.srv import stepper_srv
import object_detection_model as odm

class DLabel(QLabel):
    def __init__(self, parent=None):
        super(DLabel, self).__init__(parent)
        self.parent = parent
        self.start = QPoint()
        self.end = QPoint()
        self.pixmap = None

    def paintEvent(self, event):
        qp = QPainter(self)

        pn = QPen(Qt.green, 4, Qt.SolidLine)

        qp.setPen(pn)

        if(self.pixmap != None):
            qp.drawPixmap(self.rect(), self.pixmap)

        if(self.parent.state == 1):
            qp.drawRect(QRect(self.start, self.end))

        for trash_item in self.parent.camera.trash_items_shown:
            qp.drawRect((trash_item.x - int(trash_item.width/2)) * self.parent.scale_w, (trash_item.y - int(trash_item.height/2)) * self.parent.scale_h, trash_item.width * self.parent.scale_w, trash_item.height * self.parent.scale_h)
            qp.drawText(int((trash_item.x - (trash_item.width/2)) * self.parent.scale_w), int(trash_item.y - (trash_item.height/2) - 1) * self.parent.scale_h, str(trash_item.trash_type)+" "+str(int(trash_item.conf)))

    def mousePressEvent(self, event):
        if(self.parent.state == 0):
            self.parent.state = 1
            self.start = event.pos()
            self.end = event.pos()
            self.update()

    def mouseMoveEvent(self, event):
        if(self.parent.state == 1):
            print(self.parent.state)
            self.end = event.pos()
            self.update()
        else:
            pass

    def mouseReleaseEvent(self, event):
        if(self.parent.state == 1):
            self.parent.camera.trash_items.append(self.make_trash_item())
            self.parent.state = 0
            self.update()

    def make_trash_item(self):
        # TODO clicking without moving errors
        if(self.start.x() == self.end.x() or self.start.y() == self.end.y()):
            return None
        
        if(self.start.x() > self.end.x()):
            w = self.start.x() - self.end.x()
            x = self.start.x() - w/2
        else:
            w = self.end.x() - self.start.x()
            x = self.end.x() - w/2
        
        if(self.start.y() > self.end.y()):
            h = self.start.y() - self.end.y()
            y = self.start.y() - w/2
        else:
            h = self.end.y() - self.start.y()
            y = self.end.y() - h/2

        # TODO implement setting trash type
        return trash_item.TrashItem(x/self.parent.scale_w, y/self.parent.scale_h, w/self.parent.scale_w, h/self.parent.scale_h, 0, 100)

    def updatePixmap(self, pixmap):
        self.pixmap = pixmap
        self.update()

class TWindow(QMainWindow):
    def __init__(self, parent = None):
        super().__init__(parent)

        print("TWindow init")
        self.setAttribute(Qt.WA_DeleteOnClose)

       # Self objects
        self.camera = Camera()
        self.screen = app.primaryScreen()
        self.state = 0
        self.scale_w = 0
        self.scale_h = 0

        # Create GUI objects
        self.setGeometry(0, 0, self.screen.size().width(), self.screen.size().height())
        self.setWindowTitle("Trash MQP Companion Interface")

        image_label = QLabel(self)
        image_label.setText("Camera Feed")
        image_label.move(int(self.screen.size().width()/2) - int(image_label.size().width()/2), 10)

        self.start_button = QPushButton("Start", self)
        self.start_button.setGeometry(50, 50, 150, 50)
        self.start_button.move(int(self.screen.size().width()) - int((self.start_button.size().width()*.25)/2), 50)
        self.start_button.clicked.connect(self.start)

        self.stop_button = QPushButton("Stop", self)
        self.stop_button.setGeometry(50, 50, 150, 50)
        self.stop_button.move(int(self.screen.size().width()) - int((self.stop_button.size().width()*.25)/2), 150)
        self.stop_button.clicked.connect(self.stop)

        self.dlabel_pixmap = DLabel(self)

         # Subcribers
        self.depth_img_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
        self.color_img_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
        ts = message_filters.TimeSynchronizer([self.depth_img_sub, self.color_img_sub], 1)
        ts.registerCallback(self.new_image_recieved)

        rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, callback=self.update_camera_info, queue_size=1)

    def new_image_recieved(self, depth_image, color_img):
        cv_image = self.camera.new_image_recieved(depth_image, color_img)
        
        if(self.state == 0):
            pixmap = self.cv_image_to_pixmap(cv_image)
            self.camera.trash_items_shown = self.camera.trash_items
            self.dlabel_pixmap.updatePixmap(pixmap)
            self.dlabel_pixmap.resize(pixmap.width(), pixmap.height())
            self.dlabel_pixmap.move(int(self.screen.size().width()/2) - int(pixmap.width()/2), 50)

    def update_camera_info(self, caminfo):
        self.camera.update_camera_info(caminfo)

    def cv_image_to_pixmap(self, cv_image):
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        pixmap = QPixmap.fromImage(QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888))
        self.scale_w = (self.screen.size().width() - (self.screen.size().width() * .25))/pixmap.width()
        self.scale_h = (self.screen.size().height() - (self.screen.size().height() * .25))/pixmap.height()
        scaled_pixmap = pixmap.scaled(pixmap.width() * self.scale_w, pixmap.height() * self.scale_h, Qt.KeepAspectRatio)
        self.scale_w = scaled_pixmap.width()/pixmap.width()
        self.scale_h = scaled_pixmap.height()/pixmap.height()
        return scaled_pixmap

    def start(self):
        pass

    def stop(self):
        pass

if __name__ == '__main__':

    # Init ros
    rospy.init_node('camera', anonymous=True)

    # Init GUI
    app = QApplication(sys.argv)
    win = TWindow()
    win.show()

    # Spin
    app.exec()