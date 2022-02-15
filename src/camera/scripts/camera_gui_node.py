import sys
import random as rd
import numpy as np
import math
import time

import trash_item
from camera_node import Camera
from annotation_popup import CreateAnnotationPopUp, EditAnnotationPopUp

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
        self.trash_types = ['Cardboard',
                                'Metal', 'Rigid Plastic', 'Soft Plastic']
        self.selected_trash_item = None

    def paintEvent(self, event):
        qp = QPainter(self)
        qp.setFont(QFont("Helvetica", 14))
        pn = QPen(Qt.green, 3, Qt.SolidLine)
        qp.setBrush(Qt.NoBrush)
        qp.setPen(pn)

        if(self.pixmap != None):
            qp.drawPixmap(self.rect(), self.pixmap)

        if(self.parent.state == 1):
            qp.drawRect(QRect(self.start, self.end))

        print(self.parent.camera.trash_items_shown)
        for trash_item in self.parent.camera.trash_items_shown:
            if trash_item.conf > 2:
                self.drawTrashItem(trash_item, qp)
 
        if(self.selected_trash_item is not None):
            self.highlightBox(self.selected_trash_item, qp)

    def drawTrashItem(self, trash_item, qp):
        text_height = 25
        qp.setBrush(Qt.NoBrush)
        qp.setPen(self.getPen(trash_item.trash_type, trash_item.conf))
        qp.drawRect((trash_item.x - int(trash_item.width/2)) * self.parent.scale_w, (trash_item.y - int(trash_item.height/2))
                    * self.parent.scale_h, trash_item.width * self.parent.scale_w, trash_item.height * self.parent.scale_h)
        if((trash_item.y - int(trash_item.height/2)) > text_height + 5):
            qp.drawText((trash_item.x - int(trash_item.width/2)) * self.parent.scale_w, (trash_item.y - int(trash_item.height/2) - 1)
                        * self.parent.scale_h, str(self.trash_types[trash_item.trash_type])+" "+str(int(trash_item.conf)))
        else:
            qp.drawText((trash_item.x - int(trash_item.width/2)) * self.parent.scale_w, (trash_item.y + int(trash_item.height/2) +
                                                                                         text_height) * self.parent.scale_h, str(self.trash_types[trash_item.trash_type])+" "+str(int(trash_item.conf)))

    def getPen(self, type, conf):
        thickness = 3
        opacity = min(max(100, int((conf/10)*255)), 255)
        if(type == 0):
            pn = QPen(QColor(0, 255, 0, opacity), thickness, Qt.SolidLine)
        elif(type == 1):
            pn = QPen(QColor(255, 255, 0, opacity), thickness, Qt.SolidLine)
        elif(type == 2):
            pn = QPen(QColor(255, 0, 255, opacity), thickness, Qt.SolidLine)
        else:
            pn = QPen(QColor(0, 255, 255, opacity), thickness, Qt.SolidLine)
        return pn

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
            if(self.start.x() == self.end.x() and self.start.y() == self.end.y()):
                self.selected_trash_item = self.checkForBBox(
                    self.start.x(), self.start.y())
                if(self.selected_trash_item is not None):
                    self.update()
                    pop_up = EditAnnotationPopUp(
                        self.selected_trash_item, self.parent.camera.trash_items, self.getGlobalPos(self.selected_trash_item))
                    self.selected_trash_item = None
            else:
                start_time = time.time()
                pop_up = CreateAnnotationPopUp(
                    self.start.x(), self.start.y(), self.getGlobalPos())
                if(pop_up.label != -1):
                    self.updateXYForTime(round(time.time()-start_time, 3))
                    self.parent.camera.trash_items.append(
                        self.make_trash_item(pop_up.label))
            self.parent.state = 0
            self.update()

    def updateXYForTime(self, time):
        print("time: ", time, " s")

    def make_trash_item(self, label):
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
            y = self.start.y() - h/2
        else:
            h = self.end.y() - self.start.y()
            y = self.end.y() - h/2
        return trash_item.TrashItem(x/self.parent.scale_w, y/self.parent.scale_h, w/self.parent.scale_w, h/self.parent.scale_h, label, 100, True)

    def updatePixmap(self, pixmap):
        self.pixmap = pixmap
        self.update()

    def getGlobalPos(self, trash_item=None):
        # Calculates where to place pop-up
        buffer = 15
        x0 = self.parent.pixmap_initial_x
        y0 = 50 + 12
        z = 1
        if(trash_item is None):
            y_offset = (self.end.y()+self.start.y())/2
            if(self.end.x() > self.start.x()):
                x_offset = self.end.x()+buffer
            else:
                x_offset = self.end.x()-buffer
                z = -1
            return [x0+x_offset, y_offset+y0, z]
        else:
            y_offset = trash_item.y*self.parent.scale_h
            if(self.start.x() < trash_item.x*self.parent.scale_w):
                x_offset = (trash_item.x - int(trash_item.width/2)) * \
                    self.parent.scale_w-buffer
                z = -1
            else:
                x_offset = (trash_item.x + int(trash_item.width/2)
                            ) * self.parent.scale_w+buffer
            return [x0+x_offset, y_offset+y0, z]

    def checkForBBox(self, mouse_x, mouse_y):
        x1, y1, x2, y2 = 0, 0, 0, 0
        smallest_size = math.inf
        selected_trash = None
        for trash_item in self.parent.camera.trash_items_shown:
            x1 = (trash_item.x - int(trash_item.width/2)) * self.parent.scale_w
            y1 = (trash_item.y - int(trash_item.height/2)) * \
                self.parent.scale_h
            x2 = (trash_item.x + int(trash_item.width/2)) * self.parent.scale_w
            y2 = (trash_item.y + int(trash_item.height/2)) * \
                self.parent.scale_h
            if(mouse_x > x1 and mouse_x < x2 and mouse_y > y1 and mouse_y < y2):
                area = trash_item.width*trash_item.height
                if(area < smallest_size):
                    smallest_size = area
                    selected_trash = trash_item
        return selected_trash

    def highlightBox(self, trash_item, qp):
        background_opacity = 175
        x0, x1 = (trash_item.x - int(trash_item.width/2)) * \
            self.parent.scale_w, (trash_item.x +
                                  int(trash_item.width/2))*self.parent.scale_w
        y0, y1 = (trash_item.y - int(trash_item.height/2)) * \
            self.parent.scale_h, (trash_item.y +
                                  int(trash_item.height/2))*self.parent.scale_h

        qp.setBrush(
            QBrush(QColor(0, 0, 0, background_opacity), Qt.SolidPattern))
        qp.setPen(Qt.NoPen)

        path = QPainterPath()
        path.setFillRule(Qt.WindingFill)
        # (x, y, width, height)
        path.addRect(0, 0, x0, self.pixmap.height())
        path.addRect(x1, 0, self.pixmap.width() - x1, self.pixmap.height())
        path.addRect(x0, 0, trash_item.width *
                     self.parent.scale_w, y0)
        path.addRect(x0, y1,
                     trash_item.width*self.parent.scale_w+1, self.pixmap.height() - y1)
        qp.drawPath(path.simplified())

        self.drawTrashItem(trash_item, qp)

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
        self.pixmap_initial_x = 0

        # Create GUI objects
        self.setGeometry(0, 0, self.screen.size().width(), self.screen.size().height())
        self.setWindowTitle("Trash MQP Companion Interface")

        image_label = QLabel(self)
        image_label.setText("Camera Feed")
        image_label.move(int(self.screen.size().width()/2) - int(image_label.size().width()/2), 10)
        image_label.setFont(QFont("Helvetica", 14))
        image_label.setFixedSize(200, 50)

        short_label = QLabel(self)
        short_label.setText("Shortcuts")
        short_label.move(25, 50)
        short_label.setFont(QFont("Helvetica", 14))
        short_label.setFixedSize(200, 25)
        shortcut_label = QLabel(self)
        shortcut_label.setText(
            "1 - Cardboard \n2 - Metal\n3 - Rigid Plastic \n4 - Soft Plastic\nSpace - Boost Confidence\nDelete - Delete Annotation")
        shortcut_label.move(25, 60)
        shortcut_label.setFixedSize(200, 150)
        shortcut_label.setFont(QFont("Helvetica", 11))        

        self.start_button = QPushButton("Start", self)
        self.start_button.setGeometry(0, 0, 150, 50)
        self.start_button.move(int(self.screen.size().width(
        )/2) - int((self.start_button.size().width())), int(self.screen.size().height(
        )) - int(self.start_button.size().height()) - 150)
        self.start_button.clicked.connect(self.start)

        self.stop_button = QPushButton("Stop", self)
        self.stop_button.setGeometry(50, 50, 150, 50)
        self.stop_button.move(int(self.screen.size().width(
        )/2) + int((self.start_button.size().width())), int(self.screen.size().height(
        )) - int(self.start_button.size().height()) - 150)
        self.stop_button.clicked.connect(self.stop)

        self.dlabel_pixmap = DLabel(self)        
        self.pub_home = rospy.Publisher("/home", Int32, queue_size=1)

    def setup_subscribers(self):
        self.depth_img_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
        self.color_img_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
        ts = message_filters.TimeSynchronizer([self.depth_img_sub, self.color_img_sub], 1)
        ts.registerCallback(self.new_image_recieved)

        self.camera_info = rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, callback=self.update_camera_info, queue_size=1)

    def unsubscribe(self):
        self.depth_img_sub.unregister()
        self.color_img_sub.unregister()
        self.camera_info.unregister()


    def new_image_recieved(self, depth_image, color_img):
        cv_image = self.camera.new_image_recieved(depth_image, color_img)
        
        if(self.state == 0):
            pixmap = self.cv_image_to_pixmap(cv_image)
            self.camera.trash_items_shown = self.camera.trash_items
            self.dlabel_pixmap.updatePixmap(pixmap)
            self.dlabel_pixmap.resize(pixmap.width(), pixmap.height())
            self.pixmap_initial_x = int(self.screen.size().width()/2) - int(pixmap.width()/2)
            self.dlabel_pixmap.move(self.pixmap_initial_x, 50)

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
        print("Homing X,Y,Z Axes")
        self.pub_home.publish(Int32())
        self.setup_subscribers()

    def stop(self):
        self.unsubscribe()

if __name__ == '__main__':

    # Init ros
    rospy.init_node('camera', anonymous=True)

    # Init GUI
    app = QApplication(sys.argv)
    win = TWindow()
    win.show()

    # Spin
    app.exec()