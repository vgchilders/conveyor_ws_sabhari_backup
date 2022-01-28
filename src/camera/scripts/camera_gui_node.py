import sys
import random as rd
import numpy as np

import trash_item

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

class DLabel(QLabel):
    def __init__(self, parent=None):
        super(DLabel, self).__init__(parent)
        self.parent = parent
        self.start = QPoint()
        self.end = QPoint()
        self.pixmap = None

    def paintEvent(self, event):
        qp = QPainter(self)

        pn = QPen(Qt.black, 4, Qt.SolidLine)

        qp.setPen(pn)

        if(self.pixmap != None):
            qp.drawPixmap(self.rect(), self.pixmap)

        if(self.parent.state == 1):
            qp.drawRect(QRect(self.start, self.end))

        for trash_item in self.parent.camera.trash_items_shown:
            qp.drawRect((trash_item.x - int(trash_item.width/2)) * self.parent.scale_w, (trash_item.y - int(trash_item.height/2)) * self.parent.scale_h, trash_item.width * self.parent.scale_w, trash_item.height * self.parent.scale_h)
            qp.drawText(int((trash_item.x - (trash_item.width/2)) * self.parent.scale_w), int(trash_item.y - (trash_item.height/2) - 1) * self.parent.scale_h, str(trash_item.trash_type))

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