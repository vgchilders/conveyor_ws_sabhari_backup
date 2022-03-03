from geometry_msgs.msg import Pose
from statistics import variance


IOU_THRESHOLD = 0.5
INITIAL_ERROR = 0.5 #initial value for kalman error of the estimate & error of measurement
class KalmanParameters:
    def __init__(self):
        self.e_est = INITIAL_ERROR 
        self.est = None
        self.measurements = []

class TrashItem:

    def __init__(self, x, y, width, height, trash_type, conf, updated):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.trash_type = trash_type
        self.conf = conf
        self.updated = updated
        self.pose = Pose()
        self.kp_conf = KalmanParameters()
        self.kp_y = KalmanParameters()

    
    def update_kalman(self, mea, kp):
        if(kp.est is None):
            kp.est = mea
        kp.measurements.append(mea)
        if len(kp.measurements == 1):
            kp.e_mea = INITIAL_ERROR
        else:
            kp.e_mea = variance(kp.measurements)
        kg = e_est/(e_est + e_mea)
        kp.est = est + kg*(mea - kp.est)
        kp.e_est = (1-kg)*kp.e_est
        return kp.est
    
    def compare_item(self, new_item):
        return self.calc_iou(self.get_bounding_box(), new_item.get_bounding_box()) > IOU_THRESHOLD
            
    def update_item(self, new_item):
        delta_x = new_item.x - self.x
        self.x = new_item.x
        self.update_kalman(new_item.y, self.kp_y)
        self.y = new_item.y
        self.width = new_item.width
        self.height = new_item.height
        # self.conf += new_item.conf
        self.conf = self.update_kalman(new_item.conf, self.kp_conf)

        return delta_x

    def get_bounding_box(self):
        x1 = self.x - round(self.width / 2)
        x2 = self.x + round(self.width / 2)

        y1 = self.y - round(self.height / 2)
        y2 = self.y + round(self.height / 2)

        return [x1, y1, x2, y2]
    
    # Function from https://gist.github.com/meyerjo/dd3533edc97c81258898f60d8978eddc 
    def calc_iou(self, boxA, boxB):
        # determine the (x, y)-coordinates of the intersection rectangle
        xA = max(boxA[0], boxB[0])
        yA = max(boxA[1], boxB[1])
        xB = min(boxA[2], boxB[2])
        yB = min(boxA[3], boxB[3])

        # compute the area of intersection rectangle
        interArea = abs(max((xB - xA, 0)) * max((yB - yA), 0))
        if interArea == 0:
            return 0
        # compute the area of both the prediction and ground-truth
        # rectangles
        boxAArea = abs((boxA[2] - boxA[0]) * (boxA[3] - boxA[1]))
        boxBArea = abs((boxB[2] - boxB[0]) * (boxB[3] - boxB[1]))

        # compute the intersection over union by taking the intersection
        # area and dividing it by the sum of prediction + ground-truth
        # areas - the interesection area
        iou = interArea / float(boxAArea + boxBArea - interArea)

        # return the intersection over union value
        return iou
