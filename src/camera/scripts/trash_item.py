from geometry_msgs.msg import Pose
from statistics import variance


IOU_THRESHOLD = 0.5
INITIAL_ERROR = 0.5 #initial value for kalman error of the estimate

class KalmanParameters:
    def __init__(self, est=None):
        self.e_est = INITIAL_ERROR 
        self.est = est
        if est is None:
            self.measurements = []
        else:
            self.measurements = [est]
        

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
        #initialize confidence for each class
        self.kp_conf = [KalmanParameters(), KalmanParameters(), KalmanParameters(), KalmanParameters()]
        self.kp_conf[trash_type].est = conf #set estimate for predicted type to be conf
        self.kp_y = KalmanParameters(y)
    
    def update_kalman(self, mea, kp):
        kp.measurements.append(mea)
        if kp.est is None:
            kp.est = mea
        else:
            e_mea = variance(kp.measurements)
            kg = kp.e_est/(kp.e_est + e_mea)
            kp.est = kp.est + kg*(mea - kp.est)
            kp.e_est = (1-kg)*kp.e_est
        return kp.est
    
    def compare_item(self, new_item):
        return self.calc_iou(self.get_bounding_box(), new_item.get_bounding_box()) > IOU_THRESHOLD
            
    def update_item(self, new_item):
        #if the matched trash was updated/created by user, only update the x pos
        if self.updated:
            self.x = new_item.x
        else:
            #Update kalman parameters for conf and y pos
            self.update_kalman(new_item.conf, self.kp_conf[new_item.trash_type])
            self.update_kalman(new_item.y, self.kp_y) #update kalman estimate of y

            #set self.conf and self.trash_type to type with highest (confidence estimate)*(num frames)
            max_conf = 0
            for trash_type in range(len(self.kp_conf)):
                if self.kp_conf[trash_type].est is not None:
                    confidence = self.kp_conf[trash_type].est*len(self.kp_conf[trash_type].measurements)
                    if confidence >= max_conf:
                        max_conf = confidence
                        self.conf = self.kp_conf[trash_type].est
                        self.trash_type = trash_type

            #Use the bounding box of the most confident trash_type     
            if new_item.trash_type == self.trash_type:            
                self.x = new_item.x
                self.y = new_item.y
                self.width = new_item.width
                self.height = new_item.height
                    
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
