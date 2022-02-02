from geometry_msgs.msg import Pose


IOU_THRESHOLD = .5

class TrashItem:

    def __init__(self, x, y, width, height, trash_type, conf):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.trash_type = trash_type
        self.conf = conf
        self.pose = Pose()
    
    def compare_item(self, new_item):
        return self.calc_iou(self.get_bounding_box(), new_item.get_bounding_box()) > IOU_THRESHOLD and self.trash_type == new_item.trash_type
            
    def update_item(self, new_item):
        self.x = round((self.x + new_item.x) / 2)
        self.y = round((self.y + new_item.y) / 2)
        self.width = round((self.width + new_item.width) / 2)
        self.height = round((self.height + new_item.height) / 2)
        self.conf += new_item.conf

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
