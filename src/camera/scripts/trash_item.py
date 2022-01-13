
IOU_THRESHOLD = .5

class TrashItem:

    def __init__(self, x, y, width, height, conf):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.conf = conf
    
    def compare_item(self, new_item: TrashItem):
        return self.calc_iou(self.get_bounding_box(self), self.get_bounding_box(new_item)) > IOU_THRESHOLD
            
    
    def update_item(self, new_item: TrashItem):
        # TODO: implement this
        return

    def get_bounding_box(item: TrashItem):
        x1 = self.x - round(self.width / 2)
        x2 = self.x + round(self.width / 2)

        y1 = self.y - round(self.height / 2)
        y2 = self.y + round(self.height / 2)

        return [x1, y1, x2, y2]
    
    # Function from https://gist.github.com/meyerjo/dd3533edc97c81258898f60d8978eddc 
    def calc_iou(boxA, boxB):
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
