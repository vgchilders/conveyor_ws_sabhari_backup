from camera.scripts.trash_item import TrashItem
import trash_item

class ObjectDetectionModel:
    
    def classify(self, image):

        x, y, width, height, conf = 0

        # TODO: implement object detection model

        new_trash_item = TrashItem(x, y, width, height, conf)

        return [new_trash_item]