import trash_item as ti
import detectron2
import numpy as np

import cv2
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg

# MAYBE NEED:
# install dependencies: (use cu101 because colab has CUDA 10.1)
# !pip3 install -U torch==1.5 torchvision==0.6 -f https://download.pytorch.org/whl/cu101/torch_stable.html 
# !pip3 install cython pyyaml==5.1
# !pip3 install -U 'git+https://github.com/cocodataset/cocoapi.git#subdirectory=PythonAPI'
# 
# print(torch.__version__, torch.cuda.is_available())
# !gcc --version

# DEFINITLY NEED
# !pip3 install detectron2==0.1.3 -f https://dl.fbaipublicfiles.com/detectron2/wheels/cu101/torch1.5/index.html


class ObjectDetectionModel:
    
    def __init__(self):
        weights_path = "weights/frcnn_labtrash_weights.pth"
        cfg = get_cfg()
        cfg.MODEL.DEVICE = "cpu"
        cfg.merge_from_file(model_zoo.get_config_file("COCO-Detection/faster_rcnn_X_101_32x8d_FPN_3x.yaml"))
        cfg.MODEL.WEIGHTS = weights_path
        cfg.MODEL.ROI_HEADS.NUM_CLASSES = 5 #your number of classes + 1
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.7
        # cfg.MODEL.ROI_HEADS.NMS_THRESH_TEST = 0.6
        # cfg.MODEL.ROI_HEADS.IOU_THRESHOLDS = 0.6
        self.predictor = DefaultPredictor(cfg)

        print("Model Init")

    def classify(self, image):

        outputs = self.predictor(image)
        predictions = []

        boxes = outputs["instances"].get("pred_boxes")
        classes = outputs["instances"].get("pred_classes").tolist()
        scores = outputs["instances"].get("scores").tolist()

        for i, box in enumerate(boxes):
            XYXY_box = box.tolist()
            
            x = XYXY_box[2] + round(XYXY_box[0]/2) 
            y = XYXY_box[3] + round(XYXY_box[1]/2)
            width = XYXY_box[2] - XYXY_box[0]
            height = XYXY_box[3] - XYXY_box[1]
            
            new_trash_item = ti.TrashItem(x, y, width, height, classes[i], scores[i])
            predictions.append(new_trash_item)

        # print(predictions)
        return predictions