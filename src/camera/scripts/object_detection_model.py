import trash_item as ti
import detectron2
import numpy as np
import torch
import torchvision
from torchvision.utils import draw_bounding_boxes
import matplotlib.pyplot as plt
import torchvision.transforms.functional as F
import tensorflow as tf
import cv2 as cv2


# import cv2
# from detectron2 import model_zoo
# from detectron2.engine import DefaultPredictor
# from detectron2.config import get_cfg

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
        #self.init_faster_rcnn()
        self.init_yolo_v5()

        print("Model Init")

    # def init_faster_rcnn(self):
    #     weights_path = "weights/frcnn_labtrash_weights.pth"
    #     cfg = get_cfg()
    #     cfg.MODEL.DEVICE = "cpu"
    #     cfg.merge_from_file(model_zoo.get_config_file("COCO-Detection/faster_rcnn_X_101_32x8d_FPN_3x.yaml"))
    #     cfg.MODEL.WEIGHTS = weights_path
    #     cfg.MODEL.ROI_HEADS.NUM_CLASSES = 5 #your number of classes + 1
    #     cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.7
    #     # cfg.MODEL.ROI_HEADS.NMS_THRESH_TEST = 0.6
    #     # cfg.MODEL.ROI_HEADS.IOU_THRESHOLDS = 0.6
    #     self.predictor = DefaultPredictor(cfg)

    def init_yolo_v5(self):
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='weights/best.pt')
        #self.model = torch.hub.load('weights/custom_yolov5s.yaml', 'custom', path='weights/yolo_lab_data_aug.pt', source='local')

    def classify(self, image):
        return self.yolo_v5(image)
        #return self.faster_rcnn(image)

    # def faster_rcnn(self, image):
    #     outputs = self.predictor(image)
    #     predictions = []

    #     boxes = outputs["instances"].get("pred_boxes")
    #     classes = outputs["instances"].get("pred_classes").tolist()
    #     scores = outputs["instances"].get("scores").tolist()

    #     for i, box in enumerate(boxes):
    #         XYXY_box = box.tolist()
            
    #         x = round((XYXY_box[2] + XYXY_box[0])/2) 
    #         y = round((XYXY_box[3] + XYXY_box[1])/2) 
    #         width = XYXY_box[2] - XYXY_box[0]
    #         height = XYXY_box[3] - XYXY_box[1]
            
    #         new_trash_item = ti.TrashItem(x, y, width, height, classes[i], scores[i])
    #         predictions.append(new_trash_item)

    #     # print(predictions)
    #     return predictions
    
    def yolo_v5(self, image):
        results = self.model([image])
        predictions = []

        # results.print()
        # print(results.pandas().xyxy)
        # print(results.pandas().xyxy[0])

        bounding_boxes = []

        for i, trash in results.pandas().xyxy[0].iterrows():
            x = round((int(trash[2]) + int(trash[0]))/2)
            y = round((int(trash[3]) + int(trash[1]))/2)
            width = int(trash[2]) - int(trash[0])
            height = int(trash[3]) - int(trash[1])
            
            new_trash_item = ti.TrashItem(x, y, width, height, trash[5], float(trash[4]))
            predictions.append(new_trash_item)

            # cv2.rectangle(image,(int(trash[0]), int(trash[1])),(int(trash[2]), int(trash[3])),(200,0,0),2)

            bounding_boxes.append([int(trash[0]), int(trash[0]), int(trash[0]), int(trash[0])])

        # print(predictions)
        # print(type(image))
        # print(torch.from_numpy(image))
        # image_bb = draw_bounding_boxes(torch.from_numpy(image), torch.tensor(bounding_boxes, dtype=torch.int32), width=4)
        
        # image_bb = torchvision.transforms.ToPILImage()(image_bb)
        # image_bb.show()

        # cv2.imshow('img',image)
        # cv2.waitKey(0)   

        return predictions

    def show(self, imgs):
        if not isinstance(imgs, list):
            imgs = [imgs]
        fix, axs = plt.subplots(ncols=len(imgs), squeeze=False)
        for i, img in enumerate(imgs):
            img = img.detach()
            img = F.to_pil_image(img)
            axs[0, i].imshow(np.asarray(img))
            axs[0, i].set(xticklabels=[], yticklabels=[], xticks=[], yticks=[])