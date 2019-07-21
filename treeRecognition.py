# import the necessary packages
import sys
sys.path.append('~/yolo/lib/python3.5/site-packages')
import numpy as np
from time import sleep
import argparse
import time
import os

ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'

if ros_path in sys.path:

    sys.path.remove(ros_path)

import cv2

sys.path.append('~/yolo/lib/python3.5/site-packages')



#variables
_net = None
_camera = cv2.VideoCapture(0)
_LABELS = None
_COLORS = None
_confidence = 0
_threshold = 0



""" setup the options of the recognitiom"""
def setupArgument():
        return None

# get image function
def get_image():
    return _camera.read()[1]

def activateRecognition(yoloRep, confidence, threshold):
        _confidence = confidence
        _threshold = threshold

        # load the COCO class labels our YOLO model was trained on
        labelsPath = os.path.sep.join([yoloRep, "obj.names"])
        _LABELS = open(labelsPath).read().strip().split("\n")

        # initialize a list of colors to represent each possible class label
        np.random.seed(42)
        _COLORS = np.random.randint(0, 255, size=(len(_LABELS), 3),
                                    dtype="uint8")

        # derive the paths to the YOLO weights and model configuration
        weightsPath = os.path.sep.join([yoloRep, "yolov3-tiny-obj_final.weights"])
        configPath = os.path.sep.join([yoloRep, "yolov3-tiny-obj.cfg"])

        # load our YOLO object detector trained on COCO dataset (80 classes)
        print("[INFO] loading YOLO from disk...")
        _net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)

        # load the camera and add settings
        _camera.set(3, 640)
        _camera.set(4, 480)
        while True:
                # load our input image and grab its spatial dimensions
                frame = get_image()
                cv2.imwrite("frame.jpg", frame)
                sleep(0.1)
                image = cv2.imread("frame.jpg")
                (H, W) = image.shape[:2]

                # determine only the *output* layer names that we need from YOLO
                ln = _net.getLayerNames()
                ln = [ln[i[0] - 1] for i in _net.getUnconnectedOutLayers()]

                # construct a blob from the input image and then perform a forward
                # pass of the YOLO object detector, giving us our bounding boxes and
                # associated probabilities
                blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416),
                                             swapRB=True, crop=False)
                _net.setInput(blob)
                start = time.time()
                layerOutputs = _net.forward(ln)
                end = time.time()
                # show timing information on YOLO
                print("[INFO] YOLO took {:.6f} seconds".format(end - start))

                # initialize our lists of detected bounding boxes, confidences, and
                # class IDs, respectively
                boxes = []
                confidences = []
                classIDs = []

                # loop over each of the layer outputs
                for output in layerOutputs:
                        # loop over each of the detections
                        for detection in output:
                                # extract the class ID and confidence (i.e., probability) of
                                # the current object detection
                                scores = detection[5:]
                                classID = np.argmax(scores)
                                confidence = scores[classID]
                                # filter out weak predictions by ensuring the detected
                                # probability is greater than the minimum probability
                                if confidence > _confidence:
                                        # scale the bounding box coordinates back relative to the
                                        # size of the image, keeping in mind that YOLO actually
                                        # returns the center (x, y)-coordinates of the bounding
                                        # box followed by the boxes' width and height
                                        box = detection[0:4] * np.array([W, H, W, H])
                                        (centerX, centerY, width, height) = box.astype("int")
                                        # use the center (x, y)-coordinates to derive the top and
                                        # and left corner of the bounding box
                                        x = int(centerX - (width / 2))
                                        y = int(centerY - (height / 2))
                                        # update our list of bounding box coordinates, confidences,
                                        # and class IDs
                                        boxes.append([x, y, int(width), int(height)])
                                        confidences.append(float(confidence))
                                        classIDs.append(classID)

                # apply non-maxima suppression to suppress weak, overlapping bounding
                # boxes
                idxs = cv2.dnn.NMSBoxes(boxes, confidences, _confidence, _threshold)

                # ensure at least one detection exists
                if len(idxs) > 0:
                        # loop over the indexes we are keeping
                        for i in idxs.flatten():
                                # extract the bounding box coordinates
                                (x, y) = (boxes[i][0], boxes[i][1])
                                (w, h) = (boxes[i][2], boxes[i][3])

                                # draw a bounding box rectangle and label on the image

                                # calcuate ratio of box
                                r = (x, y) / (H, W)

                                if r > 0.3:
                                        print("SOMETHING FOUND STOP")
                                        return "STOP"

                                color = [int(c) for c in _COLORS[classIDs[i]]]
                                cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
                                text = "{}: {:.4f}".format(_LABELS[classIDs[i]], confidences[i])
                                cv2.putText(image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,
                                            0.5, color, 2)
                # show the output image
                cv2.imwrite("prediction.png", image)
                #cv2.imshow('pred', image)
                #key = cv2.waitKey(5)