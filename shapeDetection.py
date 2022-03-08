#!/usr/bin/env python3

from math import sqrt
from ctypes import *
import os
import cv2
import numpy as np
from darknet import darknet

def convertBack(x, y, w, h):
    xmin = int(round(x - (w / 2)))
    xmax = int(round(x + (w / 2)))
    ymin = int(round(y - (h / 2)))
    ymax = int(round(y + (h / 2)))
    return xmin, ymin, xmax, ymax


def angle_detection(img, pt1, pt2):
    shape_image = np.zeros_like(img)
    
    y1 = max([pt1[0] - 25, 0])
    y2 = min([pt2[0] + 25, 1280])
    x1 = max([pt1[1] - 25, 0])
    x2 = min([pt2[1] + 25, 728])
    
    shape_image[x1:x2, y1:y2] = img[x1:x2, y1:y2]
    
    shape_image = cv2.cvtColor(shape_image, cv2.COLOR_BGR2GRAY)
    
    shape_image = cv2.GaussianBlur(shape_image, (3,3), 1)
    _,thresh = cv2.threshold(shape_image,0,255,cv2.THRESH_OTSU)

    thresh[x1:x2, y1:y2] = 255 - thresh[x1:x2, y1:y2]
    
    kernel = np.ones((5,5),np.uint8)
    erosion = cv2.erode(thresh,kernel,iterations = 1)
    dilation = cv2.dilate(erosion,kernel,iterations = 2)
    erosion1 = cv2.erode(dilation,kernel,iterations = 1)

    contours = cv2.findContours(erosion1, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[1]
    print(len(contours))
    if len(contours) > 0:
        min_area = 3000
        idx = -1
    
        for i,cnt in enumerate(contours):
            print(cv2.contourArea(cnt))
            if cv2.contourArea(cnt) > min_area: 
                a = cv2.approxPolyDP(cnt, epsilon=0.08*cv2.arcLength(cnt, closed=True), closed=True)
                min_area=cv2.contourArea(cnt)
                idx = i

        if idx == -1:
            # print("No contour found((")
            return -1, -1, -1

        rect = cv2.minAreaRect(contours[idx])
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        cx = int(rect[0][0])
        cy = int(rect[0][1])

        upper_left_x = upper_left_x = int(box[0,0])
        upper_left_y = upper_left_x = int(box[0,0])
        upper_right_x = upper_left_x = int(box[1,0])
        upper_right_y = upper_left_x = int(box[1,0])

        for i in range(4):
            if(box[i,0] <= cx and box[i,1] <= cy):
                upper_left_x = int(box[i,0])
                upper_left_y = int(box[i,1])
            if(box[i,0] >= cx and box[i,1] <= cy):
                upper_right_x = int(box[i,0])
                upper_right_y = int(box[i,1])
        

        d = min (rect[1][:])
        # d = sqrt((upper_right_y - upper_left_y)**2 + (upper_left_x - upper_right_x)**2)
        print(d, rect[1][:])
        
        shape_angle = np.arctan2(upper_right_y - upper_left_y, upper_right_x - upper_left_x)
        return cx, cy, shape_angle

    else: 
        return -1, -1, 0

def cvDrawBoxes(detections, img):
    
    result_s = []
    result_x = []
    result_y = []
    result_ang = []

    for detection in detections:
        x, y, w, h = detection[2][0],\
            detection[2][1],\
            detection[2][2],\
            detection[2][3]
        xmin, ymin, xmax, ymax = convertBack(
            float(x), float(y), float(w), float(h))

        pt1 = (max([xmin, 0]), max([ymin, 0]))
        pt2 = (min([xmax, 1280]), min([ymax, 720]))
        
        print(detection[0].decode())
        print(detection[1])
        
        cx, cy, ang = angle_detection(img, pt1, pt2)

        if detection[0].decode() in ['cir', 'rng']:
            result_ang.append(0)
        else:
            result_ang.append(ang)

        result_s.append(detection[0].decode())
        result_x.append(cx)
        result_y.append(cy)

        cv2.rectangle(img, pt1, pt2, (0, 255, 0), 1)
        cv2.putText(img,
                    detection[0].decode(),
                    (pt1[0], pt1[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    [0, 255, 0], 2)

    return img, result_s, result_x, result_y, result_ang


netMain = None
metaMain = None
altNames = None

configPath = "./catkin_ws/src/beginner_tutorials/scripts/cfg/yolov4_custom.cfg"
weightPath = "./catkin_ws/src/beginner_tutorials/scripts/cfg/yolov4_custom.weights"
metaPath = "./catkin_ws/src/beginner_tutorials/scripts/cfg/coco.data"
if not os.path.exists(configPath):
        raise ValueError("Invalid config path `" +
                         os.path.abspath(configPath)+"`")
if not os.path.exists(weightPath):
        raise ValueError("Invalid weight path `" +
                         os.path.abspath(weightPath)+"`")
if not os.path.exists(metaPath):
        raise ValueError("Invalid data file path `" +
                         os.path.abspath(metaPath)+"`")
if netMain is None:
        netMain = darknet.load_net_custom(configPath.encode(
            "ascii"), weightPath.encode("ascii"), 0, 1)  # batch size = 1
if metaMain is None:
        metaMain = darknet.load_meta(metaPath.encode("ascii"))
if altNames is None:
        try:
            with open(metaPath) as metaFH:
                metaContents = metaFH.read()
                import re
                match = re.search("names *= *(.*)$", metaContents,
                                  re.IGNORECASE | re.MULTILINE)
                if match:
                    result = match.group(1)
                else:
                    result = None
                try:
                    if os.path.exists(result):
                        with open(result) as namesFH:
                            namesList = namesFH.read().strip().split("\n")
                            altNames = [x.strip() for x in namesList]
                except TypeError:
                    pass
        except Exception:
            pass

def YOLO(image, target_shape, width = 1280, height = 720):
    darknet_image = darknet.make_image(width, height, 3)

    frame_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    darknet.copy_image_from_bytes(darknet_image,frame_rgb.tobytes())

    detections = darknet.detect_image(netMain, metaMain, darknet_image, thresh=0.8)

    image, shape_name_a, point_x_a, point_y_a, angle_a = cvDrawBoxes(detections, frame_rgb)
    result_x = -1
    result_y = -1
    result_ang = 0

    for shape_name, point_x, point_y, shape_angle in zip(shape_name_a, point_x_a, point_y_a, angle_a):
        image = cv2.circle(image, (int(point_x), int(point_y)), 5, (255, 0, 0), -1)
        if shape_name == target_shape:
            result_x = point_x
            result_y = point_y
            result_ang = shape_angle

    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image, result_x, result_y, result_ang