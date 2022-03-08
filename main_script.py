#!/usr/bin/env python3


# import libraries
from cmath import pi
from math import sqrt
from pickletools import int4

from zmq import ctx_opt_names
import rospy
import cv2
import time
import numpy as np

import shapeDetection
# --------------------------------

# ----------------------------------------------------------------
from beginner_tutorials.srv import commService,commServiceResponse

var = ['N', 0]
result = [0, 0.0, 0.0, 0.0]
print(var)

def handle_comm_Service(req):
    var[0] = req.f
    var[1] = req.x

    if var[0] == 'YP':
      print("Yolo activated")
      var[1] = min([max([req.x, 0]), 5])
      return commServiceResponse(req.x, 0, 0, 0)

    elif var[0] == 'YS':
      print("Sending Values")
      var[0] = 'N'
      var[1] = 0
      return commServiceResponse(result[0], result[1], result[2], result[3])

    # Color segmentaion Ready
    elif var[0] == 'CSR':
      print("Color Segmentation activated")
      var[1] = min([max([req.x, 0]), 2])
      return commServiceResponse(req.x, 0, 0, 0)

    # Prepare to send value
    elif var[0] == 'CSP':
      print("Ready to send localization")
      var[1] = min([max([req.x, 0]), 2])
      return commServiceResponse(req.x, 0, 0, 0)

    # Send value of color segmentaion (color_index, dx, dy, angle)
    elif var[0] == 'CSS':
      print("Sending Values")
      var[0] = 'CSR'
      var[1] = min([max([req.x, 0]), 2])
      return commServiceResponse(result[0], result[1], result[2], result[3])
# -----------------------------------------------------------------



def colorSegmentaion(input_image, color_flag):

  select = np.zeros([6, 3])
  select[:,0] = [0, 17, 110, 255, 0, 255]
  select[:,1] = [100, 115, 100, 255, 0, 255]
  select[:,2] = [20, 30, 150, 255, 0, 255]

  img = cv2.cvtColor(input_image, cv2.COLOR_BGR2HSV) 
  img = cv2.GaussianBlur(img, (3,3), 1)

  lower = [select[0,color_flag], select[2,color_flag], select[4,color_flag]]
  upper = [select[1,color_flag], select[3,color_flag], select[5,color_flag]]

  # create NumPy arrays from the boundaries
  lower = np.array(lower, dtype="uint8")
  upper = np.array(upper, dtype="uint8")

  # find the colors within the specified boundaries and apply the mask
  mask = cv2.inRange(img, lower, upper)

  _,thresh = cv2.threshold(mask, 40, 255, 0)

  kernel = np.ones((5,5),np.uint8)
  erosion = cv2.erode(thresh,kernel,iterations = 1)
  dilation = cv2.dilate(erosion,kernel,iterations = 5)
  erosion1 = cv2.erode(dilation,kernel,iterations = 1)
  
  contours = cv2.findContours(erosion1, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[1]

  if len(contours) > 0 and len(contours) < 10:
    min_area = 100000
    idx = -1
    
    for i,cnt in enumerate(contours):
      if cv2.contourArea(cnt) < min_area and cv2.contourArea(cnt) > 12000: 
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

    d = sqrt((upper_right_y - upper_left_y)**2 + (upper_left_x - upper_right_x)**2)
    print(d)

    box_angle = np.arctan2(upper_right_y - upper_left_y, upper_right_x - upper_left_x)

    print("angle : ", box_angle * 180 / np.pi)
    print(cx, cy)

    return cx, cy, box_angle

  else:
    # print("Error no contour found((")
    return -1, -1, -1


print("init")
rospy.init_node('comm_Service_server')
s = rospy.Service('comm_Service', commService, handle_comm_Service)

shapes = ['cir', 'hex', 'rec', 'rng', 'slt', 'sqr']

# capturing the camera
vid = cv2.VideoCapture(0)

width = 1280
height = 720
vid.set(cv2.CAP_PROP_FRAME_WIDTH, width)
vid.set(cv2.CAP_PROP_FRAME_HEIGHT, height)


depth = 554
f = 1422
cx = 634
cy = 336

while(True):
    idx = var[1]
    if not vid.grab():
      continue

    _, frame = vid.read()
    
    if var[0] == 'N':
      output_image = cv2.circle(frame, (634, 336), 5, (255,255,255), -1)

    elif var[0] == 'CSR':
      if idx != var[1]:
        continue

      x_array = []
      y_array = []
      ang_array = []
      
      for i in range(10):
        _, frame = vid.read()
        px, py, ang = colorSegmentaion(frame, idx)

        x_array.append(px)
        y_array.append(py)
        ang_array.append(ang)

      if max(x_array) != -1:
        x_array = np.array(x_array)
        y_array = np.array(y_array)
        ang_array = np.array(ang_array)

        x = int(np.median(x_array[x_array > 0]))
        y = int(np.median(y_array[x_array > 0]))

        output_image = cv2.circle(frame, (x, y), 5, (255,0,255), -1)
        output_image = cv2.circle(output_image,(634, 336), 5, (255,255,255), -1)

      else:
        output_image = cv2.circle(frame, (634, 336), 5, (255,255,255), -1)


    # take the last average median readings and back to normal operation
    elif var[0] == 'CSP':
      if idx != var[1]:
        continue

      if  max(x_array) == -1:
        result_i = -1
        result_ang = -1
        result_x = -1 
        result_y = -1
      else:
        result_i = var[1]
        result_ang = np.median(ang_array[x_array > 0])
        result_x = (cy - y) * depth / f 
        result_y = (cx - x) * depth / f
      
      
      result = [result_i, result_x, result_y, result_ang]
      var[0] = 'CSR'
      print(result)

    elif var[0] == 'YP':
      if idx != var[1]:
        continue
      
      x_array = []
      y_array = []
      ang_array = []

      for i in range(2):
        _, frame = vid.read()
        output_image, x, y, result_ang = shapeDetection.YOLO(frame, shapes[idx])

        x_array.append(x)
        y_array.append(y)
        ang_array.append(result_ang)
      print(x_array)
      if max(x_array) != -1:
        x_array = np.array(x_array)
        y_array = np.array(y_array)
        ang_array = np.array(ang_array)

        x = int(np.median(x_array[x_array > 0]))
        y = int(np.median(y_array[x_array > 0]))

        output_image = cv2.circle(output_image,(x, y), 5, (255,0,0), -1)

        result_i = var[1]
        result_ang = np.median(ang_array[x_array > 0])

        result_x = (cy - y) * (depth - 160) / f 
        result_y = (cx - x) * (depth - 160) / f

        if idx in [2, 4]:
          result_ang += pi/2

      else:
        result_i = -1
        result_ang = 0
        result_x = -1 
        result_y = -1

      result = [result_i, result_x, result_y, result_ang]
      
      var[0] = 'YR'
	

    cv2.imshow("Output", output_image)

    if cv2.waitKey(5) & 0xFF == ord('q'):
        break
  
# After the loop release the cap object
vid.release()

# Destroy all the windows
cv2.destroyAllWindows()
