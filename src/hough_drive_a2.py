#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rospkg
import numpy as np
import cv2, random, math
from cv_bridge import CvBridge
from xycar_motor.msg import xycar_motor
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

import sys
import os
import signal
import time

prev_angle = 0
slow = 0
stop = 0
i_error = 0.0
prev_error = 0.0
start_time = time.time()

def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

image = np.empty(shape=[0])
bridge = CvBridge()
pub = None
Width = 640
Height = 480
Offset = 340
Gap = 40

def img_callback(data):
    global image    
    image = bridge.imgmsg_to_cv2(data, "bgr8")

# publish xycar_motor msg
def drive(Angle, Speed, Stop): 
    global slow
    global pub_motor
    global pub_stop
    #print(slow)
    msg = xycar_motor()
    stp = Int32()

    stp.data= Stop
    if (slow > 330):
        slow = 0
    elif(slow > 0):
        print("sllllllllow")
        msg.angle = Angle
        msg.speed = 3
        slow = slow+1
    else:
        msg.angle = Angle
        msg.speed = Speed
            
    pub_motor.publish(msg)
    pub_stop.publish(stp)

def PID(input_data, kp, ki, kd):
    global start_time, end_time, prev_error, i_error
    end_time = time.time()
    dt = end_time - start_time
    start_time = end_time
    
    error = 320 - input_data
    d_error = error - prev_error

    p_error = kp*error
    i_error = ki*i_error*error*dt
    d_error = kd*d_error/dt

    output = 0.5*p_error + 0.1*i_error + 1*d_error
    prev_error = error

    if output>50:
        output = 50
    elif output<-50:
        output = -50
    return -output
   
     
# draw lines
def draw_lines(img, lines):
    global Offset
    for line in lines:
        x1, y1, x2, y2 = line[0]
        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        img = cv2.line(img, (x1, y1+Offset), (x2, y2+Offset), color, 2)
    return img

# draw rectangle
def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2

    cv2.rectangle(img, (lpos - 5, 15 + offset),
                       (lpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (rpos - 5, 15 + offset),
                       (rpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (center-5, 15 + offset),
                       (center+5, 25 + offset),
                       (0, 255, 0), 2)    
    cv2.rectangle(img, (315, 15 + offset),
                       (325, 25 + offset),
                       (0, 0, 255), 2)
    return img

# left lines, right lines
def divide_left_right(lines):
    global Width

    low_slope_threshold = 0
    high_slope_threshold = 10

    # calculate slope & filtering with threshold
    slopes = []
    new_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]

        if x2 - x1 == 0:
            slope = 0
        else:
            slope = float(y2-y1) / float(x2-x1)
        
        if abs(slope) > low_slope_threshold and abs(slope) < high_slope_threshold:
            slopes.append(slope)
            new_lines.append(line[0])

    # divide lines left to right
    left_lines = []
    right_lines = []

    for j in range(len(slopes)):
        Line = new_lines[j]
        slope = slopes[j]

        x1, y1, x2, y2 = Line

        if (slope < 0) and (x2 < Width/2 - 60):
            left_lines.append([Line.tolist()])
        elif (slope > 0) and (x1 > Width/2 + 60):
            right_lines.append([Line.tolist()])

    return left_lines, right_lines

# get average m, b of lines
def get_line_params(lines):
    # sum of x, y, m
    x_sum = 0.0
    y_sum = 0.0
    m_sum = 0.0

    size = len(lines)
    if size == 0:
        return 0, 0

    for line in lines:
        x1, y1, x2, y2 = line[0]

        x_sum += x1 + x2
        y_sum += y1 + y2
        m_sum += float(y2 - y1) / float(x2 - x1)

    x_avg = x_sum / (size * 2)
    y_avg = y_sum / (size * 2)
    m = m_sum / size
    b = y_avg - m * x_avg

    return m, b

# get lpos, rpos
def get_line_pos(img, lines, left=False, right=False):
    global Width, Height
    global Offset, Gap
    global prev_l, prev_r

    m, b = get_line_params(lines)

    if m == 0 and b == 0:
        if left:
            pos =0 
        if right:
            pos =Width
    else:
        y = Gap / 2
        pos = (y - b) / m

        b += Offset
        x1 = (Height - b) / float(m)
        x2 = ((Height/2) - b) / float(m)

        cv2.line(img, (int(x1), Height), (int(x2), (Height/2)), (255, 0,0), 3)

    return img, int(pos)

# show image and return lpos, rpos
def process_image(frame):
    global Width
    global Offset, Gap
    global stop, slow
    global ltraffic, rtraffic

    roi = frame[Offset : Offset+Gap, 0 : Width]
    roi_w = frame[Offset : Offset+Gap, 100: 540]
    hsv = cv2.cvtColor(roi_w, cv2.COLOR_BGR2HSV)

    w_lbound = np.array([0, 0, 150], dtype=np.uint8)
    w_ubound = np.array([225, 100, 255], dtype=np.uint8)

    y_lbound = np.array([20, 55, 50], dtype=np.uint8)
    y_ubound = np.array([50, 255, 180], dtype=np.uint8)
   
    y_bin = cv2.inRange(hsv, y_lbound, y_ubound)
    if cv2.countNonZero(y_bin)> 1000:
        slow = 1
    w_bin = cv2.inRange(hsv, w_lbound, w_ubound)
    #cv2.imshow('binf', w_bin)
    #print(cv2.countNonZero(w_bin))
    if cv2.countNonZero(w_bin) > 4500:
        stop = 1
    else:
        stop = 0
    # gray
    gray = cv2.cvtColor(roi,cv2.COLOR_BGR2GRAY)

    # blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)

    # canny edge
    low_threshold = 80
    high_threshold = 120
    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)

    # HoughLinesP
    all_lines = cv2.HoughLinesP(edge_img,1,math.pi/180,30,30,10)

    # divide left, right lines
    if all_lines is None:
        return 0, 640
    left_lines, right_lines = divide_left_right(all_lines)

    # get center of lines
    frame, lpos = get_line_pos(frame, left_lines, left=True)
    frame, rpos = get_line_pos(frame, right_lines, right=True)

    # draw lines
    #frame = draw_lines(frame, left_lines)
    #frame = draw_lines(frame, right_lines)
    #frame = cv2.line(frame, (230, 235), (410, 235), (255,255,255), 2)
                                 
    # draw rectangle
    frame = draw_rectangle(frame, lpos, rpos, offset=Offset)
    #roi2 = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)
    #roi2 = draw_rectangle(roi2, lpos, rpos)

   # show image
    #cv2.imshow('origin',frame)
    return lpos, rpos

def start():
    global pub_motor
    global pub_stop
    global image
    global Width, Height
    global prev_angle
    global stop, slow

    rospy.init_node('hough_drive')
    pub_motor = rospy.Publisher('xycar_motor_hough', xycar_motor, queue_size=1)
    pub_pid = rospy.Publisher('pid', Int32, queue_size=1)
    pub_stop = rospy.Publisher('xycar_hough_stop', Int32, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    print "---------- Xycar A2 v1.0 ----------"
    rospy.sleep(3)
    rate = rospy.Rate(30)
    drive(0, 0,0)
	

    while True:
        while not image.size == (640*480*3):
            continue

        lpos, rpos = process_image(image)
        center = (lpos + rpos) / 2
        msg_pid = Int32()
        msg_pid.data = center
        pub_pid.publish(msg_pid)
        angle = PID(center,1,0,0.001)
        drive(angle, 5, stop)
        #drive(0,0,0)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        rate.sleep()
if __name__ == '__main__':

    start()


