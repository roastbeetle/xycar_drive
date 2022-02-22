#!/usr/bin/env python

import rospy, time
from sensor_msgs.msg import LaserScan
from xycar_motor.msg import xycar_motor
from std_msgs.msg import Int32
from std_msgs.msg import String 

lidar_stop = 0
hough_cnt = 0
do_ar = 0
h_speed = 0
h_angle = 0
a_speed = 0
a_angle = 0
ltraffic = 'R'
rtraffic = 'R'
traffic = 'R' 
traffic_cnt = 1

motor_msg = xycar_motor()
def ltraffic_callback(msg):
    global ltraffic
    ltraffic = msg.data
    #print('left',msg.data)
def rtraffic_callback(msg):
    global rtraffic
    #print('right',msg.data)
    rtraffic = msg.data

def callback_hough(data):
	global h_speed, h_angle 
	h_speed = data.speed
	h_angle = data.angle

def callback_ar(data):
	global a_speed, a_angle 
	a_speed = data.speed
	a_angle = data.angle

def callback_hough_stop(data):
    global hough_cnt 
    if(data.data == 1):
        hough_cnt = data.data 

def callback_lidar_stop(data):
    global lidar_stop 
    lidar_stop = data.data 
        
def callback_action(data):
	global do_ar
	do_ar = data.data

def drive_go(speed, angle):
	global motor_msg
	motor_msg.speed = speed
	motor_msg.angle = angle
	pub.publish(motor_msg)

rospy.init_node('auto_drive')
rospy.Subscriber('/xycar_motor_hough',xycar_motor, callback_hough, queue_size=1)
rospy.Subscriber('/xycar_motor_ar',xycar_motor, callback_ar, queue_size=1)
rospy.Subscriber('/lidar_stop',Int32, callback_lidar_stop, queue_size=1)
rospy.Subscriber('/xycar_hough_stop',Int32, callback_hough_stop, queue_size=1)
rospy.Subscriber('/ar_action',Int32, callback_action, queue_size=1)
rospy.Subscriber('/Left_color', String,ltraffic_callback, queue_size=1)
rospy.Subscriber('/Right_color', String,rtraffic_callback, queue_size=1)
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
time.sleep(3)
rate = rospy.Rate(30)
traffic = rtraffic

while not rospy.is_shutdown():
    if traffic_cnt == 1:
        traffic = rtraffic
    elif traffic_cnt == 2:
        traffic = rtraffic
    elif traffic_cnt == 3:
        traffic = ltraffic
    if lidar_stop == 0:
        if (do_ar == 6):
            drive_go(a_speed, a_angle)

        elif (do_ar==2):
            print('ar',do_ar)
            if(ltraffic=='Y' or rtraffic=='G'):
                traffic_cnt = 2 
                a_angle = 50
            elif(rtraffic=='Y' or ltraffic == 'G'):
                traffic_cnt = 3
                a_angle = -50
            drive_go(a_speed, a_angle)
        else:
            if(hough_cnt == 1):
                print('hough, traffic_cnt:', traffic_cnt,'traffic : ',traffic)
                if traffic == 'G':
                    #print('now traffic', traffic_cnt)
                    drive_go(h_speed, h_angle)
                    hough_cnt = 0
                else:
                    drive_go(0,0)
            else:
                drive_go(h_speed, h_angle)
    else:
        print('lidar',lidar_stop)
        drive_go(0,0)

        rate.sleep()
