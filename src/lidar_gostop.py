#!/usr/bin/env python

# st:1 frontstop st:2 backstop

import rospy, time
from sensor_msgs.msg import LaserScan
from xycar_motor.msg import xycar_motor
from std_msgs.msg import Int32

stop_msg = Int32()
distance = []
st = 0

def callback(data):
	global distance, motor_msg
	distance = data.ranges

def drive_stop(stp):
	global stop_msg 
	stop_msg.data = stp
	pub.publish(stop_msg)

rospy.init_node('lidar_driver')
rospy.Subscriber('/scan', LaserScan, callback, queue_size=1)
pub = rospy.Publisher('/lidar_stop', Int32, queue_size=1)

time.sleep(3)
rate = rospy.Rate(30)
while not rospy.is_shutdown():
    ok = 0
    for degree in range(150, 210):
        if distance[degree] <= 0.5:
            print(degree)
            ok += 1
        if ok > 40:
            st = 1
            drive_stop(st)
            break
    if ok <= 40:
        st = 0
        drive_stop(st)
    rate.sleep()
