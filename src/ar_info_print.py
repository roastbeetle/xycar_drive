#!/usr/bin/env python

import rospy, math
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from xycar_motor.msg import xycar_motor
from std_msgs.msg import String
from std_msgs.msg import Int32


arData = {"DX":0.0,"DY":0.0,"DZ":0.0,"AX":0.0,"AY":0.0,"AZ":0.0,"AW":0.0, "ID":0}
roll, pitch, yaw = 0, 0, 0

ltraffic = 'G'
rtraffic = 'G'
done_2 = 0
done_6 = 0
cross_cnt = 0
parking_cnt = 0
angle = 0 

def crossroad():
    global cross_cnt
    global done_2
    global pub_motor
    global pub_action
    global angle

    msg = xycar_motor()
    action = Int32()    

    if(cross_cnt>130):
        done_2 = 1
        action.data = 0
    elif(cross_cnt>100):
        action.data = 2
    cross_cnt = cross_cnt+1
    msg.angle = 40 
    msg.speed = 5 
    pub_action.publish(action)
    pub_motor.publish(msg)
    #print(cross_cnt) 

def park_ready():
    global parking_cnt
    global done_6
    global pub_action
    
    action = Int32()

    if(parking_cnt>15):
        action.data = 6
        done_6 = 1
        
    parking_cnt = parking_cnt+1
    pub_action.publish(action)

def park_S():
    global parking_cnt
    global done_6
    global pub_motor

    msg = xycar_motor()

    if(parking_cnt>105):
        done_6 = 2
    elif(parking_cnt>60):
        msg.angle = -50
        msg.speed = -5
    elif(parking_cnt>15):
        msg.angle = 50
        msg.speed = -5    
      
    parking_cnt = parking_cnt+1
    pub_motor.publish(msg)


def park_finish(X,Z):
    global parking_cnt
    global pub_motor
    
    msg = xycar_motor()
    
    if(Z>0.9):
        if X>0.0:
            msg.speed = 3
            msg.angle = math.atan2(x,z)*180/3.14
        elif X<-0.0:
            msg.speed = 3
            msg.angle = math.atan2(x,z)*180/3.14 
    else:
        msg.speed = 0
        msg.angle = 0
    
    pub_motor.publish(msg)

def callback(msg):
    global arData
    for i in msg.markers:
        arData["ID"] = i.id
        arData["DX"] = i.pose.pose.position.x
        arData["DY"] = i.pose.pose.position.y
        arData["DZ"] = i.pose.pose.position.z
        
        arData["AX"] = i.pose.pose.orientation.x
        arData["AY"] = i.pose.pose.orientation.y
        arData["AZ"] = i.pose.pose.orientation.z
        arData["AW"] = i.pose.pose.orientation.w

rospy.init_node('ar_drive')
rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)
pub_motor = rospy.Publisher('xycar_motor_ar', xycar_motor, queue_size=1)
pub_action = rospy.Publisher('ar_action', Int32, queue_size=1)

rate = rospy.Rate(30)

while not rospy.is_shutdown():
    (roll, pitch, yaw) = euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)
    #print(" roll : "+ str(roll))
    #print("pitch : "+ str(pitch))
    #print(" yaw : "+ str(yaw))
    #print(" x : "+ str(arData["DX"]))
    #print(" y : "+ str(arData["DY"]))
    #print(" z : "+ str(arData["DZ"]))
    #print(" id : "+ str(arData["ID"]))

    idx = str(arData["ID"])
    z = arData["DZ"]
    x = arData["DX"]

    if(idx == '2'):
        if(done_2 == 0): 
            if(z<0.71):
                crossroad()
    if(idx == '6'):
        if(done_6 == 0 and z < 0.71 ):
            park_ready()
        elif(done_6 ==1):
            park_S()
        elif(done_6 ==2):
            park_finish(x, z)

    rate.sleep()
