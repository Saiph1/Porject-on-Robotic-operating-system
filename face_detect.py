#!/usr/bin/python3
# coding=utf8
import sys
import rospy
import cv2
import math
import numpy as np
from threading import RLock, Timer, Thread

from std_msgs.msg import *
from std_srvs.srv import *
from visual_processing.msg import Result
from visual_processing.srv import SetParam
from sensor_msgs.msg import Image

from armpi_pro import bus_servo_control
from kinematics import ik_transform
from armpi_pro import Misc
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

__isRunning = False

lock = RLock()
ik = ik_transform.ArmIK()

range_rgb = {
    'red': (1, 0, 248),
    'blue': (249, 0, 0),
    'green': (0, 253, 0),
    'black': (0, 1, 4),
    'white': (251, 255, 255),
}

def initMove(delay=True):
    with lock:
        servo_data = ik.setPitchRanges((0, 0.1, 0.22), -60, -180, 0)[1]
        bus_servo_control.set_servos(joints_pub, 2300, (
            (1, 300), (4, 500), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']),
            (6, servo_data['servo6'])))
    if delay:
        rospy.sleep(2)

img_w = 640
center_x = 0
center_y = 0
d_pulse = 3
have_move = False
servo6_pulse = 500
start_greet = False
action_finish = True

def reset():
    global center_y
    global center_x
    global have_move       
    global d_pulse
    global servo6_pulse
    global start_greet
    global action_finish 

    with lock:
        center_y = 0
        center_x = 0
        have_move = False
        d_pulse = 4
        start_greet = False
        servo6_pulse = 239

# initialization
def init():
    print("face detect Init")
    initMove()
    reset()

# parameters
def move():
    global have_move
    global start_greet
    global action_finish 
    global d_pulse, servo6_pulse 
        
    while __isRunning:
        if start_greet: 
            start_greet = False                
            action_finish = False
            
            # waving gesture.
            bus_servo_control.set_servos(joints_pub, 300, ((2, 300),))
            rospy.sleep(0.13)

            bus_servo_control.set_servos(joints_pub, 600, ((2, 700),))
            rospy.sleep(0.261)
            
            bus_servo_control.set_servos(joints_pub, 600, ((2, 300),))
            rospy.sleep(0.362)
            
            bus_servo_control.set_servos(joints_pub, 300, ((2, 500),))
            rospy.sleep(0.31)
            
            bus_servo_control.set_servos(joints_pub, 400, ((1, 200),))
            rospy.sleep(0.2)
            
            bus_servo_control.set_servos(joints_pub, 400, ((1, 500),))
            rospy.sleep(1)

            bus_servo_control.set_servos(joints_pub, 400, ((1, 200),))
            rospy.sleep(0.4)

            bus_servo_control.set_servos(joints_pub, 400, ((1, 500),))
            rospy.sleep(0.4)
            
            
            have_move = True
            action_finish = True
            
        else:
            if have_move:
                # customized move if detection was found. 
                have_move = False
                bus_servo_control.set_servos(joints_pub, 200, ((1, 500), (2, 500)))
                rospy.sleep(0.35)
                bus_servo_control.set_servos(joints_pub, 300, ((2, 400), (2, 300)))
                rospy.sleep(0.62)
                bus_servo_control.set_servos(joints_pub, 400, ((1, 500), (2, 500)))
                rospy.sleep(0.2)
                
            # if not found, continue:
            if servo6_pulse > 943 or servo6_pulse < 231:
                d_pulse = -d_pulse
            bus_servo_control.set_servos(joints_pub, 50, ((6, servo6_pulse),))           
            servo6_pulse += d_pulse       
           
            rospy.sleep(0.05)                
 
 
result_sub = None
heartbeat_timer = None

# image processing result
def run(msg):
    global center_x
    global center_y
    global start_greet
    
    center_x = msg.center_x
    center_y = msg.center_y
    
    # if the face is at the middle?
    if action_finish and abs(center_x - img_w/2) < 100:
        start_greet = True 


# Standard service function:
def enter_func(msg):
    global lock
    global __isRunning
    global result_sub

    rospy.loginfo("enter face detect")
    with lock:
        init()
        if result_sub is None:
            rospy.ServiceProxy('/visual_processing/enter', Trigger)()
            result_sub = rospy.Subscriber('/visual_processing/result', Result, run)
            
    return [True, 'enter']

# standard exit function 
def exit_func(msg):
    global lock
    global __isRunning
    global result_sub
    global heartbeat_timer
    
    rospy.loginfo("exit face detect")
    with lock:
        rospy.ServiceProxy('/visual_processing/exit', Trigger)()
        __isRunning = False
        reset()
        try:
            if result_sub is not None:
                result_sub.unregister()
                result_sub = None
            if heartbeat_timer is not None:
                heartbeat_timer.cancel()
                heartbeat_timer = None
        except:
            pass
    
    return [True, 'exit']

# running loop:
def start_running():
    global lock
    global __isRunning

    rospy.loginfo("start running face detect")
    with lock:
        __isRunning = True

        th = Thread(target=move)
        th.setDaemon(True)
        th.start()

def stop_running():
    global lock
    global __isRunning

    rospy.loginfo("stop running face detect")
    with lock:
        reset()
        __isRunning = False
        initMove(delay=False)
        rospy.ServiceProxy('/visual_processing/set_running', SetParam)()

# heartbeat service
def heartbeat_srv_cb(msg):
    global heartbeat_timer
    
    if isinstance(heartbeat_timer, Timer):
        heartbeat_timer.cancel()
    if msg.data:
        heartbeat_timer = Timer(5, rospy.ServiceProxy('/face_detect/exit', Trigger))
        heartbeat_timer.start()
    rsp = SetBoolResponse()
    rsp.success = msg.data

    return rsp

# APP running
def set_running(msg):
    rospy.loginfo("%s", msg)
    
    if msg.data:
        visual_running = rospy.ServiceProxy('/visual_processing/set_running', SetParam)
        visual_running('face','')
        start_running()
    else:
        stop_running()
    
    return [True, 'set_running']

if __name__ == '__main__':
    # init
    rospy.init_node('face_detect', log_level=rospy.DEBUG)
    # publisher
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    # communication
    enter_srv = rospy.Service('/face_detect/enter', Trigger, enter_func)
    exit_srv = rospy.Service('/face_detect/exit', Trigger, exit_func)
    running_srv = rospy.Service('/face_detect/set_running', SetBool, set_running)
    heartbeat_srv = rospy.Service('/face_detect/heartbeat', SetBool, heartbeat_srv_cb)

    debug = False
    if debug:
        enter_func(1)
        start_running()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        cv2.destroyAllWindows()

