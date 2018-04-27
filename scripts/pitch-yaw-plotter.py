#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import numpy as np
from matplotlib import pyplot as plt
from time import sleep
import signal
import sys
import math

now_pitch = 0
now_yaw = 0

def sigIntHandler(signal, fram):
    sys.exit(0)

def callbackForPitch(data):
    now_pitch = data.data
    #rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
    
def callbackForYaw(data):
    now_yaw = data.data
    #rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("diabolo/pitch", Float64, callbackForPitch)
    rospy.Subscriber("diabolo/yaw", Float64, callbackForYaw)

    t = np.zeros(10)
    pitch = np.zeros(10)
    yaw = np.zeros(10)
    zero = np.zeros(10)

    plt.ion()
    plt.figure()

    plt.subplot(2,1,1)
    li_pitch, = plt.plot(t, pitch)
    li_pitch_zero, = plt.plot(t, zero)
    plt.xlabel("time[s]")
    plt.ylabel("pitch[degree]")

    plt.subplot(2,1,2)
    li_yaw, = plt.plot(t, yaw)
    li_yaw_zero, = plt.plot(t, zero)
    plt.xlabel("time[s]")
    plt.ylabel("yaw[degree]")

    now_time = 0;
    while True:
        #rospy.spinOnce()
        t = np.append(t, now_time) 
        t = np.delete(t, 0)
        pitch = np.append(pitch, math.sin(now_time)) # now_pitch
        pitch = np.delete(pitch, 0)
        yaw = np.append(yaw, math.cos(now_time)) # now_yaw
        yaw = np.delete(yaw, 0)

        li_pitch.set_xdata(t)
        li_pitch.set_ydata(pitch)           
        li_pitch_zero.set_xdata(t)
        li_pitch_zero.set_ydata(zero)

        li_yaw.set_xdata(t)
        li_yaw.set_ydata(yaw)           
        li_yaw_zero.set_xdata(t)
        li_yaw_zero.set_ydata(zero)

        plt.subplot(2,1,1)
        plt.xlim(min(t), max(t))
        plt.ylim(min(min(pitch), min(zero)), max(max(zero), max(pitch)))

        plt.subplot(2,1,2)
        plt.xlim(min(t), max(t))
        plt.ylim(min(min(zero), min(yaw)), max(max(zero), max(yaw)))

        plt.draw()

        now_time += 1
        plt.pause(0.01)
        
if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigIntHandler)
    listener()
