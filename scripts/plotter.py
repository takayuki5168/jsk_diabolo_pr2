#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import numpy as np
from matplotlib import pyplot as plt
from time import sleep
import signal
import sys
import math

def sigIntHandler(signal, fram):
    sys.exit(0)

def callbackForPitch(data):
    rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
    
def callbackForYaw(data):
    rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("diabolo/pitch", Float64, callbackForPitch)
    rospy.Subscriber("diabolo/yaw", Float64, callbackForYaw)

    t = np.zeros(10)
    pitch = np.zeros(10)
    yaw = np.zeros(10)

    plt.ion()
    plt.figure()

    plt.subplot(2,1,1)
    li_pitch, = plt.plot(t, pitch)
    plt.xlabel("time[s]")
    plt.ylabel("pitch[degree]")

    plt.subplot(2,1,2)
    li_yaw, = plt.plot(t, yaw)
    plt.xlabel("time[s]")
    plt.ylabel("yaw[degree]")

    now_time = 0;
    while True:
        #rospy.spinOnce()
        print("po")

        t = np.append(t, now_time)
        t = np.delete(t, 0)
        pitch = np.append(pitch, now_time)
        pitch = np.delete(pitch, 0)
        yaw = np.append(yaw, -now_time)
        yaw = np.delete(yaw, 0)

        li_pitch.set_xdata(t)
        li_pitch.set_ydata(pitch)           
        li_yaw.set_xdata(t)
        li_yaw.set_ydata(yaw)           

        plt.subplot(2,1,1)
        plt.xlim(min(t), max(t))
        plt.ylim(min(pitch), max(pitch))

        plt.subplot(2,1,2)
        plt.xlim(min(t), max(t))
        plt.ylim(min(yaw), max(yaw))

        plt.draw()

        now_time += 1
        plt.pause(0.01)
        
if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigIntHandler)
    listener()
