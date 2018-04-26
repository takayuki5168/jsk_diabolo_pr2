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
    y = np.zeros(10)

    plt.ion()
    plt.figure()
    li, = plt.plot(t, y)
    plt.xlabel("time[s]")
    plt.ylabel("pitch[degree]")

    now_time = 0;
    while True:
        #rospy.spinOnce()
        print("po")

        t = np.append(t, now_time)
        t = np.delete(t, 0)
        y = np.append(y, now_time)
        y = np.delete(y, 0)

        li.set_xdata(t)
        li.set_ydata(y)           
        plt.xlim(min(t), max(t))
        plt.ylim(min(y), max(y))
        plt.draw()

        now_time += 1
        plt.pause(0.01)
        
if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigIntHandler)
    listener()
