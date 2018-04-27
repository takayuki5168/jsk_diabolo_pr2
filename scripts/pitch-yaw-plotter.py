#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

import numpy as np
from matplotlib import pyplot as plt

from time import sleep
import sys, signal
import math

def sigIntHandler(signal, frame):
    sys.exit(0)
    
class PitchYawPlotter:
    def __init__(self):
        self.now_pitch = 0
        self.now_yaw = 0

        self.initNode()
        self.initGraph()

    def initNode(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("diabolo/pitch", Float64, self.callbackForPitch)
        rospy.Subscriber("diabolo/yaw", Float64, self.callbackForYaw)

    def initGraph(self):
        self.t = np.zeros(10)
        self.pitch = np.zeros(10)
        self.yaw = np.zeros(10)
        self.zero = np.zeros(10)

        plt.ion()
        plt.figure()
    
        plt.subplot(2,1,1)
        self.li_pitch, = plt.plot(self.t, self.pitch)
        self.li_pitch_zero, = plt.plot(self.t, self.zero)
        plt.xlabel("time[s]")
        plt.ylabel("pitch[degree]")
    
        plt.subplot(2,1,2)
        self.li_yaw, = plt.plot(self.t, self.yaw)
        self.li_yaw_zero, = plt.plot(self.t, self.zero)
        plt.xlabel("time[s]")
        plt.ylabel("yaw[degree]")
    
        self.now_time = 0;

    def callbackForPitch(self, data):
        self.now_pitch = data.data
        #rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
        
    def callbackForYaw(self, data):
        self.now_yaw = data.data
        #rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
    
    def plot(self):
        while True:
            #rospy.spinOnce()
            self.t = np.append(self.t, self.now_time) 
            self.t = np.delete(self.t, 0)
            self.pitch = np.append(self.pitch, math.sin(self.now_time)) # self.now_pitch
            self.pitch = np.delete(self.pitch, 0)
            self.yaw = np.append(self.yaw, math.cos(self.now_time)) # self.now_yaw
            self.yaw = np.delete(self.yaw, 0)
    
            self.li_pitch.set_xdata(self.t)
            self.li_pitch.set_ydata(self.pitch)           
            self.li_pitch_zero.set_xdata(self.t)
            self.li_pitch_zero.set_ydata(self.zero)
    
            self.li_yaw.set_xdata(self.t)
            self.li_yaw.set_ydata(self.yaw)           
            self.li_yaw_zero.set_xdata(self.t)
            self.li_yaw_zero.set_ydata(self.zero)
    
            plt.subplot(2,1,1)
            plt.xlim(min(self.t), max(self.t))
            plt.ylim(min(min(self.pitch), min(self.zero)), max(max(self.zero), max(self.pitch)))
    
            plt.subplot(2,1,2)
            plt.xlim(min(self.t), max(self.t))
            plt.ylim(min(min(self.zero), min(self.yaw)), max(max(self.zero), max(self.yaw)))
    
            plt.draw()
    
            self.now_time += 1
            plt.pause(0.01)
            
if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigIntHandler)

    plotter = PitchYawPlotter()
    plotter.plot()
