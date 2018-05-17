# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float64

import matplotlib.pyplot as plt
import numpy as np
import math

import signal, sys, time

import copy

# TODO
# 状態ベクトルを過去の状態群にするか、状態と状態の微分にするか
class ParticleSystem:
    def __init__(self):
        # parameters
        self.init_params()
        
        # variables
        self.particles = [] # particles_num * self.var_num
        self.past_particle = [0 for i in range(self.var_num)] # self.var_num
        self.ref_state = [0 for i in range(self.state_dim)] # self.state_dim
        self.idle = 0

        # Subscribers
        rospy.init_node('particle_system', anonymous=True)
        rospy.Subscriber("calc_diabolo_state/pitch", Float64, self.callbackForPitch)
        rospy.Subscriber("calc_diabolo_state/yaw", Float64, self.callbackForYaw)
        rospy.Subscriber("idle", Float64, self.callbackForIdle)        

        # Publisher
        self.pub_arm = rospy.Publisher('arm', Float64, queue_size=10)
        self.pub_base = rospy.Publisher('base', Float64, queue_size=10)
        
    def init_params(self):
         self.state_dim = 2
         self.past_state_num = 2
         self.input_dim = 2
         self.past_input_num = 1
         self.var_num = self.state_dim * (self.past_state_num + 1) + self.input_dim * (self.past_input_num + 1)

         
    def execute(self):
        while True:
            print 'executing'
            if self.idle == 0:
                continue
            
            self.calc_now_input()
            self.publish()
            self.update_past_particle()
            
            time.sleep(0.01) # TODO
            
    def calc_now_input(self):
        min_distance = 100000 # TODO
        for p_idx in self.particles:
            p = self.particles[p_idx]
            p_ = copy.deepcopy(p)
            
            # minus self.ref_state
            for i in range(self.var_num): #x_t
                if i < self.state_dim:
                    p_[i] = p_[i] - self.ref_state[i]
                elif self.state_dim * (self.state_num + 1) <= i and i < self.state_dim * (self.state_num + 1) + self.input_dim: # u_t
                    continue
                else:
                    p_[i] = self.past_particle[i]

            # clac min_distance
            distance = 0
            for i in range(self.var_num):
                distance += p_[i] * p_[i]
            if min_distance > distance:
                min_distance = distance
                min_idx = p_idx

        # calc now input
        self.now_input = [0 for i in range(self.input_dim)]
        for i_idx in range(self.input_dim):
            self.now_input[i_idx] = self.particles[p_idx][i_idx]

    def publish(self):
        print 'now input ', self.now_input[0], self.now_input[1]
        self.pub_arm.publish(self.now_input[0])
        self.pub_base.publish(self.now_input[1])

    def update_past_particle(self):
        for ps_idx in reversed(range(self.past_state_num - 1)):
            for s_idx in range(self.state_dim):
                self.past_particle[self.state_dim * ps_idx + s_idx] = self.past_particle[self.state_dim * ps_idx + s_idx - self.state_dim]
        for s_idx in range(self.state_dim):
            self.past_particle[s_isx] = self.now_state[s_idx]
            
        for pi_idx in reversed(range(self.past_input_num - 1)):
            for i_idx in range(self.input_dim):
                self.past_particle[self.state_dim * (self.past_state_num + 1) + self.input_dim * pi_idx + i_idx] = self.past_particle[self.state_dim * (self.past_state_num + 1) + self.input_dim *pi_idx + i_idx - self.input_dim]
        for i_idx in range(self.input_dim):
            self.past_particle[self.state_dim * (self.past_state_num + 1) + i_idx] = self.now_input[i_idx]
    
    
    def callbackForPitch(self, msg):
        if msg.data == np.nan:
            msg.data = 0
        self.now_state[0] = msg.data

    def callbackForYaw(self, msg):
        if msg.data == np.nan:
            msg.data = 0
        self.now_state[1] = msg.data

    def callbackForIdle(self, msg):
        if msg.data == 1:
            self.idle = 1
        else:
            self.idle = 0

            
    def load_data(self, log_files):
        for log_file in log_files:
            with open(log_file) as f:             
                for l in f.readlines():
                    w = l.split(' ')
                    if len(w) != 4:
                        continue

    def save_data(self):
        pass

     
if __name__ == '__main__':
    signal.signal(signal.SIGINT, lambda signal, frame: sys.exit(0))
    
    ps = ParticleSystem()
    ps.load_data(['../log/log-by-logger/log-by-loggerpy0.log',
                  '../log/log-by-logger/log-by-loggerpy1.log',
                  '../log/log-by-logger/log-by-loggerpy2.log',
                  '../log/log-by-logger/log-by-loggerpy3.log',
                  '../log/log-by-logger/log-by-loggerpy4.log'])
    ps.execute()
