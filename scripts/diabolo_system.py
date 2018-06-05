#!/usr/bin/env python
# -*- coding: utf-8 -*-

# TODO
# kerasの例はランダムにプロットしているので近く見える
#   でも近い値は結構あるので確認のため隣合わせの例でプロットさせる
#      ぎりぎり許せるくらい...
# chainerは数値微分？reluはどうやって登録？

# increase log_files

# CHECK
# (self.init_state is [0, 0]   # only use for simulation)
# self.state_ref is [0, 0]
# action is 2
# self.MAX_INPUT_DIFF_RESTRICTION

# TODOTODOTODO
# armの符号が逆、だけどなぜかpitchは収束する...
#   NNの中間層の数を変えると治る
# sometime, subscribe is very slow


# consider frequency of noise
#  apply lowpassfilter

# consider covariance of observation

# if not using zerograds, what happens


import random, time, sys
import numpy as np
import argparse

import chainer
from chainer import Variable, Link, Chain, ChainList, optimizers, serializers
import chainer.functions as F
import chainer.links as L
from chainer.functions.loss.mean_squared_error import mean_squared_error

from matplotlib import pyplot as plt

import rospy
from std_msgs.msg import Float64MultiArray, Float64

'''
LOG_FILES = ['../log/log-by-logger/log-by-loggerpy0.log',
             '../log/log-by-logger/log-by-loggerpy1.log',
             '../log/log-by-logger/log-by-loggerpy2.log',
             '../log/log-by-logger/log-by-loggerpy3.log',
             '../log/log-by-logger/log-by-loggerpy4.log',
             '../log/log-by-logger/log-by-loggerpy5.log',
             '../log/log-by-logger/log-by-loggerpy6.log']             
'''
LOG_FILES = ['../log/log-by-logger/log-by-loggerpy1_0.log',
             '../log/log-by-logger/log-by-loggerpy1_1.log',
             '../log/log-by-logger/log-by-loggerpy1_2.log',
             '../log/log-by-logger/log-by-loggerpy1_3.log']

class MyChain(Chain):
    def __init__(self):
        super(MyChain, self).__init__(   # FIX
            l1=L.Linear(6, 2),
            l2=L.Linear(2)
            )
        
    def forward(self, x):   # FIX
        h = F.sigmoid(self.l1(x))
        o = self.l2(h)
        return o

    # forward ans save output
    def __call__(self, x):
        self.res = self.forward(x)

    def loss(self, t):
        return F.mean_squared_error(self.res, t)
    
    ''' loss for optimize_input()
    def loss(self, t):
        return F.mean_squared_error(self.res, t)
    '''

class DiaboloSystem():
    def __init__(self):
        # PAST NUM
        self.PAST_STATE_NUM = 2   # FIX
        self.PAST_INPUT_NUM = 1   # FIX
        self.PAST_NUM = max(self.PAST_STATE_NUM, self.PAST_INPUT_NUM)

        # INPUT DIM
        self.STATE_DIM = 2
        self.INPUT_DIM = 2
        
        self.DELTA_STEP = 5   # FIX

        # HyperParameters for NeuralNetwork
        self.INPUT_NN_DIM = self.STATE_DIM * self.PAST_STATE_NUM + self.INPUT_DIM * self.PAST_INPUT_NUM
        self.OUTPUT_NN_DIM = self.STATE_DIM
        #self.VAR_NUM = max(self.INPUT_NN_DIM, self.OUTPUT_NN_DIM)
        self.train_test_ratio = 0.8   # FIX
        self.batch_size = 1000   # FIX

        # reference of state
        self.state_ref = [0., 0.]   # FIX
        #self.state_ref = [10., 0.]   # FIX        

        # real data
        self.past_states = []
        self.now_input = [0.7, 0]

        # max min restriction for input
        #self.MAX_INPUT_DIFF_RESTRICTION = [0.03, 0.001]
        self.MAX_INPUT_DIFF_RESTRICTION = [10, 10]   # TODOTODOTODO
        self.MAX_INPUT_RESTRICTION = [0.85, 0.34]   # TODO
        self.MIN_INPUT_RESTRICTION = [0.60, -0.34]   # TODO        

        # init ros variable
        rospy.init_node("DiaboloSystem")
        self.pub_input = rospy.Publisher('/diabolo_system/diabolo_input', Float64MultiArray, queue_size=1)

    # print percentage of progress every 10%
    def percentage(self, idx, loop_num):
        split_num = 10
        if (idx + 1) % int(loop_num / split_num) == 0:
            print('{}% {}/{}'.format((idx + 1) * 100 / loop_num, idx + 1, loop_num))

    # load data from log files
    def load_data(self, log_files):
        self.input_arm = []
        self.input_base = []
        self.state_pitch = []
        self.state_yaw = []

        lpf_average_num = 10
        self.input_arm_lpf = []
        self.input_base_lpf = []
        self.state_pitch_lpf = []
        self.state_yaw_lpf = []
        
        for log_file in log_files:
            with open(log_file, "r") as f:
                cnt = 0
                for l in f.readlines():
                    cnt += 1
                    val = l.split(' ')
                    if len(val) != 4:
                        break
                
                    self.input_arm.append(float(val[0]))
                    self.input_base.append(float(val[1]))
                    self.state_pitch.append(float(val[2]))
                    self.state_yaw.append(float(val[3][:-1]))

                    if cnt > lpf_average_num:
                        a = 0
                        b = 0
                        p = 0
                        y = 0
                        for i in range(lpf_average_num):
                            a += self.input_arm[-i]
                            b += self.input_base[-i]
                            p += self.state_pitch[-i]
                            y += self.state_yaw[-i]
                        self.input_arm_lpf.append(a / lpf_average_num)
                        self.input_base_lpf.append(b / lpf_average_num)
                        self.state_pitch_lpf.append(p / lpf_average_num)
                        self.state_yaw_lpf.append(y / lpf_average_num)

    # arrange data for NeuralNetwork
    def arrange_data(self):
        self.X = []
        self.Y = []
        
        for i in range((max(self.PAST_STATE_NUM, self.PAST_INPUT_NUM) + 1) * self.DELTA_STEP, len(self.input_arm_lpf)):
            x = []
            y = [self.state_pitch_lpf[i], self.state_yaw_lpf[i]]            
            for j in range(self.PAST_STATE_NUM):
                x.append(self.state_pitch_lpf[i - (j + 1) * self.DELTA_STEP])
                x.append(self.state_yaw_lpf[i - (j + 1) * self.DELTA_STEP])
            for j in range(self.PAST_INPUT_NUM):
                x.append(self.input_arm_lpf[i - (j + 1) * self.DELTA_STEP])
                x.append(self.input_base_lpf[i - (j + 1) * self.DELTA_STEP])
            self.X.append(x)
            self.Y.append(y)
        '''
        for i in range((max(self.PAST_STATE_NUM, self.PAST_INPUT_NUM) + 1) * self.DELTA_STEP, len(self.input_arm)):
            x = []
            y = [self.state_pitch[i], self.state_yaw[i]]            
            for j in range(self.PAST_STATE_NUM):
                x.append(self.state_pitch[i - (j + 1) * self.DELTA_STEP])
                x.append(self.state_yaw[i - (j + 1) * self.DELTA_STEP])
            for j in range(self.PAST_INPUT_NUM):
                x.append(self.input_arm[i - (j + 1) * self.DELTA_STEP])
                x.append(self.input_base[i - (j + 1) * self.DELTA_STEP])
            self.X.append(x)
            self.Y.append(y)
        '''         
    # make NeuralNetwork model by using MyChain class
    def make_model(self):
        self.model = MyChain()
        self.optimizer = optimizers.RMSprop(lr=0.01)
        self.optimizer.setup(self.model)

    def get_batch_train(self, n):
        x = []
        y = []
        for i in range(n):
            r = (int(random.random() * 10 * len(self.X) * self.train_test_ratio) % int(len(self.X) * self.train_test_ratio))
            x.append(self.X[r])
            y.append(self.Y[r])        
        return np.array(x), np.array(y)
    
    def get_test(self):
        x = []
        y = []
        s = int(len(self.X) * self.train_test_ratio)
        for i in range(len(self.X) - s):
            x.append(self.X[s + i])
            y.append(self.Y[s + i])
        return np.array(x), np.array(y)
    
    def load_model(self, log_file='../log/diabolo_system/mymodel.h5'):
        serializers.load_hdf5(log_file, self.model)
        
    def save_model(self):
        serializers.save_hdf5('../log/diabolo_system/mymodel.h5', self.model)
        
    def train(self, loop_num=1000, plot_loss=True):
        print('[DebugPrint] Training Start')        
        losses =[]
        for i in range(loop_num):
            self.percentage(i, loop_num)
            x, y = self.get_batch_train(self.batch_size)
        
            x_ = Variable(x.astype(np.float32).reshape(self.batch_size, 6))
            t_ = Variable(y.astype(np.float32).reshape(self.batch_size, 2))
        
            self.model.zerograds()
            self.model(x_)
            loss = self.model.loss(t_)

            loss.backward()
            self.optimizer.update()
        
            losses.append(loss.data)
            
        print(losses[-1])
        if plot_loss == True:
            plt.plot(losses)
            plt.yscale('log')
            plt.show()
            
    def test(self):
        x, y = self.get_test()        
        x_ = Variable(x.astype(np.float32).reshape(len(x),6))
        t_ = Variable(y.astype(np.float32).reshape(len(y),2))
        
        self.model.zerograds()
        self.model(x_)
        loss = self.model.loss(t_)
        loss.backward()
        #self.optimizer.update()
        
        with open('../log/diabolo_system/predict.log', 'w') as f:
            for i in range(len(x_)):
                f.write('{} {} {} {} {} {}\n'.format(x_[i][4].data, x_[i][5].data, t_[i][0].data, t_[i][1].data, self.model.res[i][0].data, self.model.res[i][1].data))
        print(loss.data)

    def realtime_feedback(self):
        rospy.Subscriber("calc_idle_diabolo_state/diabolo_state", Float64MultiArray, self.callback_for_state, queue_size=1)
        rospy.spin()        
        # callback_for_state is working background
        #   optimize_input
        #   publish_input

    def callback_for_state(self, msg):
        if msg.data[0] == np.nan:
            msg.data[0] = self.past_states[-1][0]
        if msg.data[1] == np.nan:
            msg.data[1] = self.past_states[-1][1]
        self.past_states.append([msg.data[0], msg.data[1]])
        if len(self.past_states) > self.PAST_STATE_NUM * self.DELTA_STEP:
            self.optimize_input()
            print('{} {} {} {}'.format(self.now_input[0], self.now_input[1], self.past_states[-1][0], self.past_states[-1][1]))
            self.publish_input()                            

    def publish_input(self):
        msg = Float64MultiArray()
        msg.data = [self.now_input[0], self.now_input[1]]
        self.pub_input.publish(msg)

    # CHECK
    def optimize_input(self): # TODO add max input restriction
        x = Variable(np.array([self.past_states[-1 * self.DELTA_STEP], self.past_states[-2 * self.DELTA_STEP], self.now_input]).astype(np.float32).reshape(1,6)) # TODO random value is past state
        t = Variable(np.array(self.state_ref).astype(np.float32).reshape(1,2))
        loop_flag = True
        for i in range(20):    # optimize loop  loop_num is 10 == hz is 90
            self.model(x)
            loss = self.model.loss(t)
            loss.backward()
            
            x = Variable((x - 0.01 * x.grad_var).data)
            now_input = [x[0][4].data, x[0][5].data]
            # apply input restriction
            for j in range(self.PAST_INPUT_NUM * self.INPUT_DIM):
                # diff input restriction
                if now_input[j] - self.now_input[j] > self.MAX_INPUT_DIFF_RESTRICTION[j]:
                    now_input[j] = np.float32(self.now_input[j] + self.MAX_INPUT_DIFF_RESTRICTION[j])
                    #loop_flag = False
                elif self.now_input[j] - now_input[j] > self.MAX_INPUT_DIFF_RESTRICTION[j]:
                    now_input[j] = np.float32(self.now_input[j] - self.MAX_INPUT_DIFF_RESTRICTION[j])
                    #loop_flag = False                    
                # max min input restriction
                if now_input[j] > self.MAX_INPUT_RESTRICTION[j]:
                    now_input[j] = self.MAX_INPUT_RESTRICTION[j]
                    #loop_flag = False                    
                elif now_input[j] < self.MIN_INPUT_RESTRICTION[j]:
                    now_input[j] = self.MIN_INPUT_RESTRICTION[j]
                    #loop_flag = False                    
              
            x = Variable(np.array([self.past_states[-1 * self.DELTA_STEP], self.past_states[-2 * self.DELTA_STEP], now_input]).astype(np.float32).reshape(1,6)) # TODO random value is past state
            if loop_flag == False:
                break

        self.now_input = [float(x[0][self.PAST_STATE_NUM * self.STATE_DIM + 0].data), float(x[0][self.PAST_STATE_NUM * self.STATE_DIM + 1].data)]

    def simulate(self, simulate_loop_num=1000):
        #self.init_state = [40., 0.]
        self.init_state = [40., 0.]        
        
        self.past_states = [self.init_state for i in range(self.PAST_STATE_NUM * self.DELTA_STEP)]
        print('[DebugPrint] Simulation Start')        
        with open('../log/diabolo_system/simulate.log', 'w') as f:
            for i in range(simulate_loop_num):   # simulation loop
                self.percentage(i, simulate_loop_num)
                self.optimize_input()
                now_x = Variable(np.array([self.past_states[-1 * self.DELTA_STEP], self.past_states[-2 * self.DELTA_STEP], self.now_input]).astype(np.float32).reshape(1, 6))
        
                self.model(now_x)
                res = self.model.res
                now_state = [res[0][0].data, res[0][1].data]

                #f.write('{} {} {} {}\n'.format(self.now_input[0], self.now_input[1], now_state[0], now_state[1]))
                f.write('{} {} {} {}\n'.format(self.now_input[0], self.now_input[1], self.past_states[-1][0], self.past_states[-1][1]))                
                self.past_states.append(now_state)

if __name__ == '__main__':
    # init arg parser
    parser = argparse.ArgumentParser()
    parser.add_argument("--train", "-t", help="train NN or not")
    parser.add_argument("--action", "-a", help="what action do you want")
    parser.add_argument("--plot", "-p", help="plot loss graph or not")        
    args = parser.parse_args()

    # parse train
    if args.train == None:
        train_flag = 0
    else:
        train_flag = args.train

    # parse action
    if args.action == None:
        action = 2    # 0:test  1:simulate  2:realtime feedback
    else:
        action = int(args.action)

    # parse plot
    if args.plot == None:
        plot_loss_ = 1
    else:
        plot_loss_ = args.plot
        
    ds = DiaboloSystem()

    # train model or load model
    if train_flag:
        ds.load_data(LOG_FILES)
        ds.arrange_data()
        ds.make_model()
        ds.train(loop_num=1000, plot_loss=plot_loss_)
        ds.save_model()
    else:
        ds.load_data(LOG_FILES)
        ds.arrange_data()
        ds.make_model()    
        ds.load_model(log_file='../log/diabolo_system/mymodel.h5')

    if action == 0: # test
        ds.test()
    elif action == 1: # simulate
        ds.simulate(simulate_loop_num=300)
    elif action == 2: # realtime feedback
        ds.realtime_feedback()
