#!/usr/bin/env python
# -*- coding: utf-8 -*-

# TODO

# kerasの例はランダムにプロットしているので近く見える
#   でも近い値は結構あるので確認のため隣合わせの例でプロットさせる
#      ぎりぎり許せるくらい...
# chainerは数値微分？reluはどうやって登録？

import random
import numpy as np

import chainer
from chainer import Variable, Link, Chain, ChainList, optimizers, serializers
import chainer.functions as F
import chainer.links as L
from chainer.functions.loss.mean_squared_error import mean_squared_error

from matplotlib import pyplot as plt

import rospy
from std_msgs.msg import Float64MultiArray, Float64

LOG_FILES = ['../log/log-by-logger/log-by-loggerpy0.log',
             '../log/log-by-logger/log-by-loggerpy1.log',
             '../log/log-by-logger/log-by-loggerpy2.log',
             '../log/log-by-logger/log-by-loggerpy3.log',
             '../log/log-by-logger/log-by-loggerpy4.log',
             '../log/log-by-logger/log-by-loggerpy5.log',
             '../log/log-by-logger/log-by-loggerpy6.log']             

class MyChain(Chain):
    def __init__(self):
        super(MyChain, self).__init__(   # FIX
            l1=L.Linear(6, 10),
            l2=L.Linear(2)
            )
        
    def forward(self, x):   # FIX
        h = F.relu(self.l1(x))
        o = self.l2(h)
        return o

    def __call__(self, x):
        self.res = self.forward(x)

    def loss(self, t):
        return F.mean_squared_error(self.res, t)

class DiaboloSystem():
    def __init__(self):
        self.input_arm = []
        self.input_base = []
        self.state_pitch = []
        self.state_yaw = []

        self.PAST_STATE_NUM = 2   # FIX
        self.PAST_INPUT_NUM = 1   # FIX
        self.PAST_NUM = max(self.PAST_STATE_NUM, self.PAST_INPUT_NUM)
        
        self.STATE_DIM = 2
        self.INPUT_DIM = 2

        self.DELTA_STEP = 5   # FIX

        self.INPUT_NN_DIM = self.STATE_DIM * self.PAST_STATE_NUM + self.INPUT_DIM * self.PAST_INPUT_NUM
        self.OUTPUT_NN_DIM = self.STATE_DIM
        
        self.VAR_NUM = max(self.INPUT_NN_DIM, self.OUTPUT_NN_DIM)

        self.state_ref = [0., 0.]   # FIX

        self.train_test_ratio = 0.8   # FIX
        self.batch_size = 1000   # FIX

        rospy.init_node("DiaboloSystem")
        rospy.Subscriber("calc_diabolo_state/diabolo_state", Float64MultiArray, self.callback_for_state)
        self.pub_arm = rospy.Publisher('/diabolo_system/arm', Float64, queue_size=1)
        self.pub_base = rospy.Publisher('/diabolo_system/base', Float64, queue_size=1)

    def load_data(self, log_files):
        for log_file in log_files:
            with open(log_file, "r") as f:
                for l in f.readlines():
                    val = l.split(' ')
                    if len(val) != 4:
                        break
                
                    self.input_arm.append(float(val[0]))
                    self.input_base.append(float(val[1]))
                    self.state_pitch.append(float(val[2]))
                    self.state_yaw.append(float(val[3][:-1]))

    def arrange_data(self):
        self.X = []
        self.Y = []
        
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
        
    def train(self, loop=1000, plot_loss=True):
        losses =[]
        for i in range(loop):
            print i
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
            
    def save_model(self):
        serializers.save_hdf5('../log/diabolo_system/mymodel.h5', self.model)

    def load_model(self):
        serializers.load_hdf5('../log/diabolo_system/mymodel.h5', self.model)
        
    def calc_next_state(self):
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

    def realtime(self):
        pub_arm.publish()
        pub_base.publish()
        pass

    def callback_for_state(self):
        pass

    def optimize_input(self):
        x = Variable(np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5]).astype(np.float32).reshape(1,6)) # TODO random value is past state
        t = Variable(np.array(self.state_ref).astype(np.float32).reshape(1,2))
        losses =[]        
        for i in range(1000):
            print i
            self.model(x)
            loss = self.model.loss(t)
            loss.backward()
            
            print x, x.grad_var
            x = Variable((x - 0.01 * x.grad_var).data)
            
            losses.append(loss.data)    

    def simulate(self):
        init_state = [0, 0]
        past_states = [init_state for i in range(10)]
        now_input = [0.85, 0]   #[0.75, 0]        

        with open('../log/diabolo_system/simulate.log', 'w') as f:
            for i in range(100):
                now_x = Variable(np.array([past_states[-1 * self.DELTA_STEP], past_states[-2 * self.DELTA_STEP], now_input]).astype(np.float32).reshape(1, 6))
        
                self.model(now_x)
                res = self.model.res
                now_state = [res[0][0].data, res[0][1].data]

                f.write('{} {} {} {}\n'.format(now_input[0], now_input[1], now_state[0], now_state[1]))
                past_states.append(now_state)

            

if __name__ == '__main__':
    train_flag = False
    
    ds = DiaboloSystem()
    
    if train_flag:
        ds.load_data(LOG_FILES)
        ds.arrange_data()
        ds.make_model()
        ds.train(1000)
        ds.save_model()
    else:
        ds.load_data(LOG_FILES)
        ds.arrange_data()
        ds.make_model()    
        ds.load_model()
        
    ds.calc_next_state()
    ds.simulate()
    #ds.optimize_input()        

exit()





if losses[-1] > 0.01:
    exit()
    
losses_ =[]
xx = np.array([0.5, 0.5]).astype(np.float32).reshape(1,2)
tt = np.array([0.8]).astype(np.float32).reshape(1,1)
xxx = Variable(xx)
ttt = Variable(tt)
for i in range(1000):
    print i
    
    loss = model(xxx, ttt)
    loss.backward()
    
    print xxx, xxx.grad_var
    xxx = Variable((xxx - 0.01 * xxx.grad_var).data)
    
    losses_.append(loss.data)    

print model.predict(xxx)
print model.predict(Variable(np.array([0., 0.]).astype(np.float32).reshape(1,2)))
print model.predict(Variable(np.array([0., 1.]).astype(np.float32).reshape(1,2)))
print model.predict(Variable(np.array([1., 0.]).astype(np.float32).reshape(1,2)))
print model.predict(Variable(np.array([1., 1.]).astype(np.float32).reshape(1,2)))

with open('po.log', 'w') as f:
    for i in range(-10, 10):
        for j in range(-10, 10):
            x = Variable(np.array([i / 10., j / 10.]).astype(np.float32).reshape(1,2))
            p = model.predict(x).data[0][0]
            f.write('{} {} {}\n'.format(i / 10., j / 10., p))
plt.plot(losses_)
plt.yscale('log')
plt.show()

    
