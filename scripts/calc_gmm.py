# -*- coding: utf-8 -*-                
import rospy
from std_msgs.msg import Float64

import matplotlib.pyplot as plt
import numpy as np
from sklearn.mixture import GMM
import math
import itertools

import signal, sys
import time


class CalcGMM:
    def __init__(self):
        # init metadata
        self.input_dim = 2
        self.state_dim = 2
        self.past_state_num = 2
        self.num_of_split = 3
        self.input_name = ['arm', 'base']

        # parameters for reward function
        self.reward_past_num = 3
        self.idle = 0
        
        # Subscriber
        rospy.init_node('gmm', anonymous=True)
        rospy.Subscriber("sample_pcl/diabolo/pitch", Float64, self.callbackForPitch)
        rospy.Subscriber("sample_pcl/diabolo/yaw", Float64, self.callbackForYaw)
        rospy.Subscriber("idle", Float64, self.callbackForIdle)        
        self.now_states = [[0, 0], [0, 0]]

        # Publisher
        self.pub_arm = rospy.Publisher('calc_gmm/diabolo/arm', Float64, queue_size=10)
        self.pub_base = rospy.Publisher('calc_gmm/diabolo/base', Float64, queue_size=10)

    # Callback function of Subscriber for pitch
    def callbackForPitch(self, msg):
        if msg.data == np.nan:
            msg.data = 0
        self.now_states[0][1] = msg.data - self.now_states[0][0]
        self.now_states[0][0] = msg.data

    # Callback function of Subscriber for yaw
    def callbackForYaw(self, msg):
        if msg.data == np.nan:
            msg.data = 0
        self.now_states[1][1] = msg.data - self.now_states[1][0]        
        self.now_states[1][0] = msg.data
        
    def callbackForIdle(self, msg):
        if msg.data == 1:
            self.idle = 1
            print 'this is 1'
        else:
            self.idle = 0

    def load_data(self, log_files):
        inputs = [[] for i in range(self.input_dim)]
        states = [[[] for i in range(self.past_state_num)] for i in range(self.state_dim)]
        min_inputs = [1000 for i in range(self.input_dim)]
        max_inputs = [-1000 for i in range(self.input_dim)]
        
        state_data_num = 0
        for log_file in log_files:
          pitch = 0
          yaw = 0
          cnt = 0
          with open(log_file) as f:
            for l in f.readlines():
                w = l.split(' ')
                if len(w) != 4:
                    continue
                
                past_pitch = pitch
                past_yaw = yaw
                
                arm = float(w[0])
                base = float(w[1])
                pitch = float(w[2])
                yaw = float(w[3])
                pitch_ = pitch - past_pitch
                yaw_ = yaw - past_yaw
        
                cnt += 1
                if cnt < 1:
                    continue

                # store pitch, pitch_, yaw, yaw_, arm, base
                states[0][0].append(pitch)
                states[0][1].append(pitch_)
                states[1][0].append(yaw)
                states[1][1].append(yaw_)
                inputs[0].append(arm)
                inputs[1].append(base)
                state_data_num += 1

                # calc min, max of arm and base
                if min_inputs[0] > arm:
                    min_inputs[0] = arm
                if max_inputs[0] < arm:
                    max_inputs[0] = arm
                if min_inputs[1] > base:
                    min_inputs[1] = base
                if max_inputs[1] < base:
                    max_inputs[1] = base
                    
        return states, inputs, state_data_num, min_inputs, max_inputs


    # calc borders of states map
    def calc_borders(self, states, state_data_num):
        sorted_states = [[[] for i in range(self.state_dim)] for i in range(self.past_state_num)]
        borders_states = [[[] for i in range(self.state_dim)] for i in range(self.past_state_num)]
        
        # calc borders        
        for i in range(self.state_dim):
            for j in range(self.past_state_num):
                sorted_states[i][j] = sorted(states[i][j])
        for i in range(self.num_of_split - 1):
            for j in range(self.state_dim):
                for k in range(self.past_state_num):
                    borders_states[j][k].append(sorted_states[j][k][int(1.0 * (i + 1) / self.num_of_split * state_data_num)])
                    
        return borders_states

    def split_data(self, states, inputs, state_data_num, borders_states):
        inputs_list = [[[[[[] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.input_dim)]
        rewards_xy_list = [[[[[[] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.input_dim)]
        
        # split data
        for i in range(state_data_num):
            for j in range(self.input_dim):
                idxs = [[self.num_of_split - 1 for ii in range(self.state_dim)] for ii in range(self.past_state_num)]
                r = 0
                for k in range(self.state_dim):
                    for l in range(self.past_state_num):
                        s = states[k][l][i]
                        if i < len(states[k][l]) - 3:
                            #r += abs(states[k][l][i]) - abs(states[k][l][i + 1]) # before - after
                            r += abs(states[k][l][i]) - abs(states[k][l][i + self.reward_past_num]) # before - after                             
                        for m in range(self.num_of_split - 1): # search border
                            if s < borders_states[k][l][m]:
                                idxs[k][l] = m
                                break
                inputs_list[j][idxs[0][0]][idxs[0][1]][idxs[1][0]][idxs[1][1]].append(inputs[j][i])
                if r < 0:
                    r = 0
                rewards_xy_list[j][idxs[0][0]][idxs[0][1]][idxs[1][0]][idxs[1][1]].append([inputs[j][i], r])                
                
        return inputs_list, rewards_xy_list

    def fit_aic(self, inputs_list, max_gm_num=1): # TODO increase max_gm_num
        inputs_gmm_list = [[[[[[[]] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.input_dim)]
        COVARIANCE_TYPES = ['spherical', 'tied', 'diag', 'full']
        
        # fit GMM by AIC
        for i in range(self.input_dim):
            for ip in range(calc_gmm.num_of_split):
                for ip_ in range(calc_gmm.num_of_split):
                    for iy in range(calc_gmm.num_of_split):
                        for iy_ in range(calc_gmm.num_of_split):
                            print 'input: ' + str(i) + ' p:' + str(ip) + ' p_:' + str(ip_) + ' y:' + str(iy) + ' y_:' + str(iy_)
        
                            args = list(itertools.product(COVARIANCE_TYPES, range(1, max_gm_num + 1)))
                            models = np.zeros(len(args), dtype=object)
                
                            X_raw = np.array([[ii] for ii in inputs_list[i][ip][ip_][iy][iy_]])
                            if len(X_raw) < 2:
                                inputs_gmm_list[i][ip][ip_][iy][iy_][0] = None # TODO
                                continue
                            for j, (ctype, n) in enumerate(args):
                                models[j] = GMM(n, covariance_type=ctype, tol=1e-10, min_covar=1e-10)
                                models[j].fit(X_raw)
                            aic = np.array([m.aic(X_raw) for m in models])

                            m = models[np.argmin(aic)] # TODO not substitude, but append                            
                            inputs_gmm_list[i][ip][ip_][iy][iy_][0] = [float(m.means_[0]), float(m._get_covars()[0][0])]

        return inputs_gmm_list
    
    def calc_reward(self, inputs_gmm_list, rewards_xy_list):
        rewards_list = [[[[[0 for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.input_dim)]
        for i in range(self.input_dim):
            for ip in range(self.num_of_split):
                for ip_ in range(self.num_of_split):
                    for iy in range(self.num_of_split):
                        for iy_ in range(self.num_of_split):
                            if inputs_gmm_list[i][ip][ip_][iy][iy_][0] == None:
                                continue
                            mean = inputs_gmm_list[i][ip][ip_][iy][iy_][0][0]
                            cov = inputs_gmm_list[i][ip][ip_][iy][iy_][0][1]
                            for reward in rewards_xy_list[i][ip][ip_][iy][iy_]:
                                x = reward[0]
                                r = reward[1]
                                p = self.gauss([x], mean, cov) # TODO how to calc prob of GMM
                                rewards_list[i][ip][ip_][iy][iy_] += p[0] * r
        return rewards_list
    
    def plot_gmm(self, inputs_lists, inputs_gmm_lists, rewards_xy_lists, min_inputs, max_inputs, gmm_only=False):
        print 'plot'
        if gmm_only: # gmm_only
          for ip in range(self.num_of_split):
            for ip_ in range(self.num_of_split):
                for iy in range(self.num_of_split):
                    for iy_ in range(self.num_of_split):
                        # init plot
                        fig = plt.figure()
                        ax = []
                        for i in range(self.input_dim):
                            ax.append(fig.add_subplot(len(inputs_lists), 1, i + 1))
                        for i in range(len(inputs_lists)):                            
                            for j in range(self.input_dim):
                                ax[j].set_title(self.input_name[j])

                                #ax[i].set_xlim(min_inputs[j][0], max_inputs[j][0])                                
                                if inputs_gmm_lists[i][j][ip][ip_][iy][iy_][0] == None:
                                    continue
                                # plot GMM
                                mean = float(inputs_gmm_lists[i][j][ip][ip_][iy][iy_][0][0])
                                cov = float(inputs_gmm_lists[i][j][ip][ip_][iy][iy_][0][1])
                                X_pred = np.linspace(min_inputs[i][j], max_inputs[i][j], 1000)
                                Y_pred = self.gauss(X_pred, float(mean), float(cov)) # TODO cov
                                ax[j].plot(X_pred, Y_pred)
                                print min(Y_pred), max(Y_pred)
                                print min(X_pred), max(X_pred)
                                print 'input: ' + str(i) + ' p:' + str(ip) + ' p_:' + str(ip_) + ' y:' + str(iy) + ' y_:' + str(iy_)                                
                        plt.show()
        else: # not gmm_only
          for ip in range(self.num_of_split):
            for ip_ in range(self.num_of_split):
                for iy in range(self.num_of_split):
                    for iy_ in range(self.num_of_split):
                        # init plot
                        fig = plt.figure()
                        ax = []
                        for i in range(self.input_dim):
                            for j in range(len(inputs_lists)):
                                ax.append(fig.add_subplot(len(inputs_lists), 2, i + 2 * j + 1))
                
                        for i in range(len(inputs_lists)):
                            for j in range(self.input_dim):
                                # plot histgram
                                ax[i * 2 + j].set_title(self.input_name[j])
                                ax[i * 2 + j].set_xlim(min_inputs[i][j], max_inputs[i][j])
                                ax[i * 2 + j].hist(np.array(inputs_lists[i][j][ip][ip_][iy][iy_]), normed=True)
                                
                                if inputs_gmm_lists[i][j][ip][ip_][iy][iy_][0] == None:
                                    continue

                                # plot GMM
                                mean = float(inputs_gmm_lists[i][j][ip][ip_][iy][iy_][0][0])
                                cov = float(inputs_gmm_lists[i][j][ip][ip_][iy][iy_][0][1])
                                #print mean, cov
                                X_pred = np.linspace(min_inputs[i][j], max_inputs[i][j], 1000)
                                Y_pred = self.gauss(X_pred, float(mean), float(cov)) # TODO cov
                                ax[i * 2 + j].plot(X_pred, Y_pred)

                                # plot zero line
                                ax[i * 2 + j].plot(np.array([min_inputs[i][j], max_inputs[i][j]]), np.array([0, 0]))               
                                
                                # plot reward
                                if rewards_xy_lists[i] == False:
                                    continue
                                reward_x = [min_inputs[i][j] + (max_inputs[i][j] - min_inputs[i][j]) * ii / 20.0 for ii in range(20)]
                                reward_y = [0 for ii in range(20)]
                                cnt = [0 for ii in range(20)]
                                for reward_xy in rewards_xy_lists[i][j][ip][ip_][iy][iy_]:
                                    for k in range(20):
                                        x = min_inputs[i][j] + (max_inputs[i][j] - min_inputs[i][j]) * k / 20.0
                                        if reward_xy[0] < x:
                                            reward_y[k] += reward_xy[1]
                                            cnt[k] += 1
                                            break
                                for k in range(20):
                                    if cnt[k] == 0:
                                        continue
                                    reward_y[k] = 1.0 * reward_y[k] / cnt[k]
                                ax[i * 2 + j].plot(np.array(reward_x), np.array(reward_y))
                    
                
                        print 'input: ' + str(j) + ' p:' + str(ip) + ' p_:' + str(ip_) + ' y:' + str(iy) + ' y_:' + str(iy_)
                        plt.show()                        

    def save_model(self, inputs_gmm_list, inputs, states, min_inputs, max_inputs, log_name='../log/gmm/gmm.log'):
        with open(log_name, 'w') as f:
            # write meta data
            f.write(str(self.input_dim) + ' '
                    + str(self.state_dim) + ' '
                    + str(self.past_state_num) + ' ')
            for i in range(self.input_dim):
                f.write(str(self.input_name[i]) + ' ')
            for i in range(self.input_dim):
                f.write(str(min_inputs[i]) + ' ' + str(max_inputs[i]) + ' ')
            f.write('\n')

            # write GMM
            for i in range(self.input_dim):
                for ip in range(self.num_of_split):
                    for ip_ in range(self.num_of_split):
                        for iy in range(self.num_of_split):
                            for iy_ in range(self.num_of_split):
                                mean = float(inputs_gmm_list[i][ip][ip_][iy][iy_][0][0])
                                cov = float(inputs_gmm_list[i][ip][ip_][iy][iy_][0][1])
                                f.write(str(float(mean)) + ' ' + str(float(cov)) + '\n')
            # write raw data
            for i in range(len(inputs[0])):
                f.write(str(float(inputs[0][i])) + ' ' + str(float(inputs[1][i])) + ' ' + str(float(states[0][0][i])) + ' ' + str(float(states[0][1][i])) + ' ' + str(float(states[1][0][i])) + ' ' + str(float(states[1][1][i])) + '\n')


    def load_model(self, log_name='../log/gmm/gmm.log'):
        inputs = [[] for i in range(self.input_dim)]
        states = [[[] for i in range(self.past_state_num)] for i in range(self.state_dim)]
                        
        with open(log_name, 'r') as f:
            min_inputs = [0 for i in range(2)]
            max_inputs = [0 for i in range(2)]
            # read meta data
            l = f.readline()
            w = l.split(' ')
            self.input_dim = int(w[0])
            self.state_dim = int(w[1])
            self.past_state_num = int(w[2])
            self.input_name[0] = w[3]
            self.input_name[1] = w[4]
            min_inputs[0] = float(w[5])
            max_inputs[0] = float(w[6])
            min_inputs[1] = float(w[7])
            max_inputs[1] = float(w[8])

            # read GMM
            inputs_gmm_list = [[[[[[[]] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.input_dim)]
            for i in range(self.input_dim):
                for ip in range(self.num_of_split):
                    for ip_ in range(self.num_of_split):
                        for iy in range(self.num_of_split):
                            for iy_ in range(self.num_of_split):
                                l = f.readline()
                                w = l.split(' ')
                                mean = float(w[0])
                                cov = float(w[1])
                                inputs_gmm_list[i][ip][ip_][iy][iy_][0] = [mean, cov]
            state_data_num = 0
            for l in f.readlines():
                w = l.split(' ')
                inputs[0].append(float(w[0]))
                inputs[1].append(float(w[1]))
                states[0][0].append(float(w[2]))
                states[0][1].append(float(w[3]))
                states[1][0].append(float(w[4]))
                states[1][1].append(float(w[5]))
                state_data_num += 1

            return states, inputs, state_data_num, inputs_gmm_list, min_inputs, max_inputs

    #
    # below is for using real interface
    #
    
    def realtime_act(self, inputs_gmm_list, inputs, states, borders_states, reward_list, min_inputs, max_inputs):
        ip = 0
        ip_ = 0
        iy = 0
        iy_ = 0
        past_idx = [[[0, 0], [0, 0]] for i in range(self.reward_past_num)]
        past_states = [[0, 0] for i in range(self.reward_past_num)]
        past_inputs = [[0, 0] for i in range(self.reward_past_num)]
        while True:
            if self.idle == 0:
                continue
            # calc past_idx
            for i in range(self.reward_past_num - 1):
                past_idx[i] = past_idx[i + 1]
            past_idx[self.reward_past_num - 1] = [[ip, ip_], [iy, iy_]]

            # calc state
            for i in range(self.reward_past_num - 1):
                past_states[i] = past_states[i + 1]
            past_states[self.reward_past_num - 1] = [self.now_states[0][0], self.now_states[1][0]]

            # calc next_inputs
            [[ip, ip_], [iy, iy_]] = self.calc_states_idx(borders_states)
            next_inputs = self.calc_next_inputs(inputs_gmm_list, ip, ip_, iy, iy_)

            states[0][0].append(self.now_states[0][0])
            states[0][1].append(self.now_states[0][1])
            states[1][0].append(self.now_states[1][0])
            states[1][1].append(self.now_states[1][1])
            
            inputs[0].append(next_inputs[0])
            inputs[1].append(next_inputs[1])

            
            # TODO isnt it necessary to append state and input
            
            # calc input
            for i in range(self.reward_past_num - 1):
                past_inputs[i] = past_inputs[i + 1]
            past_inputs[self.reward_past_num - 1] = [next_inputs[0], next_inputs[1]]

            inputs_gmm_list = self.update_gmm(inputs_gmm_list, past_states[0], reward_list, past_inputs[0], past_idx[0][0][0], past_idx[0][0][1], past_idx[0][1][0], past_idx[0][1][1])
            self.publish(next_inputs)
            self.save_model(inputs_gmm_list, inputs, states, min_inputs, max_inputs, '../log/gmm/gmm.log')
            time.sleep(0.01)
            
    def calc_states_idx(self, borders_states):
        states_idx = [[self.num_of_split - 1 for ii in range(self.state_dim)] for ii in range(self.past_state_num)]
        for i in range(self.state_dim):
            for j in range(self.past_state_num):
                s = self.now_states[i][j]
                for k in range(self.num_of_split - 1): # search border
                    if s < borders_states[i][j][k]:
                        states_idx[i][j] = k
                        break
        return states_idx

    def update_gmm(self, inputs_gmm_list, past_state, reward_list, past_input, ip, ip_, iy, iy_):
        BIG_NUM = 100
        # calc reward
        reward = 0
        for i in range(self.input_dim):
            reward += abs(past_state[i]) - abs(self.now_states[i][0])
        if reward == np.nan or reward > BIG_NUM:
            print '[WARN] reward is ', reward
            reward = BIG_NUM
        if reward <= 0:
            print '[WARN] reward is ', reward
            return inputs_gmm_list
            
        for i in range(self.input_dim):
            gmm_reward = reward_list[i][ip][ip_][iy][iy_]
            #if gmm_reward <= 0:
            #    print '[WARN] gmm_reward is below zero'
            if gmm_reward == np.nan or  gmm_reward > BIG_NUM:
                print '[WARN] gmm_reward is ', gmm_reward
                reward_list[i][ip][ip_][iy][iy_] = BIG_NUM
                gmm_reward = BIG_NUM
            if  gmm_reward <= 0:
                print '[WARN] gmm_reward is ', gmm_reward
                print '[Error] This is important Error'
                return inputs_gmm_list
            if gmm_reward > 0 and gmm_reward <= BIG_NUM:
                print 'OK'
            else: # TODO this is bad case
                gmm_reward = BIG_NUM

            reward_2 = reward# * reward
            gmm_reward_2 = gmm_reward# * gmm_reward

            print 'reward ', reward_2, gmm_reward_2
            mean = float(inputs_gmm_list[i][ip][ip_][iy][iy_][0][0])
            cov = float(inputs_gmm_list[i][ip][ip_][iy][iy_][0][1])
            new_cov = 1.0 * reward_2 * gmm_reward_2 / (reward_2 + gmm_reward_2);
            new_mean = 1.0 * (mean * reward_2 + past_input[i] * gmm_reward_2) / (reward_2 + gmm_reward_2);

            if new_cov > BIG_NUM or new_cov == np.nan:
                print '[WARN] new_cov is ', new_cov                
                new_cov = BIG_NUM
            if new_cov <= 0:
                print '[WARN] new_cov is ', new_cov                       
                new_cov = 1

            inputs_gmm_list[i][ip][ip_][iy][iy_][0] = [new_mean, new_cov]
        return inputs_gmm_list
    
    def calc_next_inputs(self, inputs_gmm_list, ip, ip_, iy, iy_):
        next_inputs = [0, 0]

        for i in range(self.input_dim):
            if inputs_gmm_list[i][ip][ip_][iy][iy_][0] == None:
                continue
            print 'input: ' + str(i) + ' p:' + str(ip) + ' p_:' + str(ip_) + ' y:' + str(iy) + ' y_:' + str(iy_)            
            mean = inputs_gmm_list[i][ip][ip_][iy][iy_][0][0]
            cov = inputs_gmm_list[i][ip][ip_][iy][iy_][0][1]
            if cov <= 0 or cov == np.nan:
                continue
            next_inputs[i] = np.random.normal(mean, cov, 1)[0]

        return next_inputs
    
    def publish(self, next_inputs):
        print 'next inputs ', next_inputs[0], next_inputs[1] # TODO これが[hoge] [hoge]ではなく0 0のときがある
        self.pub_arm.publish(next_inputs[0])
        self.pub_base.publish(next_inputs[1])        

    @staticmethod
    def gauss(X, mean, cov):
        Y = []
        for x in X:
            a = 1.0 / ((2.0 * math.pi * cov)**(0.5))
            b = -0.5 * (x - mean)**2 / cov
            Y.append(a * math.exp(b))
        return np.array(Y)
    
if __name__ == '__main__':
    signal.signal(signal.SIGINT, lambda signal, frame: sys.exit(0))
    
    calc_gmm = CalcGMM()
    flag = 0
    
    # load data & load model and compare
    if flag == 0:
        # load data & calc GMM
        states, inputs, state_data_num, min_inputs, max_inputs= calc_gmm.load_data(['../log/log-by-logger/log-by-loggerpy0.log',
                                                                                    '../log/log-by-logger/log-by-loggerpy1.log',
                                                                                    '../log/log-by-logger/log-by-loggerpy2.log',
                                                                                    '../log/log-by-logger/log-by-loggerpy3.log',
                                                                                    '../log/log-by-logger/log-by-loggerpy4.log'])
        borders_states = calc_gmm.calc_borders(states, state_data_num)
        inputs_list, rewards_xy_list = calc_gmm.split_data(states, inputs, state_data_num, borders_states)

        # fit
        inputs_gmm_list = calc_gmm.fit_aic(inputs_list)
        reward_list = calc_gmm.calc_reward(inputs_gmm_list, rewards_xy_list)

        # load model & calc GMM
        states_load, inputs_load, state_data_num_load, inputs_gmm_list_load, min_inputs_load, max_inputs_load = calc_gmm.load_model('../log/gmm/gmm_online_training0.log')
        borders_states_load = calc_gmm.calc_borders(states_load, state_data_num_load)                
        inputs_list_load, rewards_xy_list_load = calc_gmm.split_data(states_load, inputs_load, state_data_num_load, borders_states_load)

        
        #calc_gmm.plot_gmm([inputs_list], [inputs_gmm_list], [rewards_xy_list], [min_inputs for i in range(1)], [max_inputs for i in range(1)])
        

        #calc_gmm.plot_gmm([inputs_list, inputs_list_load], [inputs_gmm_list, inputs_gmm_list_load], [rewards_xy_list, False], [min_inputs for i in range(2)], [max_inputs for i in range(2)])
        calc_gmm.plot_gmm([inputs_list, inputs_list_load], [inputs_gmm_list, inputs_gmm_list_load], [rewards_xy_list, False], [min_inputs for i in range(2)], [max_inputs for i in range(2)], gmm_only=True)
        

    # load data & subscribe states and publish inputs
    elif flag == 1:
        # load GMM & calc GMM
        states, inputs, state_data_num, min_inputs, max_inputs= calc_gmm.load_data(['../log/log-by-logger/log-by-loggerpy0.log',
                                                                                    '../log/log-by-logger/log-by-loggerpy1.log',
                                                                                    '../log/log-by-logger/log-by-loggerpy2.log',
                                                                                    '../log/log-by-logger/log-by-loggerpy3.log',
                                                                                    '../log/log-by-logger/log-by-loggerpy4.log'])
        borders_states = calc_gmm.calc_borders(states, state_data_num)
        inputs_list, rewards_xy_list = calc_gmm.split_data(states, inputs, state_data_num, borders_states)

        # fit
        inputs_gmm_list = calc_gmm.fit_aic(inputs_list)

        # calc reward list
        reward_list = calc_gmm.calc_reward(inputs_gmm_list, rewards_xy_list)

        # realtime act
        calc_gmm.realtime_act(inputs_gmm_list, inputs, states, borders_states, reward_list, min_inputs, max_inputs)
        
    # load model & subscribe states and publish inputs
    elif flag == 2:
        # load model & calc GMM
        #states_load, inputs_load, state_data_num_load, inputs_gmm_list_load, min_inputs_load, max_inputs_load = calc_gmm.load_model('../log/gmm/gmm_online_training0.log') 
        states_load, inputs_load, state_data_num_load, inputs_gmm_list_load, min_inputs_load, max_inputs_load = calc_gmm.load_model('../log/gmm/gmm_teach.log')       
        borders_states_load = calc_gmm.calc_borders(states_load, state_data_num_load)                
        inputs_list_load, rewards_xy_list_load = calc_gmm.split_data(states_load, inputs_load, state_data_num_load, borders_states_load)

        # calc reward list
        reward_list_load = calc_gmm.calc_reward(inputs_gmm_list_load, rewards_xy_list_load)

        # realtime act
        calc_gmm.realtime_act(inputs_gmm_list_load, inputs_load, states_load, borders_states_load, reward_list_load, min_inputs_load, max_inputs_load)
        
