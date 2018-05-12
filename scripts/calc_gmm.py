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
        self.input_dim = 2
        self.state_dim = 2
        self.past_state_num = 2
        self.num_of_split = 2
        self.input_name = ['arm', 'base']
        
        # subscriber
        rospy.init_node('gmm', anonymous=True)
        rospy.Subscriber("sample_pcl/diabolo/pitch", Float64, self.callbackForPitch)
        rospy.Subscriber("sample_pcl/diabolo/yaw", Float64, self.callbackForYaw)
        self.now_states = [[0, 0], [0, 0]] # FIX

        # publisher
        self.pub_arm = rospy.Publisher('calc_gmm/diabolo/arm', Float64, queue_size=10)
        self.pub_base = rospy.Publisher('calc_gmm/diabolo/base', Float64, queue_size=10)

        
    def callbackForPitch(self, msg):
        self.now_states[0][1] = msg.data - self.now_states[0][0]
        self.now_states[0][0] = msg.data
        
    def callbackForYaw(self, msg):
        self.now_states[1][1] = msg.data - self.now_states[1][0]        
        self.now_states[1][0] = msg.data

        
    def load_data(self):
        inputs = [[] for i in range(self.input_dim)]
        states = [[[] for i in range(self.past_state_num)] for i in range(self.state_dim)]
        min_inputs = [1000 for i in range(self.input_dim)]
        max_inputs = [-1000 for i in range(self.input_dim)]
        
        log_files = ['../log/log-by-logger/log-by-loggerpy0.log',
                     '../log/log-by-logger/log-by-loggerpy1.log',
                     '../log/log-by-logger/log-by-loggerpy2.log',
                     '../log/log-by-logger/log-by-loggerpy3.log',
                     '../log/log-by-logger/log-by-loggerpy4.log']

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

    def getGMMList(self):
        return [[[[[object for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.input_dim)]            
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


    def split_data(self, states, inputs, borders_states):
        inputs_list = [[[[[[] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.input_dim)]

        # split data
        for i in range(state_data_num):
            for j in range(self.input_dim):
                idxs = [[self.num_of_split - 1 for ii in range(self.state_dim)] for ii in range(self.past_state_num)]
                for k in range(self.state_dim):
                    for l in range(self.past_state_num):
                        s = states[k][l][i]
                        for m in range(self.num_of_split - 1): # search border
                            if s < borders_states[k][l][m]:
                                idxs[k][l] = m
                                break
                inputs_list[j][idxs[0][0]][idxs[0][1]][idxs[1][0]][idxs[1][1]].append(inputs[j][i])
                
        return inputs_list

    def fit_aic(self, inputs_list, ip, ip_, iy, iy_, max_gm_num=1): # TODO increase max_gm_num
        inputs_gmm = [[] for i in range(self.input_dim)]
        
        # fit GMM by AIC
        for i in range(self.input_dim):
            COVARIANCE_TYPES = ['spherical', 'tied', 'diag', 'full']
            args = list(itertools.product(COVARIANCE_TYPES, range(1, max_gm_num + 1)))
            models = np.zeros(len(args), dtype=object)

            X_raw = np.array([[j] for j in inputs_list[i][ip][ip_][iy][iy_]])
            if len(X_raw) < 2:
                inputs_gmm_list[i][ip][ip_][iy][iy_] = None
                continue
            for j, (ctype, n) in enumerate(args):
                models[j] = GMM(n, covariance_type=ctype, tol=1e-10, min_covar=1e-10)
                models[j].fit(X_raw)
            aic = np.array([m.aic(X_raw) for m in models])
            print i, ip, ip_, iy, iy_
            inputs_gmm[i] = models[np.argmin(aic)] # TODO not substitude, but append
            
        return inputs_gmm
    
    def plot_gmm(self, inputs_list, inputs_gmm_list, min_inpus, max_inputs, ip, ip_, iy, iy_):
            fig = plt.figure()
            ax = [fig.add_subplot(211), fig.add_subplot(212)]

            for i in range(self.input_dim):
                ax[i].set_title(self.input_name[i])
                ax[i].set_xlim(min_inputs[i], max_inputs[i])
                ax[i].hist(np.array(inputs_list[i][ip][ip_][iy][iy_]), normed=True)
                
                if inputs_gmm_list[i][ip][ip_][iy][iy_] == None:
                    continue
                mean = inputs_gmm_list[i][ip][ip_][iy][iy_].means_
                cov = inputs_gmm_list[i][ip][ip_][iy][iy_]._get_covars()
                #print mean, cov
                X_pred = np.linspace(min_inputs[i], max_inputs[i], 1000)
                Y_pred = self.gauss(X_pred, float(mean[0]), float(cov[0][0])) # TODO cov
                ax[i].plot(X_pred, Y_pred)
                
            plt.show()
            

    def save_model(self, inputs_gmm_list, log_name='../log/gmm/gmm.log'):
        with open(log_name, 'w') as f:
            # write meta data
            f.write(str(self.input_dim) + ' '
                    + str(self.state_dim) + ' '
                    + str(self.past_state_num) + ' ')
            for i in range(self.input_dim):
                f.write(str(self.input_name[i]) + ' ')

            # write content
            
    def load_model(self, log_name='../log/gmm/gmm.log'):
        pass

    #
    # belowe is for using real interface
    #
    
    def realtime_act(self, inputs_gmm_list, borders_states):
        while True:
            [[ip, ip_], [iy, iy_]] = self.calc_states_idx(borders_states)
            self.update_gmm(ip, ip_, iy, iy_)
            next_inputs = self.calc_next_inputs(inputs_gmm_list, ip, ip_, iy, iy_)
            self.publish(next_inputs)
            print ip, ip_, iy, iy_
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

    def update_gmm(self, ip, ip_, iy, iy_):
        pass
    
    def calc_next_inputs(self, inputs_gmm_list, ip, ip_, iy, iy_):
        next_inputs = [0, 0] # FIX
        
        for i in range(self.input_dim):
            if inputs_gmm_list[i][ip][ip_][iy][iy_] == None:
                continue
            print i, ip, ip_, iy, iy_
            mean = inputs_gmm_list[i][ip][ip_][iy][iy_].means_
            cov = inputs_gmm_list[i][ip][ip_][iy][iy_]._get_covars()
            next_inputs[i] = np.random.normal(mean, cov, 1)

        return next_inputs
    
    def publish(self, next_inputs):
        print next_inputs[0], next_inputs[1]
        self.pub_arm.publish(next_inputs[0])
        self.pub_base.publish(next_inputs[1])        

    @staticmethod
    def gauss(X, mean, cov):
        Y = []
        for x in X:
            a = 1. / ((2. * math.pi * cov)**(0.5))
            b = -0.5 * (x - mean)**2 / cov
            Y.append(a * math.exp(b))
        return np.array(Y)
    
if __name__ == '__main__':
    signal.signal(signal.SIGINT, lambda signal, frame: sys.exit(0))
    
    calc_gmm = CalcGMM()
    load_data_flag = True
    plot_flag = False
    load_model_flag = False
    juggle_flag = True

    # load data from log file, fit and save
    if load_data_flag:
        inputs_gmm_list = calc_gmm.getGMMList() #[[[[[object for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.input_dim)]        
        states, inputs, state_data_num, min_inputs, max_inputs= calc_gmm.load_data()
        borders_states = calc_gmm.calc_borders(states, state_data_num)
        inputs_list = calc_gmm.split_data(states, inputs, borders_states)

        for ip in range(calc_gmm.num_of_split):
            for ip_ in range(calc_gmm.num_of_split):
                for iy in range(calc_gmm.num_of_split):
                    for iy_ in range(calc_gmm.num_of_split):
                        print 'p:' + str(ip) + ' p_:' + str(ip_) + ' y:' + str(iy) + ' y_:' + str(iy_)
                        inputs_gmm = calc_gmm.fit_aic(inputs_list, ip, ip_, iy, iy_)
                        for i in range(len(inputs_list)):
                            inputs_gmm_list[i][ip][ip_][iy][iy_] = inputs_gmm[i]
                        if plot_flag:
                            calc_gmm.plot_gmm(inputs_list, inputs_gmm_list, min_inputs, max_inputs, ip, ip_, iy, iy_)
                        
        #calc_gmm.save_model(inputs_gmm_list, '../log/gmm/gmm.log')

    # load model
    if load_model_flag:
        inputs_gmm_list = calc_gmm.load_model('../log/gmm/gmm.log')

    # subscribe states and publish inputs
    if juggle_flag:
        calc_gmm.realtime_act(inputs_gmm_list, borders_states)

