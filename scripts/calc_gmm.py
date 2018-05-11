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
        self.num_of_split = 3
        self.input_name = ['arm', 'base']
        
        self.inputs = [[] for i in range(self.input_dim)]
        self.states = [[[] for i in range(self.past_state_num)] for i in range(self.state_dim)]
        
        self.min_inputs = [1000 for i in range(self.input_dim)]
        self.max_inputs= [-1000 for i in range(self.input_dim)]
        
        self.inputs_list = [[[[[[] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.input_dim)]
        self.inputs_gmm_list = [[[[[[] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.input_dim)]

        # subscriber
        rospy.init_node('gmm', anonymous=True)
        rospy.Subscriber("sample_pcl/diabolo/pitch", Float64, self.callbackForPitch)
        rospy.Subscriber("sample_pcl/diabolo/yaw", Float64, self.callbackForYaw)
        self.now_states = [[0, 0], [0, 0]] # FIX

        # publisher
        self.pub_arm = rospy.Publisher('calc_gmm/diabolo/arm', Float64, queue_size=10)
        self.pub_base = rospy.Publisher('calc_gmm/diabolo/base', Float64, queue_size=10)
        self.next_inputs = [0, 0] # FIX

    def callbackForPitch(self, data):
        self.now_states[0][1] = data.data - self.now_states[0][0]
        self.now_states[0][0] = data.data
        
    def callbackForYaw(self, data):
        self.now_states[1][1] = data.data - self.now_states[1][0]        
        self.now_states[1] = data.data
        
    def load_data(self):
        log_files = ['../log/log-by-logger/log-by-loggerpy0.log',
             '../log/log-by-logger/log-by-loggerpy1.log']

        self.state_data_num = 0
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
                
                arm = float(w[0]) * 1
                base = float(w[1]) * 1
                pitch = float(w[2])
                yaw = float(w[3])
                pitch_ = pitch - past_pitch
                yaw_ = yaw - past_yaw
        
                cnt += 1
                if cnt < 1:
                    continue

                # store pitch, pitch_, yaw, yaw_, arm, base
                self.states[0][0].append(pitch)
                self.states[0][1].append(pitch_)
                self.states[1][0].append(yaw)
                self.states[1][1].append(yaw_)
                self.inputs[0].append(arm)
                self.inputs[1].append(base)
                
                self.state_data_num += 1

                # calc min, max of arm and base
                if self.min_inputs[0] > arm:
                    self.min_inputs[0] = arm
                if self.max_inputs[0] < arm:
                    self.max_inputs[0] = arm
                if self.min_inputs[1] > base:
                    self.min_inputs[1] = base
                if self.max_inputs[1] < base:
                    self.max_inputs[1] = base

    def split_data(self):                   
        # calc borders
        sorted_states = [[[] for i in range(self.state_dim)] for i in range(self.past_state_num)]
        self.borders_states = [[[] for i in range(self.state_dim)] for i in range(self.past_state_num)]        
        for i in range(self.state_dim):
            for j in range(self.past_state_num):
                sorted_states[i][j] = sorted(self.states[i][j])
        for i in range(self.num_of_split - 1):
            for j in range(self.state_dim):
                for k in range(self.past_state_num):
                    self.borders_states[j][k].append(sorted_states[j][k][int(1.0 * (i + 1) / self.num_of_split * self.state_data_num)])

        # split data
        for i in range(self.state_data_num):
            for j in range(self.input_dim):
                idxs = [[self.num_of_split - 1 for ii in range(self.state_dim)] for ii in range(self.past_state_num)]
                for k in range(self.state_dim):
                    for l in range(self.past_state_num):
                        s = self.states[k][l][i]
                        for m in range(self.num_of_split - 1): # search border
                            if s < self.borders_states[k][l][m]:
                                idxs[k][l] = m
                                break
                self.inputs_list[j][idxs[0][0]][idxs[0][1]][idxs[1][0]][idxs[1][1]].append(self.inputs[j][i])

    def fit_aic(self, ip, ip_, iy, iy_, plot_flag=True, max_gm_num=1): # TODO increase max_gm_num
        # fit GMM by AIC
        for i in range(self.input_dim):
            COVARIANCE_TYPES = ['spherical', 'tied', 'diag', 'full']
            args = list(itertools.product(COVARIANCE_TYPES, range(1, max_gm_num + 1)))
            models = np.zeros(len(args), dtype=object)

            X_raw = np.array([[j] for j in self.inputs_list[i][ip][ip_][iy][iy_]])
            print X_raw
            print len(X_raw)
            if len(X_raw) < 2:
                self.inputs_gmm_list[i][ip][ip_][iy][iy_] = None
                continue
            for j, (ctype, n) in enumerate(args):
                models[j] = GMM(n, covariance_type=ctype, tol=1e-10, min_covar=1e-10)
                models[j].fit(X_raw)
            aic = np.array([m.aic(X_raw) for m in models])
            self.inputs_gmm_list[i][ip][ip_][iy][iy_] = models[np.argmin(aic)] # TODO not substitude, but append

        # plot
        if plot_flag:
            fig = plt.figure()
            ax = [fig.add_subplot(211), fig.add_subplot(212)]

            for i in range(self.input_dim):
                ax[i].set_title(self.input_name[i])
                ax[i].set_xlim(self.min_inputs[i], self.max_inputs[i])
                ax[i].hist(np.array(self.inputs_list[i][ip][ip_][iy][iy_]), normed=True)
                print min(self.inputs_list[i][ip][ip_][iy][iy_]), max(self.inputs_list[i][ip][ip_][iy][iy_])
                
                if self.inputs_gmm_list[i][ip][ip_][iy][iy_] == None:
                    continue
                mean = self.inputs_gmm_list[i][ip][ip_][iy][iy_].means_
                cov = self.inputs_gmm_list[i][ip][ip_][iy][iy_]._get_covars()
                print mean, cov
                X_pred = np.linspace(self.min_inputs[i], self.max_inputs[i], 1000)
                Y_pred = self.gauss(X_pred, float(mean[0]), float(cov[0][0])) # TODO cov
                ax[i].plot(X_pred, Y_pred)
                
            plt.show()

    def save_model(self, log_name='../log/gmm/gmm.log'):
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

    def realtime_act(self):
        while True:
            [[ip, ip_], [iy, iy_]] = self.calc_states_idx()
            self.update_gmm(ip, ip_, iy, iy_)
            self.calc_next_inputs(ip, ip_, iy, iy_)
            self.publish()
            print ip, ip_, iy, iy_
            time.sleep(0.01)
            
    def calc_states_idx(self):
        states_idx = [[self.num_of_split - 1 for ii in range(self.state_dim)] for ii in range(self.past_state_num)]
        for i in range(self.state_dim):
            for j in range(self.past_state_num):
                s = self.now_states[i][j]
                for k in range(self.num_of_split - 1): # search border
                    if s < self.borders_states[i][j][k]:
                        states_idx[i][j] = k
                        break
        return states_idx

    def update_gmm(self, ip, ip_, iy, iy_):
        pass
    
    def calc_next_inputs(self, ip, ip_, iy, iy_):
        for i in range(self.input_dim):
            mean = self.inputs_gmm_list[i][ip][ip_][iy][iy_].means_
            cov = self.inputs_gmm_list[i][ip][ip_][iy][iy_]._get_covars()
            self.next_inputs[i] = np.random.normal(mean, cov, 1)
    
    def publish(self):
        #print self.next_inputs[0], self.next_inputs[1]
        self.pub_arm.publish(self.next_inputs[0])
        self.pub_base.publish(self.next_inputs[1])        

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
    
    gmm = CalcGMM()
    load_data_flag = True
    plot_flag = True
    load_model_flag = False
    juggle_flag = True

    # load data from log file, fit and save
    if load_data_flag:
        gmm.load_data()
        gmm.split_data()

        for i in range(gmm.num_of_split):
            for j in range(gmm.num_of_split):
                for k in range(gmm.num_of_split):
                    for l in range(gmm.num_of_split):
                        gmm.fit_aic(i, j, k, l, plot_flag)
                        
        gmm.save_model()
        
    # load model
    if load_model_flag:
        gmm.load_model()

    # subscribe states and publish inputs
    if juggle_flag:
        gmm.realtime_act()

