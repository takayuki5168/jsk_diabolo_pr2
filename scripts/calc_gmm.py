# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import numpy as np
import signal, sys
from sklearn.mixture import GMM
import itertools

def sigIntHandler(signal, frame):
    sys.exit(0)

class CalcGMM:
    def __init__(self):
        self.input_dim = 2
        self.state_dim = 2
        self.past_state_num = 2
        self.num_of_split = 2 # FIX ME
        self.input_name = ['arm', 'base'] # FIX ME
        
        self.inputs = [[] for i in range(self.input_dim)]
        self.states = [[[] for i in range(self.past_state_num)] for i in range(self.state_dim)]
        
        self.min_inputs = [1000 for i in range(self.input_dim)]
        self.max_inputs= [-1000 for i in range(self.input_dim)]
        
        self.inputs_list = [[[[[[] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.input_dim)]
        self.inputs_gmm_list = [[[[[[] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.input_dim)]
        
    def load_data(self):
        log_files = ['../log/log-by-logger/log-by-loggerpy0.log',
             '../log/log-by-logger/log-by-loggerpy1.log',
             '../log/log-by-logger/log-by-loggerpy2.log',
             '../log/log-by-logger/log-by-loggerpy3.log',
             '../log/log-by-logger/log-by-loggerpy4.log']        
        self.state_data_num = 0
        for log_file in log_files:
          pitch = 0
          yaw = 0
          odom_w = 0
          cnt = 0
          with open(log_file) as f:
            for l in f.readlines():
                w = l.split(' ')
                if len(w) != 4:
                    continue
                
                past_pitch = pitch
                past_yaw = yaw
                past_odom_w = odom_w
                
                arm = float(w[0])
                odom_w = float(w[1])
                base = odom_w - past_odom_w
                
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
        borders_states = [[[] for i in range(self.state_dim)] for i in range(self.past_state_num)]        
        for i in range(self.state_dim):
            for j in range(self.past_state_num):
                sorted_states[i][j] = sorted(self.states[i][j])
                
        for i in range(self.num_of_split - 1):
            for j in range(self.state_dim):
                for k in range(self.past_state_num):
                    borders_states[j][k].append(sorted_states[j][k][int(1.0 * (i + 1) / self.num_of_split * self.state_data_num)])

        # split data
        for i in range(self.state_data_num):
            for j in range(self.input_dim):
                idxs = [[self.num_of_split - 1 for ii in range(self.state_dim)] for ii in range(self.past_state_num)]
                for k in range(self.state_dim):
                    for l in range(self.past_state_num):
                        s = self.states[k][l][i]
                        for m in range(self.num_of_split - 1): # search border
                            if s < borders_states[k][l][m]:
                                idxs[k][l] = m
                                break
                self.inputs_list[j][idxs[0][0]][idxs[0][1]][idxs[1][0]][idxs[1][1]].append(self.inputs[j][i])

    def fit_aic(self, ip, ip_, iy, iy_, plot_flag=True, max_gm_num=1): # TODO increase max_gm_num
        for i in range(self.input_dim):
            COVARIANCE_TYPES = ['spherical', 'tied', 'diag', 'full']
            args = list(itertools.product(COVARIANCE_TYPES, range(1, max_gm_num + 1)))
            models = np.zeros(len(args), dtype=object)

            X_raw = np.array([[j] for j in self.inputs_list[i][ip][ip_][iy][iy_]])
            # X_raw = np.array([[i] for i in [0.6] * 8 + [0.8] * 10])
            for j, (ctype, n) in enumerate(args):
                models[j] = GMM(n, covariance_type=ctype)
                models[j].fit(X_raw)
            aic = np.array([m.aic(X_raw) for m in models])
            self.inputs_gmm_list[i][ip][ip_][iy][iy_] = models[np.argmin(aic)]
            
        if plot_flag:
            fig = plt.figure()
            ax = []
            ax.append(fig.add_subplot(211))
            ax.append(fig.add_subplot(212))

            for i in range(self.input_dim):
                ax[i].set_title(self.input_name[i])
                ax[i].set_xlim(self.min_inputs[i], self.max_inputs[i])
                ax[i].hist(np.array(self.inputs_list[i][ip][ip_][iy][iy_]), bins=30)
                f = lambda x : np.exp(self.inputs_gmm_list[i][ip][ip_][iy][iy_].score(x))
                X_pred = np.linspace(self.min_inputs[i], self.max_inputs[i], 1000)
                Y_pred = np.vectorize(f)(X_pred)
                ax[i].plot(X_pred, Y_pred)
                
            plt.show()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigIntHandler)
                    
    gmm = CalcGMM()
    gmm.load_data()
    gmm.split_data()
    for i in range(gmm.num_of_split):
        for j in range(gmm.num_of_split):
            for k in range(gmm.num_of_split):
                for l in range(gmm.num_of_split):
                    gmm.fit_aic(i, j, k, l)
