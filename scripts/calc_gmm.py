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
        self.min_states = [0, 0, 0, 0]   # pitch, pitch_, yaw, yaw_
        self.max_states = [5, 15, 5, 5]   # pitch, pitch_, yaw, yaw_

        self.arms = []
        self.bases = []
        
        self.pitchs = []
        self.pitchs_ = []
        self.yaws = []
        self.yaws_ = []
        
        self.min_inputs = [1000, 1000]
        self.max_inputs= [-1000, -1000]

        self.num_of_split = 2
        
        self.arm_list = [[[[[] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)]
        self.base_list = [[[[[] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)]
        self.arm_gmm_list = [[[[[] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)]
        self.base_gmm_list = [[[[[] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)] for i in range(self.num_of_split)]                
        
    def load_data(self):
        log_files = ['../log/log-by-logger/log-by-loggerpy0.log',
             '../log/log-by-logger/log-by-loggerpy1.log',
             '../log/log-by-logger/log-by-loggerpy2.log',
             '../log/log-by-logger/log-by-loggerpy3.log',
             '../log/log-by-logger/log-by-loggerpy4.log']        

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
                self.pitchs.append(pitch)
                self.pitchs_.append(pitch_)
                self.yaws.append(yaw)
                self.yaws_.append(yaw_)
                self.arms.append(arm)
                self.bases.append(base)

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
        sorted_pitchs = sorted(self.pitchs)
        sorted_pitchs_ = sorted(self.pitchs_)
        sorted_yaws = sorted(self.yaws)
        sorted_yaws_ = sorted(self.yaws_)
        borders_pitch = []
        borders_pitch_ = []
        borders_yaw = []
        borders_yaw_ = []
        for i in range(self.num_of_split - 1):
            borders_pitch.append(sorted_pitchs[int(1.0 * (i + 1) / self.num_of_split * len(self.pitchs))])
            borders_pitch_.append(sorted_pitchs_[int(1.0 * (i + 1) / self.num_of_split * len(self.pitchs_))])
            borders_yaw.append(sorted_yaws[int(1.0 * (i + 1) / self.num_of_split * len(self.yaws))])
            borders_yaw_.append(sorted_yaws_[int(1.0 * (i + 1) / self.num_of_split * len(self.yaws_))])
        
        # split data
        for i in range(len(self.pitchs)):
            pitch = self.pitchs[i]
            pitch_ = self.pitchs_[i]
            yaw = self.yaws[i]
            yaw_ = self.yaws_[i]
            arm = self.arms[i]
            base = self.bases[i]

            idx_pitch = self.num_of_split - 1
            idx_pitch_ = self.num_of_split - 1
            idx_yaw = self.num_of_split - 1
            idx_yaw_ = self.num_of_split - 1
            for j in range(self.num_of_split - 1):
                if pitch < borders_pitch[j]:
                    idx_pitch = j
                    break
            for j in range(self.num_of_split - 1):
                if pitch_ < borders_pitch_[j]:
                    idx_pitch_ = j
                    break
            for j in range(self.num_of_split - 1):
                if yaw < borders_yaw[j]:
                    idx_yaw = j
                    break
            for j in range(self.num_of_split - 1):
                if yaw_ < borders_yaw_[j]:
                    idx_yaw_ = j
                    break
                
            self.arm_list[idx_pitch][idx_pitch_][idx_yaw][idx_yaw_].append(arm)
            self.base_list[idx_pitch][idx_pitch_][idx_yaw][idx_yaw_].append(base)

    def fit_aic(self, ip, ip_, iy, iy_, max_gm_num = 5, plot_flag=True):
        COVARIANCE_TYPES = ['spherical', 'tied', 'diag', 'full']
        args = list(itertools.product(COVARIANCE_TYPES, range(1, max_gm_num + 1)))
        models = np.zeros(len(args), dtype=object)
        
        X = np.array([[i] for i in self.arm_list[ip][ip_][iy][iy_]])
        for i, (ctype, n) in enumerate(args):
            models[i] = GMM(n, covariance_type=ctype)
            models[i].fit(X)
        aic = np.array([m.aic(X) for m in models])
        self.arm_gmm_list[ip][ip_][iy][iy_] = models[np.argmin(aic)]
        
        X = np.array([[i] for i in self.base_list[ip][ip_][iy][iy_]])
        for i, (ctype, n) in enumerate(args):
            models[i] = GMM(n, covariance_type=ctype)
            models[i].fit(X)
        aic = np.array([m.aic(X) for m in models])
        self.base_gmm_list[ip][ip_][iy][iy_] = models[np.argmin(aic)]

        if plot_flag:
            fig = plt.figure()
            ax1 = fig.add_subplot(211)
            ax2 = fig.add_subplot(212)
            
            ax1.set_title('arm')
            ax1.set_xlim(self.min_inputs[0], self.max_inputs[0])
            ax1.hist(np.array(self.arm_list[ip][ip_][iy][iy_]), bins=50)
            X_raw = np.array([[i] for i in self.arm_list[ip][ip_][iy][iy_]])
            f = lambda x : np.exp(self.arm_gmm_list[ip][ip_][iy][iy_].score(x))        
            X_pred = np.linspace(self.min_inputs[0], self.max_inputs[0], 1000)
            Y_pred = np.vectorize(f)(X_pred)
            ax1.plot(X_pred, Y_pred)        
            
            ax2.set_title('base')
            ax2.set_xlim(self.min_inputs[1], self.max_inputs[1])
            ax2.hist(np.array(self.base_list[ip][ip_][iy][iy_]), bins=50)
            X_raw = np.array([[i] for i in self.base_list[ip][ip_][iy][iy_]])
            f = lambda x : np.exp(self.base_gmm_list[ip][ip_][iy][iy_].score(x))        
            X_pred = np.linspace(self.min_inputs[1], self.max_inputs[1], 1000)
            Y_pred = np.vectorize(f)(X_pred)
            ax2.plot(X_pred, Y_pred)        
            
            plt.show()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigIntHandler)
                    
    gmm = CalcGMM()
    gmm.load_data()
    gmm.split_data()
    gmm.fit_aic(0, 0, 1, 1)
