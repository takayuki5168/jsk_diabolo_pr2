import numpy as np
import random

LOG_FILES = ['../log/log-by-loggerpy0.log',
             '../log/log-by-loggerpy1.log',
             '../log/log-by-loggerpy2.log',
             '../log/log-by-loggerpy3.log']

def fit(log, DERIVATIVE_NUM_ = 3 - 1)
    # init params
    DERIVATIVE_NUM = DERIVATIVE_NUM_ # DERIVATIVE_NUM of state feedback
    STATE_DIM = 2
    INPUT_DIM = 2
    dt = 1 / 30.0 # TODO
    VAR_NUM = (DERIVATIVE_NUM + 1) * STATE_DIM + INPUT_DIM


    # load data
    input_arm = []
    input_base = []
    state_pitch = []
    state_yaw = []
    with open(LOG_FILES[log], "r") as f:
        for l in f.readlines():
            val = l.split(' ')
            if len(val) != 4:
                break
            
            input_arm.append(val[0])
            input_base.append(val[1])
            state_pitch.append(val[2])
            state_yaw.append(val[3][:-1])
            
    # init variable            
    input_list = [np.matrix([input_arm[i], input_base[i]]).T for i in range(len(input_arm))]
    state_list = [np.matrix([state_pitch[i], state_yaw[i]]).T for i in range(len(state_pitch))]


    # calc derivative
    state_derivatives_list = [[] for i in range(DERIVATIVE_NUM + 2)]
    state_derivatives_list[0] = state_list
    for i in range(DERIVATIVE_NUM + 2):
        if i == 0: # i == 0 is not derivative
            continue
        for j in range(DERIVATIVE_NUM, DATA_NUM):
            state_derivatives_list[i].append((state_derivatives_list[i - 1][j] - state_derivatives_list[i - 1][j - 1]) / dt)

            
    # assign Y
    Y = np.matrix(np.zeros((STATE_DIM, DATA_NUM)))    
    for i in range(DATA_NUM):
        Y[0:STATE_DIM, i:i + 1] = state_derivatives_list[-1][i]
    
    # assign X
    X = np.matrix(np.zeros((VAR_NUM, DATA_NUM)))
    for i in range(DATA_NUM):
        for j in range(DERIVATIVE_NUM):
            X[j * STATE_DIM:(j + 1) * STATE_DIM, i: i + 1] = state_derivatives_list[j][i]
        X[DERIVATIVE_NUM * STATE_DIM:(DERIVATIVE_NUM + 1) * STATE_DIM, i:i + 1] = input_list[i]


    # calc params
    A = Y * np.linalg.pinv(X)
    print A

    
fit(0, 2)
