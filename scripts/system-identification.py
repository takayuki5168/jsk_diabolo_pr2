import numpy as np
import random

LOG_FILES = ['../log/log-by-loggerpy.log',
             '../log/log-by-loggerpy2.log',
             '../log/log-by-loggerpy3.log',
             '../log/log-by-loggerpy4.log']

def fit(log1, log2, PRINT_A_ = True, PAST_STATE_NUM_ = 6, PAST_INPUT_NUM_ = 6):
    # init params
    PAST_STATE_NUM = PAST_STATE_NUM_
    PAST_INPUT_NUM = PAST_INPUT_NUM_
    PAST_NUM = max(PAST_STATE_NUM, PAST_INPUT_NUM)
    STATE_DIM = 2
    INPUT_DIM = 2
    
    #VAR_NUM = PAST_STATE_NUM * STATE_DIM + PAST_INPUT_NUM * INPUT_DIM + 1
    VAR_NUM = PAST_STATE_NUM * STATE_DIM + PAST_INPUT_NUM * INPUT_DIM
    
    
    
    # load data
    input_arm = []
    input_base = []
    state_pitch = []
    state_yaw = []
    with open(LOG_FILES[log1], "r") as f:
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
    
    DATA_NUM = len(input_arm)
    
    X = np.matrix(np.zeros((VAR_NUM, DATA_NUM - PAST_NUM)))
    Y = np.matrix(np.zeros((STATE_DIM, DATA_NUM - PAST_NUM)))
    
    
    
    for i in range(DATA_NUM - PAST_NUM): # 0-16
        # assign X
        for j in range(PAST_STATE_NUM):
            X[j * STATE_DIM:(j + 1) * STATE_DIM, i:i + 1] = state_list[i + PAST_NUM - j - 1]
        for j in range(PAST_INPUT_NUM):
            X[PAST_STATE_NUM * STATE_DIM + j * INPUT_DIM:PAST_STATE_NUM * STATE_DIM + (j + 1) * INPUT_DIM , i:i + 1] = input_list[i + PAST_NUM - j - 1]
        #X[PAST_STATE_NUM * STATE_DIM + PAST_INPUT_NUM * INPUT_DIM:PAST_STATE_NUM * STATE_DIM + PAST_INPUT_NUM * INPUT_DIM + 1, i:i + 1] = 1
        
        # assign Y
        for i in range(DATA_NUM - PAST_NUM): # 0-16
            Y[0:STATE_DIM, i:i + 1] = state_list[i + PAST_NUM]
    
    
    # calc params
    A = Y * np.linalg.pinv(X)
    if PRINT_A_:
        print A
    
    
    
    # reload data
    input_arm = []
    input_base = []
    state_pitch = []
    state_yaw = []
    with open(LOG_FILES[log2], "r") as f:
        for l in f.readlines():
            val = l.split(' ')
    
            if len(val) != 4:
                break
            
            input_arm.append(val[0])
            input_base.append(val[1])
            state_pitch.append(val[2])
            state_yaw.append(val[3][:-1])
    
    # reinit variable
    input_list = [np.matrix([input_arm[i], input_base[i]]).T for i in range(len(input_arm))]
    state_list = [np.matrix([state_pitch[i], state_yaw[i]]).T for i in range(len(state_pitch))]
    
    DATA_NUM = len(input_arm)
    
    X = np.matrix(np.zeros((VAR_NUM, DATA_NUM - PAST_NUM)))
    Y = np.matrix(np.zeros((STATE_DIM, DATA_NUM - PAST_NUM)))
    
    
    for i in range(DATA_NUM - PAST_NUM): # 0-16
        # reassign X
        for j in range(PAST_STATE_NUM):
            X[j * STATE_DIM:(j + 1) * STATE_DIM, i:i + 1] = state_list[i + PAST_NUM - j - 1]
        for j in range(PAST_INPUT_NUM):
            X[PAST_STATE_NUM * STATE_DIM + j * INPUT_DIM:PAST_STATE_NUM * STATE_DIM + (j + 1) * INPUT_DIM , i:i + 1] = input_list[i + PAST_NUM - j - 1]
        #X[PAST_STATE_NUM * STATE_DIM + PAST_INPUT_NUM * INPUT_DIM:PAST_STATE_NUM * STATE_DIM + PAST_INPUT_NUM * INPUT_DIM + 1, i:i + 1] = 1
    
        # reassign Y
        for i in range(DATA_NUM - PAST_NUM): # 0-16
            Y[0:STATE_DIM, i:i + 1] = state_list[i + PAST_NUM]
            
    # predict
    file_pitch = open('../log/fitting_pitch.log', 'w')
    file_yaw = open('../log/fitting_yaw.log', 'w')
    
    predicted_Y = []
    for i in range(DATA_NUM - PAST_NUM): # 0-16
        predicted_Y.append(A * X[0:VAR_NUM, i:i + 1])
    
    error_pitch = 0
    error_yaw = 0
    # write log randomly    
    for i in range(DATA_NUM - PAST_NUM): # 0-16
        idx = int((random.random() * 1000000) % (DATA_NUM - PAST_NUM))
        file_pitch.write('{} {}\n'.format(float(predicted_Y[idx][0]), float(Y[0:STATE_DIM, idx:idx + 1][0])))
        file_yaw.write('{} {}\n'.format(float(predicted_Y[idx][1]), float(Y[0:STATE_DIM, idx:idx + 1][1])))
        error_pitch += abs(float(predicted_Y[idx][0]) - float(Y[0:STATE_DIM, idx:idx + 1][0]))
        error_yaw += abs(float(predicted_Y[idx][1]) - float(Y[0:STATE_DIM, idx:idx + 1][1]))
        
    file_pitch.close()    
    file_yaw.close()
    
    print '[LogFile] ' + str(log1) + ' ' + str(log2) + ' [PastNum] ' + str(PAST_STATE_NUM_) + ' ' + str(PAST_INPUT_NUM_) + ' [ErrorPitch] ' + str(error_pitch) + ' [ErrorYaw] ' + str(error_yaw)
    return error_pitch, error_yaw

    
# test once
fit(2, 1, True)

# test some cases
#TEST_NUM = 15
#error_pitch_all = [0 for i in range(TEST_NUM)]
#error_yaw_all = [0 for i in range(TEST_NUM)]
#for i in range(len(LOG_FILES)):
#    for j in range(TEST_NUM):
#        res1, res2 = fit(1, i, False, j + 1, j + 1)
#        error_pitch_all[j] += res1
#        error_yaw_all[j] += res2
#print '[ErrorPitchAll] ' + str(error_pitch_all) + ' [ErrorYawAll] ' + str(error_yaw_all)

# test all cases
#for i in range(len(LOG_FILES)):
#    for j in range(len(LOG_FILES)):
#        for k in range(20):
#            fit(i, j, False, k + 1, k + 1)

    
