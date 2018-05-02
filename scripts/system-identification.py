import numpy as np

PAST_STATE_NUM = 3
PAST_INPUT_NUM = 3
PAST_NUM = max(PAST_STATE_NUM, PAST_INPUT_NUM)

STATE_DIM = 2
INPUT_DIM = 2

VAR_NUM = PAST_STATE_NUM * STATE_DIM + PAST_INPUT_NUM * INPUT_DIM + 1

input_arm = [1, 3, 3, 4, 5, 6, 7, 3, 5, 10, 12, 3, 14, 5, 12, 5, 8, 2, 1, 2]
input_base = [12, 3, 14, 5, 12, 5, 8, 2, 1, 2, 1, 3, 3, 4, 5, 6, 7, 3, 5, 10]

state_pitch = [1, 2, 3]
state_yaw = [3, 4, 5]
for ii in range(len(input_arm ) - 3):
    i = ii + 3
    
    tmp1 = 1 * input_arm[i - 1] + 2 * input_arm[i - 2] + 3 * input_arm[i - 3]
    + 4 * input_base[i - 1] + 5 * input_base[i - 2] + 6 * input_base[i - 3]
    + 7 * state_pitch[i - 1] + 8 * state_pitch[i - 2] + 9 * state_pitch[i - 3]
    + 10 * state_yaw[i - 1] + 11 * state_yaw[i - 2] + 12 * state_yaw[i - 3] + 10
    
    tmp2 = 11 * input_arm[i - 1] + 12 * input_arm[i - 2] + 13 * input_arm[i - 3]
    + 14 * input_base[i - 1] + 15 * input_base[i - 2] + 16 * input_base[i - 3]
    + 17 * state_pitch[i - 1] + 18 * state_pitch[i - 2] + 19 * state_pitch[i - 3]
    + 110 * state_yaw[i - 1] + 111 * state_yaw[i - 2] + 112 * state_yaw[i - 3] + 20
    
    state_pitch.append(tmp1)
    state_yaw.append(tmp2)

input_list = [np.matrix([input_arm[i], input_base[i]]).T for i in range(len(input_arm))]
state_list = [np.matrix([state_pitch[i], state_yaw[i]]).T for i in range(len(state_pitch))]

DATA_NUM = len(input_arm)

X = np.matrix(np.zeros((VAR_NUM, DATA_NUM - PAST_NUM)))
Y = np.matrix(np.zeros((STATE_DIM, DATA_NUM - PAST_NUM)))

# assign X
for i in range(DATA_NUM - PAST_NUM): # 0-16   not 3-19
    for j in range(PAST_STATE_NUM):
        X[j * STATE_DIM:(j + 1) * STATE_DIM, i:i + 1] = state_list[i + PAST_NUM - j - 1]
    for j in range(PAST_INPUT_NUM):
        X[PAST_STATE_NUM * STATE_DIM + j * INPUT_DIM:PAST_STATE_NUM * STATE_DIM + (j + 1) * INPUT_DIM , i:i + 1] = input_list[i + PAST_NUM - j - 1]
    X[PAST_STATE_NUM * STATE_DIM + PAST_INPUT_NUM * INPUT_DIM:PAST_STATE_NUM * STATE_DIM + PAST_INPUT_NUM * INPUT_DIM + 1, i:i + 1] = 1

# assign Y
for i in range(DATA_NUM - PAST_NUM): # 0 - 16   not 3-19
    Y[0:STATE_DIM, i:i + 1] = state_list[i + PAST_NUM]

A = Y * np.linalg.pinv(X)
print A
