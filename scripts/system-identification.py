import numpy as np

PAST_INPUT_NUM = 3
PAST_STATE_NUM = 3
PAST_NUM = max(PAST_INPUT_NUM, PAST_STATE_NUM)

INPUT_DIM = 2
STATE_DIM = 2

VAR_NUM = PAST_INPUT_NUM * INPUT_DIM**2 + PAST_STATE_NUM * INPUT_DIM * STATE_DIM + STATE_DIM


input_arm = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
input_base = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
state_pitch = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
state_yaw = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

input_vector = [[input_arm[i], input_base[i]] for i in range(len(input_arm))]
state_vector = [[state_pitch[i], state_yaw[i]] for i in range(len(state_pitch))]

A_all = np.zeros((VAR_NUM, VAR_NUM))
b_all = np.zeros(VAR_NUM)

# derive of state coef 
for i in range(PAST_INPUT_NUM): # 0 - 2
    for j in range(STATE_DIM):
        for k in range(STATE_DIM):
            for l in range(PAST_INPUT_NUM, len(input_arm)): # 3 - 9  (sigma)              
                b_all[i * STATE_DIM * STATE_DIM + j * STATE_DIM + k] += state_vector[l][j] * state_vector[l - i][k]

for i in range(STATE_DIM):
    for j in range(STATE_DIM):
        for k in range(STATE_DIM):
            for l in range(STATE_DIM):
                A_all[i * STATE_DIM + j][i * STATE_DIM + j] = (state_vector[t][] * state_vector[t-k][])[][]
# derive of input coef 
for i in range(PAST_INPUT_NUM): # 0 - 2
    for j in range(STATE_DIM):
        for k in range(STATE_DIM):
            for l in range(PAST_INPUT_NUM, len(input_arm)): # 3 - 9  (sigma)              
                b_all[STATE_DIM * STATE_DIM * PAST_INPUT_NUM + i * STATE_DIM * STATE_DIM + j * STATE_DIM + k] += state_vector[l][j] * input_vector[l - i][k]

# derive of state bias 
for i in range(STATE_DIM):
    for j in range(PAST_INPUT_NUM, len(input_arm)): # 3 - 9  (sigma)
       b_all[STATE_DIM * STATE_DIM * PAST_INPUT_NUM + STATE_DIM * INPUT_DIM * PAST_INPUT_NUM + i] += state_vector[j][i]
