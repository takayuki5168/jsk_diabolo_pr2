import numpy as np

PAST_INPUT_NUM = 3
PAST_STATE_NUM = 3

INPUT_NUM = 2
STATE_NUM = 2

VAR_NUM = PAST_INPUT_NUM * (INPUT_NUM ** 2) + PAST_STATE_NUM * (STATE_NUM ** 2) + STATE_NUM

np.array([1, 2, 3, 4])

pitch = np.array([1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
yaw = np.array([1, 2, 3, 4, 5, 6, 7, 8, 9, 10])

A_all = np.zeros((VAR_NUM, VAR_NUM))
b_all = np.zeros(VAR_NUM)

# derive of state coef 
for i in range(PAST_INPUT_NUM): # 0 - 10
    for j in range(np.size(pitch) - PAST_INPUT_NUM): # 0 - 7
        for k in range(STATE_NUM):
            for l in range(STATE_NUM):
                b_all[i + k * l] += state[j][k] * state[j][l]

# derive of input coef 

# derive of state bias 
