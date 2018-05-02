import numpy as np

PAST_STATE_NUM = 3
PAST_INPUT_NUM = 3
PAST_NUM = max(PAST_INPUT_NUM, PAST_STATE_NUM)

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

DATA_NUM = len(input_arm)

input_list = [np.matrix([input_arm[i], input_base[i]]).T for i in range(len(input_arm))]
state_list = [np.matrix([state_pitch[i], state_yaw[i]]).T for i in range(len(state_pitch))]

A_all = np.matrix(np.zeros((VAR_NUM, VAR_NUM)))
B_all = np.matrix(np.zeros((STATE_DIM, VAR_NUM)))

# calc B_all
# derivative of state coef
offset = 0
for i in range(PAST_STATE_NUM): # 0 - 2
    B_tmp_mat = np.matrix(np.zeros((STATE_DIM, STATE_DIM)))
    for j in range(PAST_NUM, DATA_NUM): # 3 - 19 (sigma)
        B_tmp_mat += state_list[j] * input_list[j - i - 1].T
    print B_all[0:STATE_DIM, offset + STATE_DIM * i:offset + STATE_DIM * (i + 1)]
    B_all[0:STATE_DIM, offset + STATE_DIM * i:offset + STATE_DIM * (i + 1)] = B_tmp_mat
# derivative of input coef
offset += STATE_DIM * PAST_STATE_NUM
for i in range(PAST_NUM): # 0 - 2
    B_tmp_mat = np.matrix(np.zeros((STATE_DIM, INPUT_DIM)))
    for j in range(PAST_NUM, DATA_NUM): # 3 - 19 (sigma)
        B_tmp_mat += state_list[j] * state_list[j - i - 1].T
    print B_all[0:STATE_DIM, offset + INPUT_DIM * i:offset + INPUT_DIM * (i + 1)]
    B_all[0:STATE_DIM, offset + INPUT_DIM * i:offset + INPUT_DIM * (i + 1)] = B_tmp_mat
# derivative of bias
offset += INPUT_DIM * PAST_INPUT_NUM
for i in range(1): # 0 - 2
    B_tmp_mat = np.matrix(np.zeros((STATE_DIM, 1)))
    for j in range(PAST_NUM, DATA_NUM): # 3 - 9 (sigma)
        B_tmp_mat += state_list[j]
    print B_all[0:STATE_DIM, offset + 1 * i:offset + 1 * (i + 1)]        
    B_all[0:STATE_DIM, offset + 1 * i:offset + 1 * (i + 1)] = B_tmp_mat
    
# calc A_all
# derivative of state coef
wide_offset = 0
for i in range(PAST_STATE_NUM): # 0 - 2
    narrow_offset = 0
    for j in range(PAST_INPUT_NUM):
        A_tmp_mat = np.matrix(np.zeros((STATE_DIM, STATE_DIM)))
        for k in range(PAST_INPUT_NUM, DATA_NUM): # 3 - 9 (sigma)
            A_tmp_mat += state_list[k - j] * state_list[k - i - 1].T
        print A_all[narrow_offset + INPUT_DIM * j:narrow_offset + INPUT_DIM * (j + 1), wide_offset + INPUT_DIM * i:wide_offset + INPUT_DIM * (i + 1)]
        A_all[narrow_offset + INPUT_DIM * j:narrow_offset + INPUT_DIM * (j + 1), wide_offset + INPUT_DIM * i:wide_offset + INPUT_DIM * (i + 1)] = A_tmp_mat
    narrow_offset += PAST_STATE_NUM * STATE_DIM
    for j in range(PAST_STATE_NUM):
        A_tmp_mat = np.matrix(np.zeros((INPUT_DIM, STATE_DIM)))
        for k in range(PAST_INPUT_NUM, DATA_NUM): # 3 - 9 (sigma)        
            A_tmp_mat += input_list[k - j] * state_list[k - i - 1].T
        print A_all[narrow_offset + INPUT_DIM * j:narrow_offset + INPUT_DIM * (j + 1), wide_offset + INPUT_DIM * i:wide_offset + INPUT_DIM * (i + 1)]
        A_all[narrow_offset + INPUT_DIM * j:narrow_offset + INPUT_DIM * (j + 1), wide_offset + INPUT_DIM * i:wide_offset + INPUT_DIM * (i + 1)] = A_tmp_mat
    narrow_offset += PAST_INPUT_NUM * INPUT_DIM
    for j in range(1):
        A_tmp_mat = np.matrix(np.zeros((1, STATE_DIM)))                
        for k in range(PAST_INPUT_NUM, DATA_NUM): # 3 - 9 (sigma)        
            A_tmp_mat += state_list[k - i - 1].T
        print A_all[narrow_offset + INPUT_DIM * j:narrow_offset + INPUT_DIM * (j + 1), wide_offset + INPUT_DIM * i:wide_offset + INPUT_DIM * (i + 1)]
        A_all[narrow_offset + INPUT_DIM * j:narrow_offset + INPUT_DIM * (j + 1), wide_offset + INPUT_DIM * i:wide_offset + INPUT_DIM * (i + 1)] = A_tmp_mat
# derivative of input coef
wide_offset += STATE_DIM * PAST_STATE_NUM
for i in range(PAST_INPUT_NUM): # 0 - 2
    narrow_offset = 0
    for j in range(PAST_INPUT_NUM):
        A_tmp_mat = np.matrix(np.zeros((INPUT_DIM, INPUT_DIM)))
        for k in range(PAST_INPUT_NUM, DATA_NUM): # 3 - 9 (sigma)
            A_tmp_mat += state_list[k - j] * input_list[k - i - 1].T
        print A_all[narrow_offset + INPUT_DIM * j:narrow_offset + INPUT_DIM * (j + 1), wide_offset + INPUT_DIM * i: wide_offset + INPUT_DIM * (i + 1)]
        A_all[narrow_offset + INPUT_DIM * j:narrow_offset + INPUT_DIM * (j + 1), wide_offset + INPUT_DIM * i: wide_offset + INPUT_DIM * (i + 1)] = A_tmp_mat
    narrow_offset += PAST_INPUT_NUM * INPUT_DIM
    for j in range(PAST_STATE_NUM):
        A_tmp_mat = np.matrix(np.zeros((STATE_DIM, INPUT_DIM)))
        for k in range(PAST_INPUT_NUM, DATA_NUM): # 3 - 9 (sigma)        
            A_tmp_mat += input_list[k - j] * input_list[k - i - 1].T
        print A_all[narrow_offset + INPUT_DIM * j:narrow_offset + INPUT_DIM * (j + 1), wide_offset + INPUT_DIM * i:wide_offset + INPUT_DIM * (i + 1)]
        A_all[narrow_offset + INPUT_DIM * j:narrow_offset + INPUT_DIM * (j + 1), wide_offset + INPUT_DIM * i:wide_offset + INPUT_DIM * (i + 1)] = A_tmp_mat
    narrow_offset += PAST_STATE_NUM * STATE_DIM
    for j in range(1):
        A_tmp_mat = np.matrix(np.zeros((1, INPUT_DIM)))                
        for k in range(PAST_INPUT_NUM, DATA_NUM): # 3 - 9 (sigma)        
            A_tmp_mat += input_list[k - i - 1].T
        print A_all[narrow_offset + INPUT_DIM * j:narrow_offset + INPUT_DIM * (j + 1), wide_offset + INPUT_DIM * i:wide_offset + INPUT_DIM * (i + 1)]
        A_all[narrow_offset + INPUT_DIM * j:narrow_offset + INPUT_DIM * (j + 1), wide_offset + INPUT_DIM * i:wide_offset + INPUT_DIM * (i + 1)] = A_tmp_mat
# derivative of bias
wide_offset += PAST_STATE_NUM * STATE_DIM
for i in range(1): # 0
    narrow_offset = 0
    for j in range(PAST_INPUT_NUM):
        A_tmp_mat = np.matrix(np.zeros((INPUT_DIM, 1)))
        for k in range(PAST_INPUT_NUM, DATA_NUM): # 3 - 9 (sigma)
            A_tmp_mat += state_list[k - j]
        print A_all[narrow_offset + INPUT_DIM * j:narrow_offset + INPUT_DIM * (j + 1), wide_offset + INPUT_DIM * i:wide_offset + INPUT_DIM * (i + 1)]
        A_all[narrow_offset + INPUT_DIM * j:narrow_offset + INPUT_DIM * (j + 1), wide_offset + INPUT_DIM * i:wide_offset + INPUT_DIM * (i + 1)] = A_tmp_mat
    narrow_offset += PAST_INPUT_NUM * INPUT_DIM
    for j in range(PAST_STATE_NUM):
        A_tmp_mat = np.matrix(np.zeros((STATE_DIM, 1)))
        for k in range(PAST_INPUT_NUM, DATA_NUM): # 3 - 9 (sigma)        
            A_tmp_mat += input_list[k - j]
        print A_all[narrow_offset + INPUT_DIM * j:narrow_offset + INPUT_DIM * (j + 1), wide_offset + INPUT_DIM * i:wide_offset + INPUT_DIM * (i + 1)]
        A_all[narrow_offset + INPUT_DIM * j:narrow_offset + INPUT_DIM * (j + 1), wide_offset + INPUT_DIM * i:wide_offset + INPUT_DIM * (i + 1)] = A_tmp_mat
    narrow_offset += PAST_STATE_NUM * STATE_DIM
    for j in range(1):
        A_tmp_mat = np.matrix(np.zeros((1, 1)))                
        for k in range(PAST_INPUT_NUM, DATA_NUM): # 3 - 9 (sigma)        
            A_tmp_mat += 1
        print A_all[narrow_offset + INPUT_DIM * j:narrow_offset + INPUT_DIM * (j + 1), wide_offset + INPUT_DIM * i:wide_offset + INPUT_DIM * (i + 1)]
        A_all[narrow_offset + INPUT_DIM * j:narrow_offset + INPUT_DIM * (j + 1), wide_offset + INPUT_DIM * i:wide_offset + INPUT_DIM * (i + 1)] = A_tmp_mat

# calc parameters        
print np.linalg.matrix_rank(A_all)
print B_all * A_all**-1

