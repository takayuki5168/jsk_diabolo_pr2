import numpy as np

# init params
PAST_STATE_NUM = 1
PAST_INPUT_NUM = 1
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
with open("../log/log-by-loggerpy4.log", "r") as f:
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

# assign X
for i in range(DATA_NUM - PAST_NUM): # 0-16
    for j in range(PAST_STATE_NUM):
        X[j * STATE_DIM:(j + 1) * STATE_DIM, i:i + 1] = state_list[i + PAST_NUM - j - 1]
    for j in range(PAST_INPUT_NUM):
        X[PAST_STATE_NUM * STATE_DIM + j * INPUT_DIM:PAST_STATE_NUM * STATE_DIM + (j + 1) * INPUT_DIM , i:i + 1] = input_list[i + PAST_NUM - j - 1]
    X[PAST_STATE_NUM * STATE_DIM + PAST_INPUT_NUM * INPUT_DIM:PAST_STATE_NUM * STATE_DIM + PAST_INPUT_NUM * INPUT_DIM + 1, i:i + 1] = 1

# assign Y
for i in range(DATA_NUM - PAST_NUM): # 0-16
    Y[0:STATE_DIM, i:i + 1] = state_list[i + PAST_NUM]

# calc params
A = Y * np.linalg.pinv(X)
print A


# predict
file_pitch = open('../log/fitting_pitch.log', 'w')
file_yaw = open('../log/fitting_yaw.log', 'w')

for i in range(DATA_NUM - PAST_NUM): # 0-16
    predicted_Y = A * X[0:VAR_NUM, i:i + 1]
    file_pitch.write('{} {}\n'.format(float(predicted_Y[0]), float(Y[0:STATE_DIM, i:i + 1][0])))
    file_yaw.write('{} {}\n'.format(float(predicted_Y[1]), float(Y[0:STATE_DIM, i:i + 1][1])))
    
file_pitch.close()    
file_yaw.close()
