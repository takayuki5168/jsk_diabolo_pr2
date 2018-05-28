import numpy as np
from sklearn.model_selection import train_test_split
from keras.models import Sequential
from keras.layers import Dense, Activation, Dropout, BatchNormalization
from keras.wrappers.scikit_learn import KerasRegressor
from keras.optimizers import Adam, RMSprop

LOG_FILES = [#'../log/log-by-logger/log-by-loggerpy0.log',
             #'../log/log-by-logger/log-by-loggerpy1.log',
             #'../log/log-by-logger/log-by-loggerpy2.log',
             '../log/log-by-logger/log-by-loggerpy3.log',
             '../log/log-by-logger/log-by-loggerpy4.log',
             '../log/log-by-logger/log-by-loggerpy5.log']             

class SystemRegression:
    def __init__(self):
        self.input_arm = []
        self.input_base = []
        self.state_pitch = []
        self.state_yaw = []
        
        self.PAST_STATE_NUM = 2
        self.PAST_INPUT_NUM = 1
        self.PAST_NUM = max(self.PAST_STATE_NUM, self.PAST_INPUT_NUM)
        
        self.STATE_DIM = 2
        self.INPUT_DIM = 2

        self.DELTA_STEP = 5

        self.INPUT_NN_DIM = self.STATE_DIM * self.PAST_STATE_NUM + self.INPUT_DIM * self.PAST_INPUT_NUM
        self.OUTPUT_NN_DIM = self.STATE_DIM
        
        self.VAR_NUM = max(self.INPUT_NN_DIM, self.OUTPUT_NN_DIM)

    def execute(self, log_files):
        self.load_data(log_files)
        self.make_model()
        self.arrange_data()
        self.train()
    
    def load_data(self, log_files):
        for log_file in log_files:
            with open(log_file, "r") as f:
                for l in f.readlines():
                    val = l.split(' ')
                    if len(val) != 4:
                        break
                
                    self.input_arm.append(float(val[0]))
                    self.input_base.append(float(val[1]))
                    self.state_pitch.append(float(val[2]))
                    self.state_yaw.append(float(val[3][:-1]))

    def make_model(self):
        model = Sequential()
        
        model.add(Dense(100, input_shape=(self.INPUT_NN_DIM, ), activation='relu')) # input_dim=self.INPUT_NN_DIM, activation='relu'))
        #model.add(BatchNormalization())
        #model.add(Dropout(0.2))
        #model.add(Dense(20, activation='relu'))
        #model.add(BatchNormalization())
        #model.add(Dropout(0.2))
        model.add(Dense(self.OUTPUT_NN_DIM))

        model.compile(loss='mean_squared_error', optimizer=RMSprop(lr=0.01))
        #model.summary()

        return model

    def arrange_data(self):
        self.X = []
        self.Y = []
        
        for i in range((max(self.PAST_STATE_NUM, self.PAST_INPUT_NUM) + 1) * self.DELTA_STEP, len(self.input_arm)):
            x = []
            y = [self.state_pitch[i], self.state_yaw[i]]            
            for j in range(self.PAST_STATE_NUM):
                x.append(self.state_pitch[i - (j + 1) * self.DELTA_STEP])
                x.append(self.state_yaw[i - (j + 1) * self.DELTA_STEP])
            for j in range(self.PAST_INPUT_NUM):
                x.append(self.input_arm[i - (j + 1) * self.DELTA_STEP])
                x.append(self.input_base[i - (j + 1) * self.DELTA_STEP])
            self.X.append(x)
            self.Y.append(y)
            
    def train(self):
        x_train, x_test, y_train, y_test = train_test_split(np.array(self.X), np.array(self.Y), test_size=int(len(self.X) * 0.2), shuffle=False)

        model = self.make_model()
        model.fit(x_train, y_train,
                    batch_size=50,
                    epochs=1000
                    )

        #print(self.y_pred)

        #self.estimator = KerasRegressor(build_fn=self.make_model, epochs=10, batch_size=10, verbose=0)
        #self.estimator.fit(x_train, y_train)
        
        self.y_pred = model.predict(x_test)
        f_arm_pitch_test = open('output_arm_pitch_test.log', 'w')
        f_base_yaw_test = open('output_base_yaw_test.log', 'w')
        f_test = open('output_test.log', 'w')                
        for i in range(1, len(self.y_pred) - 1):
            f_arm_pitch_test.write('{} {} {}\n'.format(x_test[i - 1][4], y_test[i][0] - y_test[i - 1][0], self.y_pred[i][0] - y_test[i - 1][0]))
            f_base_yaw_test.write('{} {} {}\n'.format(x_test[i - 1][5], y_test[i][1] - y_test[i - 1][1], self.y_pred[i][1] - y_test[i - 1][1]))
            f_test.write('{} {} {} {} {} {}\n'.format(x_test[i - 1][4], x_test[i - 1][5], y_test[i][0] - y_test[i - 1][0], self.y_pred[i][0] - y_test[i - 1][0], y_test[i][1] - y_test[i - 1][1], self.y_pred[i][1] - y_test[i - 1][1]))
        f_arm_pitch_test.close()
        f_base_yaw_test.close()

        self.y_pred = model.predict(x_train)        
        f_arm_pitch_train = open('output_arm_pitch_train.log', 'w')
        f_base_yaw_train = open('output_base_yaw_train.log', 'w')
        f_train = open('output_train.log', 'w')                
        for i in range(1, len(self.y_pred) - 1):
            f_arm_pitch_train.write('{} {} {}\n'.format(x_train[i - 1][4], y_train[i][0] - y_train[i - 1][0], self.y_pred[i][0] - y_train[i - 1][0]))
            f_base_yaw_train.write('{} {} {}\n'.format(x_train[i - 1][5], y_train[i][1] - y_train[i - 1][1], self.y_pred[i][1] - y_train[i - 1][1]))
            f_train.write('{} {} {} {} {} {}\n'.format(x_train[i - 1][4], x_train[i - 1][5], y_train[i][0] - y_train[i - 1][0], self.y_pred[i][0] - y_train[i - 1][0], y_train[i][1] - y_train[i - 1][1], self.y_pred[i][1] - y_train[i - 1][1]))
        f_arm_pitch_train.close()
        f_base_yaw_train.close()
        f_train.close()                
        

if __name__ == '__main__':
    sg = SystemRegression()
    sg.execute(LOG_FILES)
