import numpy as np
from sklearn.model_selection import train_test_split
from keras.models import Sequential
from keras.layers import Dense, Activation, Dropout
from keras.wrappers.scikit_learn import KerasRegressor

LOG_FILES = ['../log/log-by-logger/log-by-loggerpy0.log',
             '../log/log-by-logger/log-by-loggerpy1.log',
             '../log/log-by-logger/log-by-loggerpy2.log',
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
                
                    self.input_arm.append(val[0])
                    self.input_base.append(val[1])
                    self.state_pitch.append(val[2])
                    self.state_yaw.append(val[3][:-1])

    def make_model(self):
        model = Sequential()
        
        model.add(Dense(self.INPUT_NN_DIM, input_dim=self.INPUT_NN_DIM, activation='relu'))
        #model.add(BatchNormalization())
        model.add(Dropout(0.2))
        model.add(Dense(128, activation='relu'))
        #model.add(BatchNormalization())
        model.add(Dropout(0.2))
        model.add(Dense(self.OUTPUT_NN_DIM))

        model.compile(loss='mean_squared_error', optimizer='adam')
        model.summary()

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

        self.estimator = KerasRegressor(build_fn=self.make_model, epochs=100, batch_size=10, verbose=0)
        self.estimator.fit(x_train, y_train)

        f = open('output.log', 'w')
        self.y_pred = self.estimator.predict(x_test)
        for i in range(len(self.y_pred)):
            for j in range(len(self.y_pred[i])):
                f.write('{} {}\n'.format(y_test[i][j], self.y_pred[i][j]))
        f.close()

if __name__ == '__main__':
    sg = SystemRegression()
    sg.execute(LOG_FILES)
