#!/usr/bin/env python3

import numpy as np
from keras.models import Sequential
from keras.layers import Input, Dense, Activation, Lambda
from keras.regularizers import l2
from keras.optimizers import Adam
from keras import models

INPUT_DIM      = 3
OUTPUT_DIM     = 1
HIDDEN_LAYER_1 = 100
HIDDEN_LAYER_2 = 100
L2             = 0.01
ACTION_BOUND   = 255
CLIP_VALUE     = 40
BATCH_SIZE     = 64
EPOCH_NUM      = 60

data_file = './../../data/balance-small-2.csv'
data = np.genfromtxt(data_file, delimiter=',', skip_header=1)

x_train = data[:, :INPUT_DIM]
y_train = data[:,  INPUT_DIM].reshape(-1, 1)

# =================================Network=================================

states = Input(shape=(INPUT_DIM,), name='states')

# layer1 
net = Dense(units=HIDDEN_LAYER_1, activation='relu', kernel_regularizer=l2(L2))(states)

# layer2 
# net = Dense(units=HIDDEN_LAYER_2, activation='relu', kernel_regularizer=l2(L2))(net)

# layer3 
actions = Dense(units=OUTPUT_DIM, kernel_regularizer=l2(L2))(net)
# actions = Lambda(lambda x: x * ACTION_BOUND)(actions)

model = models.Model(inputs=states, outputs=actions) 

op = Adam(clipvalue=CLIP_VALUE)
model.compile(loss='mse', 
              optimizer=op)

# =================================Train=================================

history = model.fit(x_train, 
                    y_train, 
                    batch_size=BATCH_SIZE, 
                    verbose=0, 
                    epochs=EPOCH_NUM, 
                    shuffle=True, 
                    validation_split=0.5)

loss = history.history.get('loss')
val_loss = history.history.get('val_loss')

model.save('fit-PID-model.h5')

# =================================Plot=================================

import matplotlib.pyplot as plt
plt.figure(0)

plt.subplot(121)
plt.plot(range(len(loss)), loss)
plt.title('Loss')

plt.subplot(122)
plt.plot(range(len(val_loss)), val_loss)
plt.title('Val Loss')

plt.show()
plt.close()


