import numpy as np
import matplotlib.pyplot as plt
import pickle
from keras.layers import Dense
from keras import models

with open('ac.bin', 'rb') as p:
    ac = pickle.load(p)

(l1,l2) = ac.shape

INPUT_N = l2

ae2 = models.Sequential()
ae2.add(Dense(int(INPUT_N/2),input_shape=(INPUT_N,),activation="relu"))
ae2.add(Dense(INPUT_N, activation="linear"))
ae2.compile(loss="mean_squared_error", optimizer="adam")

history = ae2.fit(
    ac,
    ac,
    epochs = 50,
    batch_size = 256,
    validation_split = 0.1
)

loss = history.history['loss']  # 訓練用データの誤差
vloss = history.history['val_loss']  # 検証用データの誤差

plt.plot(np.arange(len(loss)), loss)
plt.plot(np.arange(len(vloss)), vloss)
plt.show()

with open('2zAE.bin', 'wb') as p:
    pickle.dump(ae2, p)
