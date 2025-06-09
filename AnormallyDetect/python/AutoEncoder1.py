import numpy as np
import matplotlib.pyplot as plt
import pickle
from keras.layers import Dense
from keras import models


with open('a1.bin', 'rb') as p:
    a1 = pickle.load(p)

(l1,l2) = a1.shape

INPUT_N = l2

ae1 = models.Sequential()
ae1.add(Dense(int(INPUT_N/8),input_shape=(INPUT_N,),activation="relu"))
ae1.add(Dense(INPUT_N, activation="linear"))
ae1.compile(loss="mean_squared_error", optimizer="adam")

history = ae1.fit(
    a1,
    a1,
    epochs = 50,
    batch_size = 256,
    validation_split = 0.1
)

loss = history.history['loss']  # 訓練用データの誤差
vloss = history.history['val_loss']  # 検証用データの誤差

plt.plot(np.arange(len(loss)), loss)
plt.plot(np.arange(len(vloss)), vloss)
plt.show()

with open('1zAE.bin', 'wb') as p:
    pickle.dump(ae1, p)

