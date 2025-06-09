import numpy as np
import pickle
from sklearn.decomposition import PCA



with open('am.bin', 'rb') as p:
    am = pickle.load(p)


NM = 100

pcam = PCA( n_components = NM )
am = pcam.fit_transform(am)

print(np.sum(pcam.explained_variance_ratio_))


with open('1zPCAM.bin', 'wb') as p:
    pickle.dump(pcam, p)


# (l1,l2) = am.shape

# INPUT_N = l2

# aem = models.Sequential()
# aem.add(Dense(int(INPUT_N/16),input_shape=(INPUT_N,),activation="relu"))
# aem.add(Dense(INPUT_N, activation="linear"))
# aem.compile(loss="mean_squared_error", optimizer="adam")

# history = aem.fit(
#     am,
#     am,
#     epochs = 50,
#     batch_size = 256,
#     validation_split = 0.2
# )

# loss = history.history['loss']  # 訓練用データの誤差
# vloss = history.history['val_loss']  # 検証用データの誤差

# plt.plot(np.arange(len(loss)), loss)
# plt.plot(np.arange(len(vloss)), vloss)
# plt.show()

# encoder = models.clone_model(aem)
# encoder.compile(loss="mean_squared_error", optimizer="adam")
# encoder.set_weights(aem.get_weights())
# encoder.pop()

# with open('1zEncoder.bin', 'wb') as p:
#     pickle.dump(encoder, p)
