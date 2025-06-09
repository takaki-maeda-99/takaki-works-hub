# -*- coding: utf-8 -*-
import cv2
import numpy as np
import matplotlib.pyplot as plt
import pickle
from sklearn.decomposition import PCA
from sklearn import preprocessing
from keras.layers import Dense
from keras import models

#　　定数　　#
N1 = 30
N2 = 2
CL = 3

with open('a1.bin', 'rb') as p:
    a1 = pickle.load(p)

with open('b1.bin', 'rb') as p:
    b1 = pickle.load(p)


(l1,l2) = a1.shape
(m1,m2) = b1.shape

ss1 = preprocessing.StandardScaler()
pca1 = PCA( n_components = N1 )

a1 = ss1.fit_transform(a1)
b1 = ss1.transform(b1)

ai = a1
bi = b1

with open('1zAE.bin', 'rb') as p:
    model = pickle.load(p)

encoder = models.clone_model(model)
encoder.compile(loss="mean_squared_error", optimizer="adam")
encoder.set_weights(model.get_weights())
encoder.pop()

# a1 = pca1.fit_transform(a1)
# b1 = pca1.transform(b1)

a1 = encoder.predict(a1)
b1 = encoder.predict(b1)

ar = a1
br = b1

# ao = pca1.inverse_transform(a1)
# bo = pca1.inverse_transform(b1)

ao = model.predict(ai)
bo = model.predict(bi)

a_ano = np.abs(ai - ao)
b_ano = np.abs(bi - bo)

# hey = np.reshape(b_ano,(m1,int(H/RE),int(W/RE)))

# for n in hey :
#     cv2.imshow("mask",n)
#     if cv2.waitKey(100) & 0xFF == 27:                     #エスケープキーが押されたら処理終了
#         break

a_ano[a_ano < 30 ] = 0
b_ano[b_ano < 30 ] = 0

# print(np.sum(pca1.explained_variance_ratio_))

partition = np.zeros((m1,))
partition[(335,625,985,1305,1655),] = np.max(np.sum(b_ano,axis=1))
plt.plot(np.arange(len(a_ano)), np.sum(a_ano,axis=1))
plt.plot(np.arange(len(b_ano)), np.sum(b_ano,axis=1))
plt.plot(np.arange(len(partition)), partition)
plt.show()
plt.cla()


a_ano[a_ano < 35 ] = 0
b_ano[b_ano < 35 ] = 0

# print(np.sum(pca1.explained_variance_ratio_))

partition = np.zeros((L,))
partition[(335,625,985,1305,1655),] = np.max(np.sum(b_ano,axis=1))
plt.plot(np.arange(len(a_ano)), np.sum(a_ano,axis=1))
plt.plot(np.arange(len(b_ano)), np.sum(b_ano,axis=1))
plt.plot(np.arange(len(partition)), partition)
plt.show()
plt.cla()

a_ano[a_ano < 40 ] = 0
b_ano[b_ano < 40 ] = 0

# print(np.sum(pca1.explained_variance_ratio_))

partition = np.zeros((L,))
partition[(335,625,985,1305,1655),] = np.max(np.sum(b_ano,axis=1))
plt.plot(np.arange(len(a_ano)), np.sum(a_ano,axis=1))
plt.plot(np.arange(len(b_ano)), np.sum(b_ano,axis=1))
plt.plot(np.arange(len(partition)), partition)
plt.show()
plt.cla()

for c in range(int(l1-CL+1)):

    if c == 0:
        ac = [ar[0:CL,:].flatten()]
    else:
        ac = np.append(ac,[ar[c:c+CL,:].flatten()],axis=0)
    preceed(c,int(l1-CL+1),"ac")


for c in range(int(m1-CL+1)):
    if c == 0:
        bc = [br[0:CL,:].flatten()]
    else:
        bc = np.append(bc,[br[c:c+CL,:].flatten()],axis=0)
    preceed(c,int(m1-CL+1),"bc")



ss2 = preprocessing.StandardScaler()
pca2 = PCA( n_components = N2 )

ac = ss1.fit_transform(ac)
ac = pca1.fit_transform(ac)
bc = ss1.transform(bc)
bc = pca1.transform(bc)

fig,ax = plt.subplots()

def plot(X, color :str):
    ax.scatter(X[:,0], X[:,1], color=color, alpha=0.3 )

plot(ac,"b")
plot(bc,"r")

plt.show()