# -*- coding: utf-8 -*-
import cv2
import numpy as np
import matplotlib.pyplot as plt
from sklearn.decomposition import PCA
from sklearn import preprocessing
from keras.layers import Dense
from keras.models import Sequential



#　　定数　　#
PATH1 = "data/test3.mp4"
PATH2 = "data/test1.mp4"
RE = 12
TH0 = 80
N1 = 100
N2 = 400
CL = 10

#　　動画関係　　#
import cv2
def readVF():
    ret,frame = v_cap.read()                                #フレーム読み込み
    if not ret:                                             #読み込み失敗でret=Falseを返す
        return 0,ret
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)         #フレームをグレイスケール化して縮小
    frame = cv2.resize(frame, (int(W/RE),int(H/RE)))
    return ret, frame

def preceed(c1,c2,name):
    print(int((c1/c2)*100),"%",name)
    return

#　　正常データの準備　　#

v_cap = cv2.VideoCapture( PATH1 )
W = int(v_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
H = int(v_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
L = int(v_cap.get(cv2.CAP_PROP_FRAME_COUNT))

print(L*int(W/RE)*int(H/RE),L,int(W/RE),int(H/RE))

for i in range(L):
    ret,f0 = readVF()
    if not ret :
        break
    if i == 1:
        a1 = [cv2.absdiff(f0, bg)]
    if i != 1 and i != 0 :
        a1 = np.append(a1, [cv2.absdiff(f0, bg)], axis=0)
    bg = f0
    preceed(i,L,"a1")
v_cap.release()

v_cap = cv2.VideoCapture( PATH2 )
W = int(v_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
H = int(v_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
L = int(v_cap.get(cv2.CAP_PROP_FRAME_COUNT))
for i in range(L):
    ret,f0 = readVF()
    if not ret :
        break
    if i == 1:
        b1 = [cv2.absdiff(f0, bg)]
    if i != 1 and i != 0 :
        b1 = np.append(b1, [cv2.absdiff(f0, bg)], axis=0)
    bg = f0
    preceed(i,L,"b1")
v_cap.release()

a1[a1 < TH0] = 0
a1[a1 >= TH0] = 255

b1[b1 < TH0] = 0
b1[b1 >= TH0] = 255

# a1 = a1/255
# b1 = b1/255

# for n in a1 :
#     cv2.imshow("Mask", n)
#     if cv2.waitKey(100) & 0xFF == 27:                     #エスケープキーが押されたら処理終了
#         break

(l1,l2,l3) = a1.shape
(m1,m2,m3) = b1.shape

a1 = np.reshape(a1,(l1,l2*l3))
b1 = np.reshape(b1,(m1,m2*m3))

ai = a1
bi = b1

ss1 = preprocessing.StandardScaler()
pca1 = PCA( n_components = N1 )

a1 = ss1.fit_transform(a1)
a1 = pca1.fit_transform(a1)
b1 = ss1.transform(b1)
b1 = pca1.transform(b1)

ar = a1
br = b1

a1 = pca1.inverse_transform(a1)
ao = ss1.inverse_transform(a1)
b1 = pca1.inverse_transform(b1)
bo = ss1.inverse_transform(b1)

a_ano = np.abs(ai - ao)
b_ano = np.abs(bi - bo)

a_ano[a_ano < 250 ] = 0
b_ano[b_ano < 250 ] = 0

print(np.sum(pca1.explained_variance_ratio_))

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

ac = ss2.fit_transform(ac)
bc = ss2.transform(bc)

aii = ac
bii = bc

# ac = pca2.fit_transform(ac)
# bc = pca2.transform(bc)

# ac = pca2.inverse_transform(ac)
# aoo = ss2.inverse_transform(ac)
# bc = pca2.inverse_transform(bc)
# boo = ss2.inverse_transform(bc)

# a_anoo = np.abs(aii - aoo)
# b_anoo = np.abs(bii - boo)

# print(np.sum(pca2.explained_variance_ratio_))

# partition = np.zeros((L,))
# partition[(335,625,985,1305,1655),] = np.max(np.sum(b_anoo,axis=1))
# plt.plot(np.arange(len(a_anoo)), np.sum(a_anoo,axis=1))
# plt.plot(np.arange(len(b_anoo)), np.sum(b_anoo,axis=1))
# plt.plot(np.arange(len(partition)), partition)
# plt.show()
# plt.cla()

INPUT_N = N1*CL

model = Sequential()
model.add(Dense(int(INPUT_N/3),input_shape=(INPUT_N,),activation="relu"))
model.add(Dense(INPUT_N, activation="linear"))
model.compile(loss="mean_squared_error", optimizer="adam")

history = model.fit(
    ac,
    ac,
    epochs = 50,
    batch_size = 50,
    validation_split = 0.2
)

a_anoo = np.abs(aii - model.predict(aii))
b_anoo = np.abs(bii - model.predict(bii))

a_anoo[a_anoo < 2 ] = 0
b_anoo[b_anoo < 2 ] = 0

partition = np.zeros((L,))
partition[(335,625,985,1305,1655),] = np.max(np.sum(b_anoo,axis=1))
plt.plot(np.arange(len(a_anoo)), np.sum(a_anoo,axis=1))
plt.plot(np.arange(len(b_anoo)), np.sum(b_anoo,axis=1))
plt.plot(np.arange(len(partition)), partition)
plt.show()
plt.cla()


# hey = np.reshape(b_ano,(m1,m2,m3))

# for n in hey :
#     cv2.imshow("why", n)
#     if cv2.waitKey(100) & 0xFF == 27:                     #エスケープキーが押されたら処理終了
#         break


# i = 0      # カウント変数
# th = 50    # 差分画像の閾値

# # 動画ファイルのキャプチャ
# cap = cv2.VideoCapture(
#     "data/test1.mp4")

# # 最初のフレームを背景画像に設定
# ret, bg = cap.read()

# # グレースケール変換
# bg = cv2.cvtColor(bg, cv2.COLOR_BGR2GRAY)

# while(cap.isOpened()):
#     # フレームの取得
#     ret, frame = cap.read()

#     # グレースケール変換
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#     # 差分の絶対値を計算
#     mask = cv2.absdiff(gray, bg)

#     # 差分画像を二値化してマスク画像を算出
#     mask[mask < th] = 0
#     mask[mask >= th] = 255

#     # フレームとマスク画像を表示
#     cv2.imshow("Mask", mask)
#     time.sleep(0.03)

#     # 待機(0.03sec)

#     # 背景画像の更新（一定間隔）
#     ret, bg = cap.read()
#     bg = cv2.cvtColor(bg, cv2.COLOR_BGR2GRAY)

#     # qキーが押されたら途中終了
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# cap.release()
# cv2.destroyAllWindows()
