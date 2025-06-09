import cv2
import numpy as np
import pickle
from sklearn import preprocessing



PATH1 = "/Users/taka/Library/Mobile Documents/com~apple~CloudDocs/AO/data/test3.mp4"
PATH2 = "/Users/taka/Library/Mobile Documents/com~apple~CloudDocs/AO/data/test1.mp4"
RE = 12
TH0 = 70



def readVF():
    ret,frame = v_cap.read()
    if not ret:
        return 0,ret
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame = cv2.resize(frame, (int(W/RE),int(H/RE)))
    return ret, frame

def looding(c1,c2,name):
    print(int((c1/c2)*100),"%",name)
    return



v_cap = cv2.VideoCapture( PATH1 )
W = int(v_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
H = int(v_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
L = int(v_cap.get(cv2.CAP_PROP_FRAME_COUNT))

for i in range(L):
    ret,f0 = readVF()
    if not ret :
        break
    if i == 1:
        a1 = [f0.flatten()]
    if i != 1 and i != 0 :
        a1 = np.append(a1, [f0.flatten()], axis=0)
    looding(i,L,"a1")

v_cap.release()

with open('as.bin', 'wb') as p:
    pickle.dump(a1, p)



v_cap = cv2.VideoCapture( PATH2 )
W = int(v_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
H = int(v_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
L = int(v_cap.get(cv2.CAP_PROP_FRAME_COUNT))

for i in range(L):
    ret,f0 = readVF()
    if not ret :
        break
    if i == 1:
        b1 = [f0.flatten()]
    if i != 1 and i != 0 :
        b1 = np.append(b1, [f0.flatten()], axis=0)
    looding(i,L,"b1")

v_cap.release()

with open('bs.bin', 'wb') as p:
    pickle.dump(b1, p)



(l1,l2) = a1.shape
(m1,m2) = b1.shape

ap = (l1,int(H/RE),int(W/RE))
bp = (m1,int(H/RE),int(W/RE))

am = cv2.absdiff(a1[1:l1],a1[0:l1-1])
bm = cv2.absdiff(b1[1:m1],b1[0:m1-1])

ss1 = preprocessing.StandardScaler()
a1 = ss1.fit_transform(a1)
b1 = ss1.transform(b1)



with open('a1.bin', 'wb') as p:
    pickle.dump(a1, p)

with open('b1.bin', 'wb') as p:
    pickle.dump(b1, p)

with open('ap.bin', 'wb') as p:
    pickle.dump(ap, p)

with open('bp.bin', 'wb') as p:
    pickle.dump(bp, p)

with open('ss1.bin', 'wb') as p:
    pickle.dump(ss1, p)




am[ am < TH0 ] = 0
am[ am >= TH0 ] = 255

bm[ bm < TH0 ] = 0
bm[ bm >= TH0 ] = 255

ssm = preprocessing.StandardScaler()
am = ssm.fit_transform(am)
bm = ssm.transform(bm)

with open('am.bin', 'wb') as p:
    pickle.dump(am, p)

with open('bm.bin', 'wb') as p:
    pickle.dump(bm, p)

with open('ssm.bin', 'wb') as p:
    pickle.dump(ssm, p)

