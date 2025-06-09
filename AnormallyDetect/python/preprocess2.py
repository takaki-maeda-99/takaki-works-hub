import numpy as np
import pickle
from sklearn import preprocessing

CL = 20

with open('am.bin', 'rb') as p:
    am = pickle.load(p)

with open('bm.bin', 'rb') as p:
    bm = pickle.load(p)


# with open('1zEncoder.bin', 'rb') as p:
#     encoder = pickle.load(p)
# ar = encoder.predict(am)
# br = encoder.predict(bm)


with open('1zPCAM.bin', 'rb') as p:
    pcam = pickle.load(p)

ar = pcam.transform(am)
br = pcam.transform(bm)


(l1,l2) = ar.shape
(m1,m2) = br.shape


for c in range(int(l1-CL+1)):

    if c == 0:
        ac = [ar[0:CL,:].flatten()]
    else:
        ac = np.append(ac,[ar[c:c+CL,:].flatten()],axis=0)
    print(c)

for c in range(int(m1-CL+1)):

    if c == 0:
        bc = [br[0:CL,:].flatten()]
    else:
        bc = np.append(bc,[br[c:c+CL,:].flatten()],axis=0)
    print(c)


ss2 = preprocessing.StandardScaler()
ac = ss2.fit_transform(ac)
bc = ss2.transform(bc)

with open('ac.bin', 'wb') as p:
    pickle.dump(ac, p)

with open('bc.bin', 'wb') as p:
    pickle.dump(bc, p)