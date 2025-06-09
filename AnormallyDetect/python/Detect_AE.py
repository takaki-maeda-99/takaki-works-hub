import numpy as np
import matplotlib.pyplot as plt
import pickle



with open('a1.bin', 'rb') as p:
    a1 = pickle.load(p)

with open('b1.bin', 'rb') as p:
    b1 = pickle.load(p)

with open('1zAE.bin', 'rb') as p:
    ae1 = pickle.load(p)

a_ano = np.abs(a1 - ae1.predict(a1))
b_ano = np.abs(b1 - ae1.predict(b1))

(l1,l2) = a1.shape
(m1,m2) = b1.shape


with open('ap.bin', 'rb') as p:
    ap = pickle.load(p)
(l1,l2,l3) = ap

# partition = np.zeros((m1,))
# partition[(335,625,985,1305,1655),] = np.max(np.sum(b_ano,axis=1))
# plt.plot(np.arange(len(a_ano)), np.sum(a_ano,axis=1))
# plt.plot(np.arange(len(b_ano)), np.sum(b_ano,axis=1))
# plt.plot(np.arange(len(partition)), partition)
# plt.show()
# plt.cla()







with open('ac.bin', 'rb') as p:
    ac = pickle.load(p)

with open('bc.bin', 'rb') as p:
    bc = pickle.load(p)

with open('2zAE.bin', 'rb') as p:
    ae2 = pickle.load(p)



a_anoo = np.abs(ac - ae2.predict(ac))
b_anoo = np.abs(bc - ae2.predict(bc))

a_anoo[a_anoo < 2 ] = 0
b_anoo[b_anoo < 2 ] = 0

(l1,l2) = ac.shape
(m1,m2) = bc.shape

# partition = np.zeros((m1,))
# partition[(335,625,985,1305,1655),] = np.max(np.sum(b_anoo,axis=1))
# plt.plot(np.arange(len(a_anoo)), np.sum(a_anoo,axis=1))
# plt.plot(np.arange(len(b_anoo)), np.sum(b_anoo,axis=1))
# plt.plot(np.arange(len(partition)), partition)
# plt.show()
# plt.cla()