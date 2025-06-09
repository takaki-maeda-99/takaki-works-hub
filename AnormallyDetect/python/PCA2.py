import numpy as np
import pickle
from sklearn.decomposition import PCA


N2 = 250

with open('ac.bin', 'rb') as p:
    ac = pickle.load(p)

pca2 = PCA( n_components = N2 )
ac = pca2.fit_transform(ac)

print(np.sum(pca2.explained_variance_ratio_))

with open('2zPCA.bin', 'wb') as p:
    pickle.dump(pca2, p)

