import numpy as np
import pickle
from sklearn.decomposition import PCA


N1 = 100

with open('a1.bin', 'rb') as p:
    a1 = pickle.load(p)

pca1 = PCA( n_components = N1 )
a1 = pca1.fit_transform(a1)

print(np.sum(pca1.explained_variance_ratio_))

with open('1zPCA.bin', 'wb') as p:
    pickle.dump(pca1, p)

