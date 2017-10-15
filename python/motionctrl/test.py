import numpy as np
import warnings

a = np.array([[1,2,3], [4,5,6]])
b = np.zeros([2, 4])
b[:, 0:3] = a
print(b)
a = np.array([[1,1,1], [1,1,1]])
print(b)
b[:,-1] = np.nan
print(b)
print(b[:,:-1])
