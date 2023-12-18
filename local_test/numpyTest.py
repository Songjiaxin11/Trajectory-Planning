import numpy as np

arr = np.array([[1, 2], [3, 4]])
print(arr)
print(arr ** 2)
print(np.sum(arr ** 2, axis=0))
print(np.sqrt(np.sum(arr ** 2, axis=0)))
print(np.sum(np.sqrt(np.sum(arr ** 2, axis=0))))