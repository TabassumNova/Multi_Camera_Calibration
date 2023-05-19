import numpy as np

# h = np.array([[1,0,0,1],[0,1,0,1],[0,0,1,1]])
# v= np.array([[0,0,0,1]])
# x = np.concatenate((h,v), axis=0)
# print(h.shape,v.shape,x.shape)

x = np.array([[1], [2], [3]])
y = np.array([4, 5, 6])
b = np.broadcast(x, y)
out = np.empty(b.shape)
out.flat = [u+v for (u,v) in b]
print(out)