import numpy as np

camMatrix= np.zeros((3,3))
camDist= np.zeros((1,5))

if camMatrix.all() == 0 and camDist.all() == 0:
       print('done')
else:
       print("not done")