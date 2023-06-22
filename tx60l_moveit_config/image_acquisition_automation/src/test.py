import json
import numpy as np

x = np.arange(0, 280.0, 5)
y = np.arange(0, -280, -5)
z = np.concatenate((x,y))

for i in z:
    print(i)