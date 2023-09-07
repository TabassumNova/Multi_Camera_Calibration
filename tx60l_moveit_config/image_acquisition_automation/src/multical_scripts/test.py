from scipy.spatial.transform import Rotation as R
import math
from src.multical.transform.matrix import *
import numpy as np
r, t = (split(np.eye(4)))
rotation_deg = R.magnitude(R.from_matrix(r)) * 180.0 / math.pi
translation = np.linalg.norm(t)
pass