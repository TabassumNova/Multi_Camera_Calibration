import cv2
import numpy as np


from structs.struct import struct
# from py_structs import struct
from src.multical.transform import rtvec


def aruco_config(attrs):
  config = cv2.aruco.DetectorParameters_create()
  for k, v in attrs.items():
    assert hasattr(config, k), f"aruco_config: no such detector parameter {k}"
    setattr(config, k, v)  
  return config

empty_detection = struct(corners=np.zeros([0, 2]), ids=np.zeros(0, dtype=np.int))
empty_matches = struct(points1=[], points2=[], ids=[], object_points=[])



def create_dict(name, offset):
  dict_id = name if isinstance(name, int)\
    else getattr(cv2.aruco, f'DICT_{name}')

  aruco_dict=cv2.aruco.getPredefinedDictionary(dict_id)
  aruco_dict.bytesList=aruco_dict.bytesList[offset:]
  return aruco_dict


def has_min_detections_grid(grid_size, ids, min_points, min_rows):
  w, h = grid_size
  dims = np.unravel_index(ids, shape=(h, w)) 
  has_rows = [np.unique(d).size >= min_rows for d in dims]
  return ids.size >= min_points and all(has_rows)

def estimate_pose_points(board, camera, detections, method="solvePnPGeneric"):
    if not board.has_min_detections(detections):
        return None, 0, [None]

    undistorted = camera.undistort_points(detections.corners).astype('float32')
    # undistorted = (detections.corners).astype('float32')

    if method == "solvePnPGeneric":
        objPoints = board.points[detections.ids].astype('float32')
        valid, rvec, tvec, error = cv2.solvePnPGeneric(objPoints, undistorted, camera.intrinsic, camera.dist)
        inliers = np.arange(len(objPoints)).reshape((1,-1))[0].tolist()

    if method == "solvePnPRansac":
        objPoints = board.points[detections.ids].astype('float32')
        valid0, rvec0, tvec0, error0 = cv2.solvePnPGeneric(objPoints, undistorted, camera.intrinsic, camera.dist)
        valid1, rvec1, tvec1, inliers1 = cv2.solvePnPRansac(objPoints, undistorted, camera.intrinsic, camera.dist, reprojectionError = 1.0)
        if valid1 == False:
            valid, rvec, tvec, error = valid0, rvec0, tvec0, error0
            inliers = [None]
        elif valid1 == True and len(inliers1.tolist())<6:
            valid, rvec, tvec, error = valid0, rvec0, tvec0, error0
            inliers = [None]
        else:
            objPoints2 = np.array([objPoints[i[0]] for i in inliers1])
            undistorted2 = np.array([undistorted[i[0]] for i in inliers1])
            valid, rvec, tvec, error = cv2.solvePnPGeneric(objPoints2, undistorted2, camera.intrinsic, camera.dist)
            inliers = inliers1.reshape((1,-1))[0].tolist()

    elif method == "solvePnP":
        valid, rvec, tvec = cv2.solvePnP(board.points[detections.ids], undistorted, camera.intrinsic, camera.dist)
        error = np.zeros((2, 1))
        inliers = [None]

    elif method == "solvePnP_P3P":
        objPoints = board.points[detections.ids].astype('float32')
        ret, rvecs, tvecs = cv2.solveP3P(objPoints[0:3,:], undistorted[0:3,:], camera.intrinsic, camera.dist, flags=cv2.SOLVEPNP_P3P)
        error = np.zeros((2, 1))
        inliers = [None]


    if not valid:
      return None, 0, [None]

    return rtvec.join(rvec[0].flatten(), tvec[0].flatten()), error[0][0], inliers


def subpix_corners(image, detections, window):
  criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.0001)               
  reshaped = np.array(detections.corners).reshape(-1, 1, 2).astype(np.float32)
  refined = cv2.cornerSubPix(image, reshaped, (window, window), (-1, -1), criteria)
  return detections._extend(corners=refined.reshape(-1, 2))


def quad_polygons(quads):
  assert quads.ndim == 2 and quads.shape[1] == 4

  # Append 4 (number of sides) to each quad
  return np.concatenate([np.full( (quads.shape[0], 1), 4), quads], axis=1)

def grid_mesh(points, size):
  w, h = size
  indices = np.arange(points.shape[0]).reshape(h - 1, w - 1)
  quad = np.array([indices[0, 0], indices[1, 0], indices[1, 1], indices[0, 1]])
  offsets = indices[: h - 2, :w - 2]

  quads = quad.reshape(1, 4) + offsets.reshape(-1, 1)
  return struct(points=points, polygons=quad_polygons(quads))









