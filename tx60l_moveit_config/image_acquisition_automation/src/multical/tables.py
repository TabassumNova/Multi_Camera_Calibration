from functools import partial
from multical.io.report import report_pose_errors
from .io.logging import debug, info
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

from structs.struct import transpose_structs, invert_keys
from structs.numpy import shape_info, struct, Table, shape

from .transform import rtvec, matrix
from . import graph



def fill_sparse(n, values, ids):
  dense = np.zeros((n, *values.shape[1:]), dtype=values.dtype)
  dense[ids] = values

  mask = np.full(n, False)
  mask[ids] = True
  return dense, mask

def fill_sparse_tile(n, values, ids, tile):
  assert tile.shape == values.shape[1:]
  dense = np.broadcast_to(np.expand_dims(tile, 0), (n, *tile.shape)).copy()
  dense[ids] = values

  mask = np.full(n, False)
  mask[ids] = True
  return dense, mask



def sparse_points(points):
  ids = np.flatnonzero(points.valid)
  return struct(corners=points.points[ids], ids=ids)

invalid_pose = struct(poses=np.eye(4), num_points=0, valid=False)

def valid_pose(t):
  return struct(poses=t, valid=True)


def extract_pose(points, board, camera):
  detections = sparse_points(points)
  poses = board.estimate_pose_points(camera, detections)

  return valid_pose(rtvec.to_matrix(poses))._extend(num_points=len(detections.ids))\
      if poses is not None else invalid_pose

def map_table(f, point_table, boards, cameras):
  return [[[f(points, board, camera)
           for points, board in zip(frame_points._sequence(), boards)]  
             for frame_points in points_camera._sequence()]
               for points_camera, camera in zip(point_table._sequence(), cameras)]

def make_pose_table(point_table, boards, cameras):
  poses = map_table(extract_pose, point_table, boards, cameras)
  return make_nd_table(poses, n = 3)

def make_point_table(detections, boards):
  num_points = np.max([board.num_points for board in boards])

  def extract_points(frame_dets):    
    points, mask = fill_sparse(
        num_points, frame_dets.corners, frame_dets.ids)
    return struct(points=points, valid=mask)

  points = [[[extract_points(d) for d in frame_dets]
            for frame_dets in cam_dets]
              for cam_dets in detections]


  return make_nd_table(points, n = 3)
  
  
def make_nd_table(items, n):
  if n > 1:
    rows = [make_nd_table(row, n - 1) for row in items]
    return Table.stack(rows)
  else:
    return Table.stack(items)

dimensions = struct(
    camera=0,
    frame=1,
    board=2
)

dimension_name = invert_keys(dimensions)

def map_pairs(f, table, axis=0):
  n = table._prefix[axis]
  pairs = {}

  for i in range(n):
    row_i = table._index[i]
    for j in range(i + 1, n):
      pairs[(i, j)] = f(row_i, table._index[j])

  return pairs


def common_entries(row1, row2):
  valid = np.nonzero(row1.valid  & row2.valid)
  return row1._index[valid], row2._index[valid], valid[0]

def matching_points(points, board, cam1, cam2):
  points1, points2 = points._index[cam1], points._index[cam2]
  matching = []

  for i, j in zip(points1._sequence(0), points2._sequence(0)):
    row1, row2, ids = common_entries(i, j)
    matching.append(struct(
        points1=row1.points,
        points2=row2.points,
        object_points=board.points[ids],
        ids=ids
    ))

  return transpose_structs(matching)


def pattern_overlaps(table, axis=0):
  n = table._prefix[axis]
  overlaps = np.zeros([n, n])

  for i in range(n):
    for j in range(i + 1, n):
      row_i, row_j = table._index_select(
          i, axis=axis), table._index_select(j, axis=axis)

      has_pose = (row_i.valid & row_j.valid)
      weight = np.min([row_i.num_points, row_j.num_points], axis=0)
      overlaps[i, j] = overlaps[j, i] = np.sum(
          has_pose.astype(np.float32) * weight)
  return overlaps

def rms(errors):
  return np.sqrt(np.square(errors).mean())

def estimate_transform(table, i, j, axis=0):
  table_i = table._index_select(i, axis=axis)
  table_j = table._index_select(j, axis=axis)

  assert table_i._shape == table_j._shape
  valid = (table_i.valid & table_j.valid).ravel()

  poses_i = table_i.poses.reshape(-1, 4, 4)
  poses_j = table_j.poses.reshape(-1, 4, 4)

  t, inliers = matrix.align_transforms_robust(poses_i, poses_j, valid=valid)

  err = matrix.pose_errors(t @ poses_i[valid], poses_j[valid])._map(rms)
  err_inlier = matrix.pose_errors(t @ poses_i[inliers], poses_j[inliers])._map(rms)

  info(f"Estimate transform axis={axis}, pair {(i, j)}, "
       f"inliers {inliers.sum()}/{valid.sum()}"
  )

  info(f"RMS (frobius): {err_inlier.frobius:.4f} ({err.frobius:.4f})"
       f" (deg): {err_inlier.rotation_deg:.4f} ({err.rotation_deg:.4f})"
       f" (trans): {err_inlier.translation:.4f} ({err.translation:.4f})"
  )

  info(t)
  return t

def fill_poses(pose_dict, n):
  valid_ids = sorted(pose_dict)
  pose_table = np.array([pose_dict[k] for k in valid_ids])

  values, mask = fill_sparse_tile(n, pose_table, valid_ids, np.eye(4))
  return Table.create(poses=values, valid=mask)

def count_valid(valid, axes=[]):
  dims = np.arange(valid.ndim)
  for axis in axes:
     assert axis in dims
  sum_axes = [axis for axis in dims if not axis in axes]
  return valid.sum(axis=tuple(sum_axes))

def table_info( valid, names):
  def named_counts(names, axes=[]):
    n = count_valid(valid, axes=axes)
    return dict(zip(names, n))

  camera_points = named_counts(names.camera, [0])
  board_points = named_counts(names.board, [2])

  info(f"Total: {count_valid(valid)}, cameras: {camera_points}, " 
       f"Boards: {board_points}")

  if len(names.camera) > 1 and len(names.board) > 1:
    board_points = count_valid(valid, axes=[0, 2])
    info("Camera-board matrix")
    info(board_points)


def estimate_relative_poses(table, axis=0, hop_penalty=0.9, name=None, names=None):
  name = name or dimension_name[axis]
  n = table._shape[axis]
  overlaps = pattern_overlaps(table, axis=axis)

  info(f"Overlaps by {name}:")
  info(overlaps)

  master, pairs = graph.select_pairs(overlaps, hop_penalty)
  info(f"Selected master {master} and pairs {pairs}")

  pose_dict = {master: np.eye(4)}

  for parent, child in pairs:
    t = estimate_transform(table, parent, child, axis=axis)
    pose_dict[child] = t @ pose_dict[parent]

  rel_poses = fill_poses(pose_dict, n)
  return multiply(rel_poses, np.linalg.inv(rel_poses.poses[0]))

def estimate_relative_poses_inv(table, axis=2, hop_penalty=0.9):
  return inverse(estimate_relative_poses(inverse(table), axis=axis, hop_penalty=hop_penalty))


def valid(estimates, point_table):
  valid = (np.expand_dims(estimates.camera.valid, [1, 2]) & 
    np.expand_dims(estimates.times.valid, [0, 2]) &
    np.expand_dims(estimates.board.valid, [0, 1]))

  return point_table.valid & np.expand_dims(valid, valid.ndim)


def valid_reprojection_error(points1, points2):
  errors, mask = reprojection_error(points1, points2)
  return errors[mask]


def reprojection_error(points1, points2):
  mask = points1.valid & points2.valid
  error = np.linalg.norm(points1.points - points2.points, axis=-1)
  error[~mask] = 0

  return error, mask


def inverse(table):
  return table._extend(poses=np.linalg.inv(table.poses))


def multiply(t1, t2):
  assert isinstance(t1, np.ndarray) or isinstance(t1, Table)
  assert isinstance(t2, np.ndarray) or isinstance(t2, Table)

  if isinstance(t1, np.ndarray):
    if isinstance(t2, np.ndarray):
      return t1 @ t2
    else:
      return t2._extend(poses=t1 @ t2.poses)
  else:
    if isinstance(t2, np.ndarray):
      return t1._extend(poses=t1.poses @ t2)   
    else:
      return multiply_tables(t1, t2)


def can_broadcast(shape1, shape2):
  return  len(shape1) == len(shape2) and all(
      [n1 == n2 or n1 == 1 or n2 == 1 for n1, n2 in zip(shape1, shape2)])


def broadcast_to(table1, table2):
  assert can_broadcast(table1._shape, table2._shape),\
     (f"broadcast_to: table shapes must broadcast "
      f"{table1._shape} vs {table2._shape}")

  return table1._zipWith(lambda t1, t2: np.broadcast_to(t1, t2.shape), table2)

def multiply_tables(table1, table2):
  assert can_broadcast(table1._shape, table2._shape),\
     (f"multiply_tables: table shapes must broadcast "
      f"{table1._shape} vs {table2._shape}")

  return Table.create(
    poses=table1.poses @ table2.poses,
    valid= table1.valid & table2.valid
  )

def multiply_expand(table1, dims1, table2, dims2):
  return multiply_tables(expand(table1, dims1), expand(table2, dims2))  


def expand(table, dims):
  f = partial(np.expand_dims, axis=dims)
  return table._map(f)


def expand_views(estimates):
  return multiply_expand(estimates.camera, 1, estimates.times, 0) 

def expand_boards(estimates):
  return multiply_expand(estimates.times, 1, estimates.board, 0) 


def expand_poses(estimates):
  view_poses = expand_views(estimates)
  return multiply_expand(view_poses, 2, estimates.board, [0, 1])


def mean_robust_n(pose_table, axis=0):
  def f(poses):
    if not np.any(poses.valid):
      return invalid_pose
    else:
      return valid_pose(matrix.mean_robust(poses.poses[poses.valid]))

  mean_poses = [f(poses) for poses in pose_table._sequence(axis)]
  return Table.stack(mean_poses)


def relative_between(table1, table2):
  common1, common2, valid = common_entries(table1, table2)
  if valid.size == 0:
    return invalid_pose
  else:
    t, _ = matrix.align_transforms_robust(common1.poses, common2.poses)
    return valid_pose(t)

def relative_between_inv(table1, table2):
  return inverse(relative_between(inverse(table1), inverse(table2)))


def relative_between_n(table1, table2, axis=0, inv=False):

  f = relative_between_inv if inv else relative_between 
  relative_poses = [f(poses1, poses2) for poses1, poses2 
    in zip(table1._sequence(axis), table2._sequence(axis))]

  return Table.stack(relative_poses)


def report_poses(k, init, ref):
  errs = matrix.pose_errors(init, ref)  
  for i in range(init.shape[0]):
    info(f"{k} {i}: frobius {errs.frobius[i]:.4f}, "
    f"rotation (deg): {errs.rotation_deg[i]:.4f}, "
    f"translation: {errs.translation[i]:.4f}")

def estimate_camera_poses(ws):
  master_cam = 0
  boards = ws.boards
  # objp = boards[0].adjusted_points

  # select one board for each camera
  num_cam = ws.point_table['points'].shape[0]
  num_img = ws.point_table['points'].shape[1]
  num_board = ws.point_table['points'].shape[2]
  selected_board = 0
  board_selection_matrix = np.zeros((num_cam))

  for cam in range(0, num_cam):
    board_selection_list = []
    board_selection_dict = {}
    for im in range(0, num_img):
      for b in range(0, num_board):
        length = len(ws.detected_points[cam][im][b]['ids'])
        # board_selection_matrix[cam][im][b] = length
        if length > 0:
          board_selection_list.append(b)
          if b in board_selection_dict:
            board_selection_dict[b] = board_selection_dict[b] + length
          else:
            board_selection_dict[b] = length

    sorted_dict = sorted(board_selection_dict.items(), reverse=False)
    # count = Counter(board_selection_list).most_common()
    # board_selection_matrix[cam] = max(board_selection_list,key=board_selection_list.count)
    board_selection_matrix[cam] = next(iter(board_selection_dict))

  skip_board = []
  for b in range (0, num_board):
    if b not in board_selection_matrix:
      skip_board.append(b)

  #### Calibration ####
  # termination criteria
  criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
  # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
  # objp = np.zeros((11 * 8, 3), np.float32)
  # objp[:, :2] = np.mgrid[0:11, 0:8].T.reshape(-1, 2)
  objp = boards[0].adjusted_points
  # Arrays to store object points and image points from all the images.
  objpoints = []  # 3d point in real world space
  imgpoints = []  # 2d points in image plane.

  cam_board_transformation = {}
  ## count skip img
  skip_img = []
  for cam in range(0, num_cam):
    selected_board = int(board_selection_matrix[cam])
    for im in range(0, num_img):
      for b in skip_board:
        ws.pose_table['valid'][cam][im][b] = ws.pose_table['valid'][cam][im][b] * 0
        ws.point_table['valid'][cam][im][b] = ws.point_table['valid'][cam][im][b] * 0
      if len(ws.detected_points[cam][im][selected_board]['ids']) < 8:
        skip_img.append(im)
        for c in range(0, num_cam):
          ws.pose_table['valid'][c][im] = ws.pose_table['valid'][c][im]*0
          ws.point_table['valid'][c][im] = ws.point_table['valid'][c][im]*0

  obj_points_dict = {}
  img_points_dict = {}
  img_obj_point_dict = {}
  for cam in range(0, num_cam):
    cam_board_transformation[cam] = {}
    selected_board = int(board_selection_matrix[cam])
    objpoints = []  # 3d point in real world space
    imgpoints = []
    img_obj_point_dict[cam] = {}
    for im in range(0, num_img):
      if im not in skip_img:
        img_obj_point_dict[cam][im] = {}
        if len(ws.detected_points[cam][im][selected_board]['ids']) > 4:
          # working version
          c = ws.detected_points[cam][im][selected_board]['corners']
          imgpoints.append(ws.detected_points[cam][im][selected_board]['corners'])
          objpoints.append(np.array([list(objp[i]) for i in ws.detected_points[cam][im][selected_board]['ids']]))

          j = 0
          for i in ws.detected_points[cam][im][selected_board]['ids']:
            img_obj_point_dict[cam][im][i] = {}
            img_obj_point_dict[cam][im][i]['corners'] = ws.detected_points[cam][im][selected_board]['corners'][j]
            img_obj_point_dict[cam][im][i]['obj_point'] = objp[i]
            j += 1

    obj_points_dict[cam] = objpoints
    img_points_dict[cam] = imgpoints
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (5472, 3648), None, None)
    # ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (1920, 1080), None, None)

    Rotation = np.zeros((len(rvecs), 3, 3))
    for r in range(0, len(rvecs)):
      rot = R.from_rotvec(rvecs[r].T)
      Rotation[r] = rot.as_matrix()

    cam_board_transformation[cam]['rvecs'] = Rotation
    Translation = np.zeros((len(tvecs), 3))
    for t in range(0, len(tvecs)):
      Translation[t] = tvecs[t].T
    cam_board_transformation[cam]["tvecs"] = Translation
    # cam_board_transformation[cam]['mtx'] = mtx
    # cam_board_transformation[cam]['dist'] = dist

  ##### hand eye
  cam_transformation_dict = {}
  cam_pose_dict = {}
  board_pose_dict = {}
  for c1 in range(0, num_cam):
    if c1 == master_cam:
      b1 = board_selection_matrix[c1]
      r = np.eye(3, dtype='float64')
      t = np.array([[0, 0, 0]], dtype='float64').T
      h = np.hstack([r, t])
      v = np.array([[0, 0, 0, 1]], dtype='float64')
      board_pose_dict[int(b1)] = np.concatenate((h,v), axis=0)
      for c2 in range(0, num_cam):
        b2 = board_selection_matrix[c2]

        base_world_r, base_world_t, c1_c2_r, c1_c2_t = cv2.calibrateRobotWorldHandEye(
          cam_board_transformation[c1]['rvecs'],
          cam_board_transformation[c1]['tvecs'],
          cam_board_transformation[c2]['rvecs'],
          cam_board_transformation[c2]['tvecs'])
        if b2 != b1:
          v = np.array([[0, 0, 0, 1]], dtype='float64')
          h = np.hstack([base_world_r, base_world_t])
          rt = np.concatenate((h, v), axis=0)
          board_pose_dict[int(b2)] = rt
        text = 'cam' + str(c1) + '_to_' + 'cam' + str(c2)
        cam_transformation_dict[text] = {}
        cam_transformation_dict[text]['R'] = c1_c2_r
        cam_transformation_dict[text]['T'] = c1_c2_t
        v = np.array([[0,0,0,1]], dtype='float64')
        h = np.hstack([c1_c2_r, c1_c2_t])
        rt = np.concatenate((h,v), axis=0)
        # rt = rtvec.join(c1_c2_r, c1_c2_t.T)
        cam_pose_dict[c2] = rt
        # print(cam_pose)

  rel_cam_poses = fill_poses(cam_pose_dict, num_cam)
  rel_board_poses = fill_poses(board_pose_dict, num_board)


  # print('end', cam_pose)
  return rel_cam_poses, rel_board_poses, board_selection_matrix


def initialise_poses_old(pose_table, camera_poses=None):
    # Find relative transforms between cameras and rig poses
  camera = estimate_relative_poses(pose_table, axis=0)

  if camera_poses is not None:
    info("Camera initialisation vs. supplied calibration")
    report_poses("camera", camera_poses, camera.poses)
    camera = Table.create(
      poses=camera_poses, 
      valid=np.ones(camera_poses.shape[0], dtype=np.bool)
    )
    
  board  = estimate_relative_poses_inv(pose_table, axis=2)

  # solve for the rig transforms cam @ rig @ board = pose
  # first take inverse of both sides by board pose  
  # cam @ rig = board_relative = pose @ board^-1
  board_relative = multiply_tables(pose_table, expand(inverse(board), [0, 1]) )
  
  # solve for unknown rig 
  expanded = broadcast_to(expand(camera, [1, 2]), board_relative)
  times = relative_between_n(expanded, board_relative, axis=1, inv=True)

  return struct(times=times, camera=camera, board=board)

def initialise_poses(ws, camera_poses=None):

  # Find relative transforms between cameras and rig poses
  # camera = estimate_relative_poses(ws.pose_table, axis=0)
  camera, board, board_selection_matrix = estimate_camera_poses(ws)
  print(camera)

  if camera_poses is not None:
    info("Camera initialisation vs. supplied calibration")
    report_poses("camera", camera_poses, camera.poses)
    camera = Table.create(
      poses=camera_poses,
      valid=np.ones(camera_poses.shape[0], dtype=np.bool)
    )

  board2 = estimate_relative_poses_inv(ws.pose_table, axis=2)

  # solve for the rig transforms cam @ rig @ board = pose
  # first take inverse of both sides by board pose
  # cam @ rig = board_relative = pose @ board^-1
  board_relative = multiply_tables(ws.pose_table, expand(inverse(board), [0, 1]))

  # solve for unknown rig
  expanded = broadcast_to(expand(camera, [1, 2]), board_relative)
  times = relative_between_n(expanded, board_relative, axis=1, inv=True)

  return struct(times=times, camera=camera, board=board, selected_boards=board_selection_matrix)


def stereo_calibrate(points, board, cameras, i, j, **kwargs):
  matching = matching_points(points, board, i, j)
  return stereo_calibrate((cameras[i], cameras[j]), matching, **kwargs)


def stack_boards(boards):
  padded_points = max([board.num_points for board in boards])

  def pad_points(board):
    points = board.adjusted_points.astype(np.float64)
    return struct(
      points=np.pad(points, [(0, padded_points - points.shape[0]), (0, 0)]),
      valid = np.arange(padded_points) < board.num_points
    )
  return Table.stack([pad_points(board) for board in boards]) 

def expand_dims(table, axis):
  return table._map(partial(np.expand_dims, axis=axis))


def transform_points(pose_table, board_points):
  assert can_broadcast(pose_table._shape, board_points._shape)
  return Table.create(
    points = matrix.transform_homog(t = pose_table.poses, points=board_points.points),
    valid = pose_table.valid & board_points.valid
  )