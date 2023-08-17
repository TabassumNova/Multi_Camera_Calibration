# from multical.motion.static_frames import project_points
import numpy as np
from structs.struct import struct, subset
from src.multical import tables
from src.multical.io.export_calib import export_poses
from src.multical.optimization.parameters import IndexMapper, Parameters
from cached_property import cached_property
from src.multical.motion.motion_model import MotionModel
from multical.optimization.pose_set import PoseSet
from src.multical.transform import rtvec as rtvec
from structs.numpy import Table, shape
from structs.struct import struct, subset
from src.multical import tables
from numpy.linalg import inv
import cv2
from scipy.spatial.transform import Rotation as R
from copy import copy
from structs.numpy import shape_info, struct, Table, shape
from src.multical.transform import matrix as matrix
'''
Renamed on 15th August
New one hand_eye.py
'''
def project_cameras(cameras, local_points):
  image_points = [camera.project(p) for camera, p in
      zip(cameras, local_points.points)]

  return Table.create(points=np.stack(image_points), valid=local_points.valid)

def project_points(pose_table, cameras, camera_poses, world_points):
  pose_estimates = struct(camera = camera_poses,  times=pose_table)
  pose_table = tables.expand_views(pose_estimates)

  points = tables.transform_points(
    tables.expand_dims(pose_table, (2, 3)),
    tables.expand_dims(world_points, (0, 1))
  )

  return project_cameras(cameras, points)

class HandEye(Parameters, MotionModel):
# class HandEye(PoseSet, MotionModel):
  """
  Optimises the world to camera rig transform (frames) constrained to a 
  'robot world' hand-eye calibration form where world_wrt_base and gripper_wrt_camera are 
  optimised and a set of base_gripper transforms are provided (and constant).

  world_wrt_camera = gripper_wrt_camera @ base_wrt_gripper @ world_wrt_base
  """
  def __init__(self, base_wrt_camera, gripper_wrt_world, times, workspace, names):
    self.workspace = workspace
    self.names = names
    self.times = times
    self.base_wrt_camera = base_wrt_camera
    self.gripper_wrt_world = gripper_wrt_world

    # self.base_wrt_gripper = base_wrt_gripper
    # n = base_wrt_gripper._shape[0]
    #
    # self.names = names or [str(i) for i in range(n)]
    #
    # self.world_wrt_base = world_wrt_base
    # self.gripper_wrt_camera = gripper_wrt_camera
    # self.board_selection_matrix = board_selection_matrix
    # self.times = times


  @property
  def size(self):
    return self.poses.shape[0]


  def project(self, cameras, camera_poses, world_points, estimates=None):
      masterCam = 0
      masterBoard = 0
      num_cam = self.workspace.sizes.camera
      num_board = self.workspace.sizes.board
      num_img = self.workspace.sizes.image
      imagePoints = np.zeros((num_cam, num_img, num_board, world_points['points'].shape[1], 2))
      imagePoints_valid = np.zeros((num_cam, num_img, num_board, world_points['points'].shape[1]), dtype=bool)
      for cam_id, cam_v in enumerate(self.workspace.pose_table.poses):
          for img_id, img_v in enumerate(self.workspace.pose_table.poses[cam_id]):
              for board_id, board_v in enumerate(self.workspace.pose_table.poses[cam_id][img_id]):
                  if self.workspace.pose_table.valid[masterCam][img_id][board_id]:
                      t = matrix.transform(self.workspace.pose_table.poses[masterCam][img_id][board_id],
                                           np.linalg.inv(self.base_wrt_camera[cam_id]))
                  else:
                      valid_board = np.flatnonzero(self.workspace.pose_table.valid[masterCam][img_id])
                      t0 = matrix.relative_to(self.gripper_wrt_world[board_id], self.gripper_wrt_world[valid_board[0]])
                      t1 = matrix.transform(t0, self.workspace.pose_table.poses[masterCam][img_id][valid_board[0]])
                      t = matrix.transform(t1, np.linalg.inv(self.base_wrt_camera[cam_id]))
                  t0 = self.workspace.pose_table.poses[cam_id][img_id][board_id]
                  err = rtvec.from_matrix(t0) - rtvec.from_matrix(t)
                  # objectPoints = (world_points['points'][board_id])
                  objectPoints = np.array([self.workspace.boards[board_id].adjusted_points])
                  cameraMatrix = (cameras[cam_id].intrinsic)
                  distortion = (cameras[cam_id].dist)
                  rvec, tvec = rtvec.split(rtvec.from_matrix(t))
                  imP, _ = cv2.projectPoints(np.copy(objectPoints), rvec, tvec, cameraMatrix, distortion)
                  cornerPoints = self.workspace.point_table.points[cam_id][img_id][board_id]
                  error = np.linalg.norm(cornerPoints - imP.reshape([-1, 2]), axis=-1)
                  imagePoints[cam_id][img_id][board_id] = imP.reshape([-1, 2])
                  imagePoints_valid[cam_id][img_id][board_id] = True

      return Table.create(points=imagePoints, valid=imagePoints_valid)


  @cached_property
  def valid(self):
    return self.pose_table.valid
  #
  @cached_property
  def pose_table(self):
    return self.times

  @property
  def frame_poses(self):
    return self.pose_table

  def __getitem__(self, k):
    if isinstance(k, str):
      if k not in self.names:
        raise KeyError(f"pose {k} not found in {self.names}")

      return self.poses[self.names.index(k)]
    else:
      return self.poses[k]
  #
  # def relative(self, src, dest):
  #   return self[dest] @ np.linalg.inv(self[src])
  #
  @cached_property
  def poses(self):
    return self.pose_table.poses

  def pre_transform(self, t):
    return self.copy(gripper_wrt_world = t @ self.gripper_wrt_world)

  def post_transform(self, t):
    return self.copy(base_wrt_camera = self.base_wrt_camera @ t)

  @cached_property
  def params(self):
    return struct(
      base_wrt_camera = rtvec.from_matrix(self.base_wrt_camera),
      gripper_wrt_world = rtvec.from_matrix(self.gripper_wrt_world)
    )

  def with_params(self, params):
    return self.copy(
      base_wrt_camera = rtvec.to_matrix(params.base_wrt_camera),
      gripper_wrt_world = rtvec.to_matrix(params.gripper_wrt_world)
    )
  #
  #
  # def sparsity(self, index_mapper : IndexMapper, axis : int):
  #   return index_mapper.all_points(rtvec.size * 2)


  # def export(self):
  #   return struct(
  #     base_wrt_gripper = export_poses(self.base_wrt_gripper, self.names),
  #     world_wrt_base = self.world_wrt_base.tolist(),
  #     gripper_wrt_camera = self.gripper_wrt_camera.tolist()
  #   )

  def __getstate__(self):
    attrs = ['base_wrt_camera', 'gripper_wrt_world', 'times', 'workspace', 'names']
    return subset(self.__dict__, attrs)

  def copy(self, **k):
    """Copy object and change some attribute (no mutation)"""
    d = self.__getstate__()
    d.update(k)
    return self.__class__(**d)

  @staticmethod
  def init(workspace, pose_init, names=None):
    # p = pose
    # return HandEye(workspace, pose_init, names)
    return HandEye(pose_init.camera.poses, pose_init.board.poses, pose_init.times, workspace, names)