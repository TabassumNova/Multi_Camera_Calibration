# from multical.motion.static_frames import project_points
import numpy as np
from structs.struct import struct, subset
from multical import tables
from multical.io.export_calib import export_poses
from multical.optimization.parameters import IndexMapper, Parameters
from cached_property import cached_property
from multical.motion.motion_model import MotionModel
from multical.optimization.pose_set import PoseSet
from multical.transform import rtvec as transform_vec
from structs.numpy import Table, shape
from structs.struct import struct, subset
from multical import tables
from numpy.linalg import inv
import cv2
from scipy.spatial.transform import Rotation as R
from copy import copy
from structs.numpy import shape_info, struct, Table, shape

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
  def __init__(self, base_wrt_gripper, world_wrt_base, gripper_wrt_camera, board_selection_matrix, times,  names=None):
    self.base_wrt_gripper = base_wrt_gripper
    n = base_wrt_gripper._shape[0]

    self.names = names or [str(i) for i in range(n)]

    self.world_wrt_base = world_wrt_base
    self.gripper_wrt_camera = gripper_wrt_camera
    self.board_selection_matrix = board_selection_matrix
    self.times = times


  @property
  def size(self):
    return self.poses.shape[0]

  def project_old(self, cameras, camera_poses, world_points, estimates=None):
    board_poses = self.world_wrt_base
    board_points = world_points
    base_wrt_camera = tables.multiply(self.gripper_wrt_camera, self.base_wrt_gripper)
    pose_table = tables.multiply(base_wrt_camera, self.world_wrt_base)
    return project_points(pose_table, cameras, camera_poses, board_points)

  # changed from previous version
  # def project(self, cameras, camera_poses, world_points, point_table, estimates=None):
  def project(self, cameras, camera_poses, world_points,estimates=None):
    board_pose = self.world_wrt_base
    cam_pose = self.gripper_wrt_camera
    # print('world_base_board: ', board_pose)
    # print('grip_cam_cam: ', cam_pose)
    # object_points = world_points

    ## projection calc
    num_cam = cam_pose.shape[0]
    num_board = board_pose.shape[0]
    master_cam = 0
    num_img = self.base_wrt_gripper['poses'].shape[1]
    error_dict = {}
    imagePoints = np.zeros((num_cam, num_img, num_board, world_points['points'].shape[1], 2))
    imagePoints_valid = np.zeros((num_cam, num_img, num_board, world_points['points'].shape[1]), dtype=bool)
    for c1 in range (0, num_cam):
        b1 = int(self.board_selection_matrix[c1])
        if c1 == master_cam:
            error_dict[c1] = {}
            for c2 in range (0, num_cam):
                b2 = int(self.board_selection_matrix[c2])
                error_dict[c1][c2] = {}
                for im in range (0, num_img):
                    # a = inv(cam_pose['poses'][c2])
                    # b = (self.base_wrt_gripper['poses'][c1][im][b1])
                    # c = (board_pose['poses'][b2])
                    transformation = inv(cam_pose[c2]) @ (self.base_wrt_gripper['poses'][c1][im][b1]) @ (board_pose[b2])
                    objectPoints = (world_points['points'][b2])
                    cameraMatrix = (cameras[c2].intrinsic)
                    distortion = (cameras[c2].dist)
                    rmatrix = transformation[0:3, 0:3]
                    tmatrix = transformation[0:3, 3]
                    rvec = (R.from_matrix([rmatrix]).as_rotvec())
                    tvec = (tmatrix.T)
                    imP, _ = cv2.projectPoints(np.copy(objectPoints), rvec, tvec, cameraMatrix, distortion)
                    imagePoints[c2][im][b2] = imP.reshape([-1, 2])
                    imagePoints_valid[c2][im][b2] = True

                    # error = imagePoints.reshape([-1, 2]) - ws.detected_points[c2][im][b2]['corners']
                    # error_dict[c1][c2][im] = error
    ##

    ## broadcast
    # ex1 = np.expand_dims(board_pose['poses'], axis=(0,1))
    # board_b1 = np.broadcast_to(ex1, self.base_wrt_gripper['poses'].shape)
    # ex2 = np.expand_dims(cam_pose['poses'], axis=(1,2))
    # cam_b2 = np.broadcast_to(ex2, self.base_wrt_gripper['poses'].shape)
    # ##
    # transformation = inv(cam_b2) @ self.base_wrt_gripper['poses'] @ board_b1
    # base_wrt_camera = tables.multiply(self.gripper_wrt_camera, self.base_wrt_gripper)
    # pose_table = tables.multiply(base_wrt_camera, self.world_wrt_base)
    return Table.create(points=imagePoints, valid=imagePoints_valid)

  @cached_property
  def valid(self):
    return self.pose_table.valid
  #
  @cached_property
  def pose_table(self):
    # base_wrt_camera = tables.multiply(self.gripper_wrt_camera, self.base_wrt_gripper)
    return self.times

  @property
  def frame_poses(self):
    return self.pose_table
  #
  # def pre_transform(self, t):
  #   return self.copy(gripper_wrt_camera = t @ self.gripper_wrt_camera)
  #
  # def post_transform(self, t):
  #   return self.copy(world_wrt_base = self.world_wrt_base @ t)
  #
  #
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
    return self.copy(gripper_wrt_camera = t @ self.gripper_wrt_camera)

  def post_transform(self, t):
    return self.copy(world_wrt_base = self.world_wrt_base @ t)

  @cached_property
  def params(self):
    return struct(
      world_wrt_base = transform_vec.from_matrix(self.world_wrt_base),
      gripper_wrt_camera = transform_vec.from_matrix(self.gripper_wrt_camera)
    )

  def with_params(self, params):
    return self.copy(
      world_wrt_base = transform_vec.to_matrix(params.world_wrt_base),
      gripper_wrt_camera = transform_vec.to_matrix(params.gripper_wrt_camera)
    )
  #
  #
  def sparsity(self, index_mapper : IndexMapper, axis : int):
    return index_mapper.all_points(transform_vec.size * 2)

  def export(self):
    return struct(
      base_wrt_gripper = export_poses(self.base_wrt_gripper, self.names),
      world_wrt_base = self.world_wrt_base.tolist(),
      gripper_wrt_camera = self.gripper_wrt_camera.tolist()
    )

  def __getstate__(self):
    attrs = ['base_wrt_gripper', 'world_wrt_base', 'gripper_wrt_camera', 'board_selection_matrix', 'times', 'names']
    return subset(self.__dict__, attrs)

  def copy(self, **k):
    """Copy object and change some attribute (no mutation)"""
    d = self.__getstate__()
    d.update(k)
    return self.__class__(**d)

  @staticmethod
  def init(all_pose, pose_init, names=None):
    # p = pose
    return HandEye(all_pose, pose_init.board['poses'], pose_init.camera['poses'], pose_init.selected_boards, pose_init.times, names)