from structs.struct import map_none
from src.multical.io.import_calib import load_calibration
from src.multical.motion.static_frames import StaticFrames
from src.multical.motion.rolling_frames import RollingFrames
from src.multical.motion.hand_eye import HandEye
from src.multical.workspace import Workspace

from .arguments import *


def get_motion_model(motion_model):
    if motion_model == "rolling":
        return RollingFrames
    elif motion_model == "static":
        return StaticFrames
    elif motion_model == "hand_eye":
        return HandEye
    else:
        assert False, f"unknown motion model {motion_model}, (static|rolling)"


def initialise_with_images(ws : Workspace, boards, camera_images, 
  camera_opts : CameraOpts = CameraOpts(), runtime : RuntimeOpts = RuntimeOpts()):


    ws.add_camera_images(camera_images, j=runtime.num_threads)
    ws.detect_boards(boards, load_cache=not runtime.no_cache, j=runtime.num_threads)

    calib = map_none(load_calibration, camera_opts.calibration)

    if calib is not None:
      ws.set_calibration(calib.cameras)
    else:
      ws.calibrate_single(camera_opts.distortion_model,
          reprojection_error_limit=camera_opts.intrinsic_error_limit,
          fix_aspect=camera_opts.fix_aspect,
          has_skew=camera_opts.allow_skew, 
          max_images=camera_opts.limit_intrinsic,
          isFisheye=camera_opts.isFisheye)


    if camera_opts.motion_model == 'hand_eye':
        ws.initialise_HandEye(
            # motion=camera_opts.motion_model,
            motion_model=get_motion_model(camera_opts.motion_model),
            camera_poses=calib.camera_poses if calib is not None else None
        )
    elif camera_opts.motion_model == 'calibrate_board':
        ws.initialise_board(runtime)
        pass
    else:
        ws.initialise_poses(
            motion_model=get_motion_model(camera_opts.motion_model),
            camera_poses=calib.camera_poses if calib is not None else None,
            method = runtime.pose_estimation,
            show_all_poses = runtime.show_all_poses
          )
    return ws


def optimize(ws : Workspace, opt : OptimizerOpts = OptimizerOpts()):

  ws.calibrate("calibration", loss=opt.loss,
    boards=opt.adjust_board,
    cameras=not opt.fix_intrinsic,
    camera_poses=not opt.fix_camera_poses,
    board_poses=not opt.fix_board_poses,
    motion=not opt.fix_motion,
    auto_scale=opt.auto_scale, 
    outlier_threshold=opt.outlier_threshold, quantile=opt.outlier_quantile, adjust_outliers=opt.adjust_outliers)

  return ws