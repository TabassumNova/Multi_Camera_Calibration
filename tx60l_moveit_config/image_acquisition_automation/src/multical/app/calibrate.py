
from src.multical.io.logging import setup_logging
from .vis import visualize_ws

from structs.struct import struct, map_none, to_structs
import numpy as np

from src.multical.config import *
from dataclasses import dataclass

@dataclass
class Calibrate:
    """Run camera calibration"""
    paths  : PathOpts 
    camera  : CameraOpts
    runtime    : RuntimeOpts
    optimizer  : OptimizerOpts 
    vis : bool = False        # Visualize result after calibration

    def execute(self):
        calibrate(self)

    def execute_new(self):
        ws = detection_new(self)
        return ws

    def execute_board(self):
        calibrate_board(self)


def calibrate(args): 
  np.set_printoptions(precision=4, suppress=True)

  # Use image path if not explicity specified
  output_path = args.paths.image_path or args.paths.output_path 

  ws = workspace.Workspace(output_path, args.paths.name)
  
  setup_logging(args.runtime.log_level, [ws.log_handler], log_file=path.join(output_path, f"{args.paths.name}.txt"))

  boards = find_board_config(args.paths.image_path, board_file=args.paths.boards)
  camera_images = find_camera_images(args.paths.image_path, 
    args.paths.cameras, args.paths.camera_pattern, limit=args.paths.limit_images)
  # print(ws)
  initialise_with_images(ws, boards, camera_images, args.camera, args.runtime)
  optimize(ws, args.optimizer)

  ws.export()
  ws.dump()

  if args.vis:
    visualize_ws(ws)

def calibrate_board(args):
    np.set_printoptions(precision=4, suppress=True)

    # Use image path if not explicity specified
    output_path = args.paths.image_path or args.paths.output_path

    ws = workspace.Workspace(output_path, args.paths.name)

    setup_logging(args.runtime.log_level, [ws.log_handler], log_file=path.join(output_path, f"{args.paths.name}.txt"))

    boards = find_board_config(args.paths.image_path, board_file=args.paths.boards)
    camera_images = find_camera_images(args.paths.image_path,
                                       args.paths.cameras, args.paths.camera_pattern, limit=args.paths.limit_images)
    # print(ws)
    initialise_with_images(ws, boards, camera_images, args.camera, args.runtime)

    ws.export()
    ws.dump()

def detection_new(args):
    np.set_printoptions(precision=4, suppress=True)

    # Use image path if not explicity specified
    output_path = args.paths.image_path or args.paths.output_path

    ws = workspace.Workspace(output_path, args.paths.name)

    setup_logging(args.runtime.log_level, [ws.log_handler], log_file=path.join(output_path, f"{args.paths.name}.txt"))

    boards = find_board_config(args.paths.image_path, board_file=args.paths.boards)
    camera_images = find_camera_images(args.paths.image_path,
                                       args.paths.cameras, args.paths.camera_pattern, limit=args.paths.limit_images)
    # ws.add_camera_images(camera_images, j=args.runtime.num_threads)
    # ws.detect_boards(boards, load_cache=not args.runtime.no_cache, j=args.runtime.num_threads)
    initialise_with_images(ws, boards, camera_images, args.camera, args.runtime)
    # calib = map_none(load_calibration, args.camera.calibration)
    # ws.set_calibration(calib.cameras)

    return ws


if __name__ == '__main__':
  run_with(Calibrate)
