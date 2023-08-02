import os.path

import src.multical.app.calibrate as calibrate
import src.multical.config.arguments as args
import src.multical.config.workspace as workspace
from src.multical.tables import *
import json
from structs.struct import struct, to_dicts, transpose_lists
from src.multical.workspace import *
import src.multical.workspace as ws


def export_workspace(workspace, path):
    # workspace.__delattr__("calibrations")
    # workspace.__delattr__("detections")
    # # workspace.__delattr__("detections_file")
    # # workspace.__delattr__("filenames")
    # # workspace.__delattr__("initialisation")
    # # workspace.__delattr__("latest_calibration")
    # workspace.__delattr__("log_handler")
    boards = [workspace.boards[0].adjusted_points,
              workspace.boards[1].adjusted_points,
              workspace.boards[2].adjusted_points,
              workspace.boards[3].adjusted_points,
              workspace.boards[4].adjusted_points]
    new_workspace = struct(boards = boards, detected_points = workspace.detected_points,
                           pose_table = workspace.pose_table)
    '''
    ########### For json dict ##############
    new_workspace_dict = {}
    new_workspace_dict['boards'] = [b.tolist() for b in boards]
    detected_points = []
    cam_list = []
    for cam in range(len(workspace.names.camera)):
        image_list = []
        for img in range(len(workspace.names.image)):
            board_list = []
            for board in range(len(workspace.names.board)):
                board_list.append(workspace.detected_points[cam][img][board].corners.tolist())
            image_list.append(board_list)
        cam_list.append(image_list)
    new_workspace_dict['detected_points'] = cam_list
    new_workspace_dict['pose_table'] = workspace.pose_table.poses.tolist()
    json_object = json.dumps(new_workspace_dict, indent=4)
    # Writing to sample.json
    json_path = os.path.join(path, "workspace.json")
    with open(json_path, "w") as outfile:
        outfile.write(json_object)
    '''

    workspace_pickle = os.path.join(path, "workspace.pkl")
    with open(workspace_pickle, "wb") as file:
        pickle.dump(workspace, file)
    # # pickle.dump(workspace, open("workspace.pkl", "wb"))
    # json_dict = {}
    # # json_dict['cameras'] = workspace.cameras
    # json_dict['num_points'] = workspace.pose_table.num_points.tolist()
    # json_dict['reprojection_error'] = workspace.pose_table.reprojection_error.tolist()
    # json_dict['view_angles'] = workspace.pose_table.view_angles.tolist()
    # json_dict["names"] = to_dicts(workspace.names)
    # json_dict["pose_table"] = workspace.pose_table.poses.tolist()
    # json_object = json.dumps(json_dict, indent=4)
    # # Writing to sample.json
    # json_path = os.path.join(path, "workspace.json")
    # with open(json_path, "w") as outfile:
    #     outfile.write(json_object)
    # pass


if __name__ == '__main__':

    base_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset\V30"
    workspace_path = os.path.join(base_path, 'workspace.json')
    pathO = args.PathOpts(image_path=base_path)
    cam = args.CameraOpts(motion_model="calibrate_board", calibration="D:\MY_DRIVE_N\Masters_thesis\Dataset\V30\calibration.json")
    pose_estimation_method = "solvePnPGeneric"
    runt = args.RuntimeOpts(pose_estimation=pose_estimation_method)
    opt = args.OptimizerOpts(outlier_threshold=1.2, fix_intrinsic=True)
    c = calibrate.Calibrate(paths=pathO, camera=cam, runtime=runt, optimizer=opt)
    workspace = c.execute_board()
    workspace.dump()
    export_workspace(workspace, base_path)

    pass
