import os.path

import src.multical.app.calibrate as calibrate
import src.multical.config.arguments as args
import src.multical.config.workspace as workspace
from src.multical.tables import *
import json
from structs.struct import struct, to_dicts, transpose_lists
from src.multical.workspace import *
import src.multical.workspace as ws

base_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset\V38"

def export_workspace(workspace, path):
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

def main1(datasetPath):
    pathO = args.PathOpts(image_path=datasetPath)
    cam = args.CameraOpts(intrinsic_error_limit=1)
    # pose_estimation_method = "solvePnPRansac"
    pose_estimation_method = "solvePnPGeneric"
    runt = args.RuntimeOpts(pose_estimation=pose_estimation_method)
    opt = args.OptimizerOpts(outlier_threshold=1.2, fix_intrinsic=True, adjust_outliers=False)
    c = calibrate.Calibrate(paths=pathO, camera=cam, runtime=runt, optimizer=opt)
    workspace = c.execute_board()
    # workspace.point_table.valid = workspace.pose_table.inliers
    return workspace

if __name__ == '__main__':

    ws = main1(base_path)
    export_workspace(ws, base_path)
    pass
