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
    workspace_pickle = os.path.join(path, "workspace.pkl")
    with open(workspace_pickle, "wb") as file:
        pickle.dump(workspace, file)
    # pickle.dump(workspace, open("workspace.pkl", "wb"))
    json_dict = {}
    # json_dict['cameras'] = workspace.cameras
    json_dict['num_points'] = workspace.pose_table.num_points.tolist()
    json_dict['reprojection_error'] = workspace.pose_table.reprojection_error.tolist()
    json_dict['view_angles'] = workspace.pose_table.view_angles.tolist()
    json_dict["names"] = to_dicts(workspace.names)
    json_dict["pose_table"] = workspace.pose_table.poses.tolist()
    json_object = json.dumps(json_dict, indent=4)
    # Writing to sample.json
    json_path = os.path.join(path, "workspace.json")
    with open(json_path, "w") as outfile:
        outfile.write(json_object)
    pass


if __name__ == '__main__':

    base_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset\V33"
    workspace_path = os.path.join(base_path, 'workspace.json')
    pathO = args.PathOpts(image_path=base_path)
    cam = args.CameraOpts()
    runt = args.RuntimeOpts(show_all_poses=True)
    # opt = args.OptimizerOpts()
    opt = args.OptimizerOpts(outlier_threshold=1.2, fix_intrinsic=False)

    c = calibrate.Calibrate(paths=pathO, camera=cam, runtime=runt, optimizer=opt)
    workspace = c.execute_new()
    workspace.pose_table = make_pose_table(workspace.point_table, workspace.boards,
                                                workspace.cameras)
    workspace.export()
    workspace.dump()
    export_workspace(workspace, base_path)

    pass
