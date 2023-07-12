import src.multical.app.calibrate as calibrate
import src.multical.config.arguments as args
import src.multical.config.workspace as workspace
from src.multical.tables import *
import json
from structs.struct import struct, to_dicts, transpose_lists

workspace_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset/bundle_adjustment_test/workspace.json"

def export_workspace(workspace, path):
    json_dict = {}
    # json_dict['cameras'] = workspace.cameras
    json_dict['num_points'] = workspace.pose_table.num_points.tolist()
    json_dict['reprojection_error'] = workspace.pose_table.reprojection_error.tolist()
    json_dict['view_angles'] = workspace.pose_table.view_angles.tolist()
    json_dict["names"] = to_dicts(workspace.names)
    # json_dict["pose_table"] = workspace.pose_table
    json_object = json.dumps(json_dict, indent=4)
    # Writing to sample.json
    with open(path, "w") as outfile:
        outfile.write(json_object)
    pass

if __name__ == '__main__':

    # pathO = args.PathOpts(image_path="/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/view_plan/viewPlan2/data/")
    # cam = args.CameraOpts(motion_model='hand_eye', calibration='/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/view_plan/viewPlan2/all_camera_intrinsic_V24.json')
    pathO = args.PathOpts(image_path="D:\MY_DRIVE_N\Masters_thesis\Dataset/bundle_adjustment_test")
    cam = args.CameraOpts()
    runt = args.RuntimeOpts(show_all_poses=True)
    # opt = args.OptimizerOpts()
    opt = args.OptimizerOpts(outlier_threshold=1.2, fix_intrinsic=False)

    c = calibrate.Calibrate(paths=pathO, camera=cam, runtime=runt, optimizer=opt)
    workspace = c.execute_new()
    workspace.pose_table = make_pose_table(workspace.point_table, workspace.boards,
                                                workspace.cameras)
    export_workspace(workspace, workspace_path)

    pass
