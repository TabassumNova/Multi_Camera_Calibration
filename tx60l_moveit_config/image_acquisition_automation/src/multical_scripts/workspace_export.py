import src.multical.app.calibrate as calibrate
import src.multical.config.arguments as args
import src.multical.config.workspace as workspace
from src.multical.tables import *

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
    pass
