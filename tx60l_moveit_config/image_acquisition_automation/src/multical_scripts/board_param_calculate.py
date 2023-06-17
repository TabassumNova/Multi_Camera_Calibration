import src.multical.app.calibrate as calibrate
import src.multical.config.arguments as args
import src.multical.config.workspace as workspace

if __name__ == '__main__':

    pathO = args.PathOpts(image_path="D:\MY_DRIVE_N\Masters_thesis\Dataset/board_param/board_param_final3")
    cam = args.CameraOpts(motion_model="calibrate_board", calibration="D:\MY_DRIVE_N\Masters_thesis\Dataset/board_param/board_param_final3/all_camera_intrinsic.json")
    # pathO = args.PathOpts(image_path="D:\MY_DRIVE_N\Masters_thesis\Dataset\V10_debug\data/train")
    # cam = args.CameraOpts()
    runt = args.RuntimeOpts()
    # opt = args.OptimizerOpts()
    opt = args.OptimizerOpts(outlier_threshold=1.2, fix_intrinsic=True)

    c = calibrate.Calibrate(paths=pathO, camera=cam, runtime=runt, optimizer=opt)
    c.execute_board()