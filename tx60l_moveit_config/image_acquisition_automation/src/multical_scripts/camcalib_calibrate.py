import src.multical.app.calibrate as calibrate
import src.multical.config.arguments as args
import src.multical.config.workspace as workspace

if __name__ == '__main__':


    pathO = args.PathOpts(image_path="D:\MY_DRIVE_N\Masters_thesis\Dataset\V35_test")
    cam = args.CameraOpts(motion_model="hand_eye", calibration="D:\MY_DRIVE_N\Masters_thesis\Dataset\V35_test\Calibration_handeye.json")
    pose_estimation_method = "solvePnPGeneric"
    runt = args.RuntimeOpts(pose_estimation=pose_estimation_method)
    opt = args.OptimizerOpts(outlier_threshold=0.5, fix_intrinsic=True, iter=3)

    c = calibrate.Calibrate(paths=pathO, camera=cam, runtime=runt, optimizer=opt)
    c.execute()
