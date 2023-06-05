import multical.app.calibrate as calibrate
import multical.config.arguments as args
import multical.config.workspace as workspace

if __name__ == '__main__':

    pathO = args.PathOpts(image_path="D:\MY_DRIVE_N\Masters_thesis\Dataset\V23\V23")
    cam = args.CameraOpts(motion_model='hand_eye', calibration="D:\MY_DRIVE_N\Masters_thesis\Dataset\V23\V23/calibration.json")
    # pathO = args.PathOpts(image_path="D:\MY_DRIVE_N\Masters_thesis\Dataset\V10_debug\data/train")
    # cam = args.CameraOpts()
    runt = args.RuntimeOpts()
    # opt = args.OptimizerOpts()
    opt = args.OptimizerOpts(outlier_threshold=1.2, fix_intrinsic=True)

    c = calibrate.Calibrate(paths=pathO, camera=cam, runtime=runt, optimizer=opt)
    c.execute()
