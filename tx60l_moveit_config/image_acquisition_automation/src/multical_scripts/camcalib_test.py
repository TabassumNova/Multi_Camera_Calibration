import src.multical.app.calibrate as calibrate
import src.multical.config.arguments as args

if __name__ == '__main__':

    pathO = args.PathOpts(image_path="D:\MY_DRIVE_N\Masters_thesis\Dataset\V10_debug\data/test")
    cam = args.CameraOpts(calibration="D:\MY_DRIVE_N\Masters_thesis\Dataset\V10_debug\data/train/calibration.json")
    runt = args.RuntimeOpts()
    opt = args.OptimizerOpts(fix_intrinsic=True, fix_camera_poses=True)

    c = calibrate.Calibrate(paths=pathO, camera=cam, runtime=runt, optimizer=opt)
    c.execute()
