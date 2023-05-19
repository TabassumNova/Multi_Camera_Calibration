import multical.app.calibrate as calibrate
import multical.config.arguments as args

if __name__ == '__main__':

    pathO = args.PathOpts(image_path="D:\MY_DRIVE_N\Masters_thesis\multical_test_new/net-2-base\data")
    cam = args.CameraOpts()
    runt = args.RuntimeOpts()
    opt = args.OptimizerOpts()

    c = calibrate.Calibrate(paths=pathO, camera=cam, runtime=runt, optimizer=opt)
    c.execute()
