import multical.app.intrinsic as intrinsic
import multical.config.arguments as args
import multical.config.workspace as workspace

if __name__ == '__main__':

    pathO = args.PathOpts(image_path="D:\MY_DRIVE_N\Masters_thesis\Dataset\V23/36220113")
    # cam = args.CameraOpts(motion_model='hand_eye')
    cam = args.CameraOpts(limit_intrinsic=None)
    runt = args.RuntimeOpts()

    i = intrinsic.Intrinsic(paths=pathO, camera=cam, runtime=runt)
    i.execute()
