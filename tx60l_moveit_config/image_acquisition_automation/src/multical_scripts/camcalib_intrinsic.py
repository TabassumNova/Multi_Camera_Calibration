import src.multical.app.intrinsic as intrinsic
import src.multical.config.arguments as args
import src.multical.config.workspace as workspace

if __name__ == '__main__':

    pathO = args.PathOpts(image_path="D:\MY_DRIVE_N\Masters_thesis\Dataset\intrinsic_V24/08320220")
    # cam = args.CameraOpts(motion_model='hand_eye')
    cam = args.CameraOpts(limit_intrinsic=None)
    runt = args.RuntimeOpts()

    i = intrinsic.Intrinsic(paths=pathO, camera=cam, runtime=runt)
    i.execute()
