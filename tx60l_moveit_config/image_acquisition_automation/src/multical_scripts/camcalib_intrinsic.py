import src.multical.app.intrinsic as intrinsic
import src.multical.config.arguments as args
import src.multical.config.workspace as workspace

if __name__ == '__main__':

    pathO = args.PathOpts(image_path="D:\MY_DRIVE_N\Masters_thesis\Dataset\intrinsic_26June\cam222")
    # cam = args.CameraOpts(motion_model='hand_eye')
    cam = args.CameraOpts(limit_intrinsic=None, intrinsic_error_limit=0.5)
    runt = args.RuntimeOpts()

    i = intrinsic.Intrinsic(paths=pathO, camera=cam, runtime=runt)
    i.execute()
