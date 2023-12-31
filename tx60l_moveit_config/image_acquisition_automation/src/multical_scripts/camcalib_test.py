import src.multical.app.calibrate as calibrate
import src.multical.config.arguments as args
import src.multical.config.workspace as workspace
import os

def collect_initialization_results(base_path):
    calib_dict = {}
    calib_dict['calibration_path'] = []
    calib_dict['calibration_name'] = []
    for path, subdirs, files in os.walk(base_path):
        for name in files:
            if "calibration_" in name:
                name1 = name.split('calibration_')[1].split('.')[0]
                calib_path = os.path.join(base_path, name)
                calib_dict['calibration_path'].append(calib_path)
                calib_dict['calibration_name'].append('calibration_' + name1)
    return calib_dict

def camera_initialization(base_path, cam_name):
    cam_init = "calibration_" + cam_name + '.json'
    for path, subdirs, files in os.walk(base_path):
        for name in files:
            if name == cam_init:
                calib_path = os.path.join(base_path, name)
    return calib_path

def main1(base_path):
    calibration_dict = collect_initialization_results(base_path)

    for idx, v in enumerate(calibration_dict['calibration_path']):
        pathO = args.PathOpts(name=calibration_dict['calibration_name'][idx], image_path=base_path)
        cam = args.CameraOpts(intrinsic_error_limit=0.5, calibration=calibration_dict['calibration_path'][idx])
        pose_estimation_method = "solvePnPRansac"
        runt = args.RuntimeOpts(pose_estimation=pose_estimation_method)
        opt = args.OptimizerOpts(fix_intrinsic=True, fix_camera_poses=True)

        c = calibrate.Calibrate(paths=pathO, camera=cam, runtime=runt, optimizer=opt)
        c.execute()

def main2(base_path, cam_name):
    # '08320217' , '08320218', '08320220', '08320221', '08320222', '36220113'
    calib_path = camera_initialization(base_path, cam_name)
    pathO = args.PathOpts(name=cam_name, image_path=base_path)
    cam = args.CameraOpts(intrinsic_error_limit=0.5, calibration=calib_path)
    pose_estimation_method = "solvePnPRansac"
    runt = args.RuntimeOpts(pose_estimation=pose_estimation_method)
    opt = args.OptimizerOpts(fix_intrinsic=True, fix_camera_poses=True)

    c = calibrate.Calibrate(paths=pathO, camera=cam, runtime=runt, optimizer=opt)
    c.execute()

if __name__ == '__main__':

    base_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset\V24_test2"
    main1(base_path)
    # main2(base_path, '08320217')

# import src.multical.app.calibrate as calibrate
# import src.multical.config.arguments as args
#
# if __name__ == '__main__':
#
#     pathO = args.PathOpts(image_path="D:\MY_DRIVE_N\Masters_thesis\Dataset\V10_debug\data/test")
#     cam = args.CameraOpts(calibration="D:\MY_DRIVE_N\Masters_thesis\Dataset\V10_debug\data/train/calibration.json")
#     runt = args.RuntimeOpts()
#     opt = args.OptimizerOpts(fix_intrinsic=True, fix_camera_poses=True)
#
#     c = calibrate.Calibrate(paths=pathO, camera=cam, runtime=runt, optimizer=opt)
#     c.execute()
