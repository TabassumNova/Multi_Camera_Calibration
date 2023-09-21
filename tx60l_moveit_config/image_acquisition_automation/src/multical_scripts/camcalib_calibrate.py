import src.multical.app.calibrate as calibrate
import src.multical.config.arguments as args
import src.multical.config.workspace as workspace
import os
import random
import pickle
import numpy as np
import json

def collect_initialization_results(base_path):
    calib_dict = {}
    calib_dict['calibration_path'] = []
    calib_dict['calibration_name'] = []
    for path, subdirs, files in os.walk(base_path):
        for name in files:
            if "initial_calibration_M" in name:
                name1 = name.split('initial_calibration_M')[1].split('.')[0]
                calib_path = os.path.join(base_path, name)
                calib_dict['calibration_path'].append(calib_path)
                calib_dict['calibration_name'].append('calibration_' + name1)
    return calib_dict

def camera_initialization(base_path, cam_name):
    cam_init = "initial_calibration_M" + cam_name + '.json'
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
        pose_estimation_method = "solvePnPGeneric"
        runt = args.RuntimeOpts(pose_estimation=pose_estimation_method)
        opt = args.OptimizerOpts(outlier_threshold=0.5, fix_intrinsic=True, iter=2)

        c = calibrate.Calibrate(paths=pathO, camera=cam, runtime=runt, optimizer=opt)
        c.execute()

def choose_random_images(base_path, image_number = 20):
    for path, subdirs, files in os.walk(base_path):
        dir = os.path.join(base_path, subdirs[0])
        for _,_,images in os.walk(dir):
            img_list = random.sample(images, image_number)
            return img_list

def load_workspace_pkl(pkl_path):
    workspace = pickle.load(open(pkl_path, "rb"))
    inlier_error = workspace.calibrations['calibration'].reprojection_inliers
    final_error = np.sqrt(np.square(inlier_error).mean())
    return final_error


def main3(base_path, masterCam):
    calibration_dict = collect_initialization_results(base_path)
    iteration_dict = {}
    for idx, v in enumerate(calibration_dict['calibration_path']):
        if idx == masterCam:
            error = 1
            iter = 1
            while error >= 1:
                iteration_dict[iter] = {}
                image_list = choose_random_images(base_path)
                pathO = args.PathOpts(name=calibration_dict['calibration_name'][idx], image_path=base_path)
                cam = args.CameraOpts(intrinsic_error_limit=0.5, calibration=calibration_dict['calibration_path'][idx])
                pose_estimation_method = "solvePnPGeneric"
                runt = args.RuntimeOpts(pose_estimation=pose_estimation_method, image_list=image_list)
                opt = args.OptimizerOpts(outlier_threshold=0.5, fix_intrinsic=True, iter=2)

                c = calibrate.Calibrate(paths=pathO, camera=cam, runtime=runt, optimizer=opt)
                c.execute()
                pkl_path = os.path.join(base_path, calibration_dict['calibration_name'][idx]+'.pkl')
                error = load_workspace_pkl(pkl_path)
                iteration_dict[iter]['reprojection_error'] = error
                iteration_dict[iter]['image_list'] = image_list
                json_object = json.dumps(iteration_dict, indent=4)
                # Writing to sample.json
                json_path = os.path.join(base_path, "calibration_iteration.json")
                with open(json_path, "w") as outfile:
                    outfile.write(json_object)
                iter += 1
    pass

def main2(base_path, cam_name):
    # '08320217' , '08320218', '08320220', '08320221', '08320222', '36220113'
    calib_path = camera_initialization(base_path, cam_name)
    pathO = args.PathOpts(name=cam_name, image_path=base_path)
    cam = args.CameraOpts(intrinsic_error_limit=0.5, calibration=calib_path)
    pose_estimation_method = "solvePnPGeneric" #"solvePnPRansac"
    runt = args.RuntimeOpts(pose_estimation=pose_estimation_method)
    opt = args.OptimizerOpts(outlier_threshold=0.5, fix_intrinsic=True, iter=2)

    c = calibrate.Calibrate(paths=pathO, camera=cam, runtime=runt, optimizer=opt)
    c.execute()

if __name__ == '__main__':

    base_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset\V36_1"
    # main1(base_path)
    # main2(base_path, '08320217')
    main3(base_path, 0)