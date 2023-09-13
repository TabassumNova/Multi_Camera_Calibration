import os.path

import numpy as np

import src.multical.app.calibrate as calibrate
import src.multical.config.arguments as args
from matplotlib import pyplot as plt
import src.multical.config.workspace as workspace
from src.multical.tables import *
import json
from structs.struct import struct, to_dicts, transpose_lists
from src.multical.workspace import *
import src.multical.workspace as ws



def export_workspace(ws, path):
    '''
    ########### For json dict ##############
    new_workspace_dict = {}
    new_workspace_dict['boards'] = [b.tolist() for b in boards]
    detected_points = []
    cam_list = []
    for cam in range(len(workspace.names.camera)):
        image_list = []
        for img in range(len(workspace.names.image)):
            board_list = []
            for board in range(len(workspace.names.board)):
                board_list.append(workspace.detected_points[cam][img][board].corners.tolist())
            image_list.append(board_list)
        cam_list.append(image_list)
    new_workspace_dict['detected_points'] = cam_list
    new_workspace_dict['pose_table'] = workspace.pose_table.poses.tolist()
    json_object = json.dumps(new_workspace_dict, indent=4)
    # Writing to sample.json
    json_path = os.path.join(path, "workspace.json")
    with open(json_path, "w") as outfile:
        outfile.write(json_object)
    '''

    workspace_pickle = os.path.join(path, "workspace.pkl")
    with open(workspace_pickle, "wb") as file:
        pickle.dump(ws, file)

def camera_intrinsic_dataset(ws, path):
    intrinsic_dataset = {}

    for cam_id, cam in enumerate(ws.names.camera):
        intrinsic_dataset[cam] = {}
        board_list = []
        image_list = []
        for idx, board_id in enumerate(ws.cameras[cam_id].intrinsic_dataset['board_ids']):
            board_list.append(ws.names.board[int(board_id)])
            image_id = ws.cameras[cam_id].intrinsic_dataset['image_ids'][idx]
            image_list.append(ws.names.image[int(image_id)])
            pass
        intrinsic_dataset[cam]['boards'] = board_list
        intrinsic_dataset[cam]['images'] = image_list

    json_object = json.dumps(intrinsic_dataset, indent=4)
    # Writing to sample.json
    json_path = os.path.join(path, "intrinsic_dataset.json")
    with open(json_path, "w") as outfile:
        outfile.write(json_object)
    pass
def collect_corner_pixels(ws):
    all_pixel_list = []
    for cam in range(ws.sizes.camera):
        for img in range(ws.sizes.image):
            for board in range(ws.sizes.board):
                if ws.pose_table.valid[cam][img][board]:
                    corners = ws.detected_points[cam][img][board].corners
                    for c in corners:
                        height_limit = [int(c[0])-10, int(c[0])+10]
                        width_limit = [int(c[1])-10, int(c[1])+10]
                        img_pixel = ws.images[cam][img][width_limit[0]: width_limit[1], height_limit[0]:height_limit[1]].ravel()
                        all_pixel_list.extend(img_pixel)

        q = np.quantile(all_pixel_list, .85)
        print(q)
        plt.figure(figsize=(10, 6))
        plt.hist(all_pixel_list, 100, [0, 256])
        plt.xlabel('Corner pixel values')
        plt.ylabel('Number of pixel')
        plt.show()
    pass

def main1(datasetPath, calibration_path):
    pathO = args.PathOpts(image_path=datasetPath)
    if calibration_path:
        cam = args.CameraOpts(calibration=calibration_path,intrinsic_error_limit=1.0)
    else:
        cam = args.CameraOpts(intrinsic_error_limit=1.0)
    # pose_estimation_method = "solvePnPRansac"
    pose_estimation_method = "solvePnPGeneric"
    runt = args.RuntimeOpts(pose_estimation=pose_estimation_method, show_all_poses=True)
    opt = args.OptimizerOpts(outlier_threshold=1.2, fix_intrinsic=True, adjust_outliers=False)
    c = calibrate.Calibrate(paths=pathO, camera=cam, runtime=runt, optimizer=opt)
    workspace = c.execute_board()
    # workspace.point_table.valid = workspace.pose_table.inliers
    return workspace

if __name__ == '__main__':
    base_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset\V41_test"
    calibration_path = None
    # calibration_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset\V38\calibration.json"
    ws = main1(base_path, calibration_path)
    collect_corner_pixels(ws)
    # export_workspace(ws, base_path)
    # if not calibration_path:
    #     camera_intrinsic_dataset(ws, base_path)
    # pass
