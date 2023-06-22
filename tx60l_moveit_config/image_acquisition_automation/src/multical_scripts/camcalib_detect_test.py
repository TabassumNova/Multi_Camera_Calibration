import src.multical.app.calibrate as calibrate
import src.multical.config.arguments as args
import numpy as np
from collections import Counter
import cv2
from scipy.spatial.transform import Rotation as R
from numpy.linalg import inv

if __name__ == '__main__':

    pathO = args.PathOpts(image_path="D:/MY_DRIVE_N/Masters_thesis/Dataset/view_plan/Intrinsic/test")
    cam = args.CameraOpts()
    runt = args.RuntimeOpts()
    opt = args.OptimizerOpts()

    c = calibrate.Calibrate(paths=pathO, camera=cam, runtime=runt, optimizer=opt)
    ws, boards = c.execute_new()


    # select one board for each camera
    num_cam = ws.point_table['points'].shape[0]
    num_img = ws.point_table['points'].shape[1]
    num_board = ws.point_table['points'].shape[2]
    selected_board = 0
    board_selection_matrix = np.zeros((num_cam))

    for cam in range(0,num_cam):
        board_selection_list = []
        board_selection_dict = {}
        for im in range (0, num_img):
            for b in range (0, num_board):
                length = len(ws.detected_points[cam][im][b]['ids'])
                # board_selection_matrix[cam][im][b] = length
                if length > 0:
                    board_selection_list.append(b)
                    if b in board_selection_dict:
                        board_selection_dict[b] = board_selection_dict[b] + length
                    else:
                        board_selection_dict[b] = length

        sorted_dict = sorted(board_selection_dict.items(),  reverse = False)
        # count = Counter(board_selection_list).most_common()
        # board_selection_matrix[cam] = max(board_selection_list,key=board_selection_list.count)
        board_selection_matrix[cam] = next(iter(board_selection_dict))

    #### Calibration ####
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    # objp = np.zeros((11 * 8, 3), np.float32)
    # objp[:, :2] = np.mgrid[0:11, 0:8].T.reshape(-1, 2)
    objp = boards['cube2_0'].adjusted_points
    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    cam_board_transformation = {}
    ## count skip img
    skip_img = []
    for cam in range (0, num_cam):
        selected_board = int(board_selection_matrix[cam])
        for im in range(0, num_img):
            if len(ws.detected_points[cam][im][selected_board]['ids']) < 8:
                skip_img.append(im)


    obj_points_dict = {}
    img_points_dict = {}
    img_obj_point_dict = {}
    for cam in range (0, num_cam):
        cam_board_transformation[cam] = {}
        selected_board = int(board_selection_matrix[cam])
        objpoints = []  # 3d point in real world space
        imgpoints = []
        img_obj_point_dict[cam] = {}
        for im in range (0, num_img):
            if im not in skip_img:
                img_obj_point_dict[cam][im] = {}
                if len(ws.detected_points[cam][im][selected_board]['ids'])>4:
                    # c = np.zeros((objp.shape[0],2), dtype="float32")
                    # ids = ws.detected_points[cam][im][selected_board]['ids']
                    # j = 0
                    # for i in ids:
                    #     c[i] = np.float32(ws.detected_points[cam][im][selected_board]['corners'][j])
                    #     j += 1
                    # imgpoints.append(np.array(c))
                    # objpoints.append(np.array(objp))

                    # working version
                    c = ws.detected_points[cam][im][selected_board]['corners']
                    imgpoints.append(ws.detected_points[cam][im][selected_board]['corners'])
                    objpoints.append(np.array([list(objp[i]) for i in ws.detected_points[cam][im][selected_board]['ids']]))

                    j = 0
                    for i in ws.detected_points[cam][im][selected_board]['ids']:
                        img_obj_point_dict[cam][im][i] = {}
                        img_obj_point_dict[cam][im][i]['corners'] = ws.detected_points[cam][im][selected_board]['corners'][j]
                        img_obj_point_dict[cam][im][i]['obj_point'] = objp[i]
                        j += 1


        obj_points_dict[cam] = objpoints
        img_points_dict[cam] = imgpoints
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (5472, 3648), None, None)
        # valid, rvec1, tvec1 = cv2.solvePnP(objpoints, imgpoints, mtx, dist)

        Rotation = np.zeros((len(rvecs),3 ,3))
        for r in range (0, len(rvecs)):
            rot = R.from_rotvec(rvecs[r].T)
            Rotation[r] = rot.as_matrix()

        cam_board_transformation[cam]['rvecs'] = Rotation
        Translation = np.zeros((len(tvecs), 3))
        for t in range (0, len(tvecs)):
            Translation[t] = tvecs[t].T
        cam_board_transformation[cam]["tvecs"] = Translation
        cam_board_transformation[cam]['mtx'] = mtx
        cam_board_transformation[cam]['dist'] = dist
        cam_board_transformation[cam]['ret'] = ret
        # cam_board_transformation[cam]['pnpT'] = tvec1

    ## check solvePnP
    # pose_dict = {}
    # for cam in range (0, num_cam):
    #     # cam_board_transformation[cam] = {}
    #     pose_dict[cam] = {}
    #     selected_board = int(board_selection_matrix[cam])
    #     # objpoints = []  # 3d point in real world space
    #     # imgpoints = []
    #     # img_obj_point_dict[cam] = {}
    #     for im in range (0, num_img):
    #         pose_dict[cam][im] = {}
    #         if im not in skip_img:
    #             img_obj_point_dict[cam][im] = {}
    #             if len(ws.detected_points[cam][im][selected_board]['ids'])>4:
    #                 # c = np.zeros((objp.shape[0],2), dtype="float32")
    #                 # ids = ws.detected_points[cam][im][selected_board]['ids']
    #                 imgpoints = ws.detected_points[cam][im][selected_board]['corners']
    #                 objpoints = np.array([list(objp[i]) for i in ws.detected_points[cam][im][selected_board]['ids']])
    #                 valid, rvec1, tvec1 = cv2.solvePnP(objpoints, imgpoints, cam_board_transformation[cam]['mtx'], cam_board_transformation[cam]['dist'])
    #                 pose_dict[cam][im]['R'] = rvec1
    #                 pose_dict[cam][im]['T'] = tvec1

    # ##### hand eye
    # cam_transformation_dict = {}
    # cam_pose_dict = {}
    # board_pose_dict = {}
    # master_cam = 0
    # master_board = board_selection_matrix[master_cam]
    #
    # for c1 in range (0, num_cam):
    #     b1 = board_selection_matrix[c1]
    #     if c1 == master_cam:
    #         b1 = board_selection_matrix[c1]
    #         r = np.eye(3, dtype='float64')
    #         t = np.array([[0, 0, 0]], dtype='float64').T
    #         h = np.hstack([r, t])
    #         v = np.array([[0, 0, 0, 1]], dtype='float64')
    #         board_pose_dict[int(b1)] = np.concatenate((h, v), axis=0)
    #         for c2 in range(0, num_cam):
    #             b2 = board_selection_matrix[c2]
    #
    #             base_world_r, base_world_t, c1_c2_r, c1_c2_t = cv2.calibrateRobotWorldHandEye(
    #                 cam_board_transformation[c1]['rvecs'],
    #                 cam_board_transformation[c1]['tvecs'],
    #                 cam_board_transformation[c2]['rvecs'],
    #                 cam_board_transformation[c2]['tvecs'])
    #             # text = 'cam' + str(c1) + '_to_' + 'cam' + str(c2)
    #             # cam_transformation_dict[text] = {}
    #             # cam_transformation_dict[text]['R'] = c1_c2_r
    #             # cam_transformation_dict[text]['T'] = c1_c2_t
    #             if b2 != b1:
    #                 v = np.array([[0, 0, 0, 1]], dtype='float64')
    #                 h = np.hstack([base_world_r, base_world_t])
    #                 rt = np.concatenate((h, v), axis=0)
    #                 board_pose_dict[int(b2)] = rt
    #             text = 'cam' + str(c1) + '_to_' + 'cam' + str(c2)
    #             cam_transformation_dict[text] = {}
    #             cam_transformation_dict[text]['R'] = c1_c2_r
    #             cam_transformation_dict[text]['T'] = c1_c2_t
    #             v = np.array([[0, 0, 0, 1]], dtype='float64')
    #             h = np.hstack([c1_c2_r, c1_c2_t])
    #             rt = np.concatenate((h, v), axis=0)
    #             # rt = rtvec.join(c1_c2_r, c1_c2_t.T)
    #             cam_pose_dict[c2] = rt

    print('start handEye new')
    ##### hand eye new
    cam_transformation_dict = {}
    cam_pose_dict = {}
    board_pose_dict = {}
    master_cam = 0
    master_board = board_selection_matrix[master_cam]

    for c1 in range(0, num_cam):
        cam_pose_dict[c1] = {}
        board_pose_dict[c1] = {}
        b1 = board_selection_matrix[c1]
        r = np.eye(3, dtype='float64')
        t = np.array([[0, 0, 0]], dtype='float64').T
        h = np.hstack([r, t])
        v = np.array([[0, 0, 0, 1]], dtype='float64')
        board_pose_dict[c1][int(b1)] = np.concatenate((h, v), axis=0)
        for c2 in range(0, num_cam):
            cam_pose_dict[c1][c2] = {}
            b2 = board_selection_matrix[c2]

            base_world_r, base_world_t, c1_c2_r, c1_c2_t = cv2.calibrateRobotWorldHandEye(
                cam_board_transformation[c1]['rvecs'],
                cam_board_transformation[c1]['tvecs'],
                cam_board_transformation[c2]['rvecs'],
                cam_board_transformation[c2]['tvecs'])
            # text = 'cam' + str(c1) + '_to_' + 'cam' + str(c2)
            # cam_transformation_dict[text] = {}
            # cam_transformation_dict[text]['R'] = c1_c2_r
            # cam_transformation_dict[text]['T'] = c1_c2_t
            if b2 != b1:
                v = np.array([[0, 0, 0, 1]], dtype='float64')
                h = np.hstack([base_world_r, base_world_t])
                rt = np.concatenate((h, v), axis=0)
                board_pose_dict[c1][int(b2)] = rt
            # text = 'cam' + str(c1) + '_to_' + 'cam' + str(c2)
            # cam_transformation_dict[text] = {}
            # cam_transformation_dict[text]['R'] = c1_c2_r
            # cam_transformation_dict[text]['T'] = c1_c2_t
            v = np.array([[0, 0, 0, 1]], dtype='float64')
            h = np.hstack([c1_c2_r, c1_c2_t])
            rt = np.concatenate((h, v), axis=0)
            # rt = rtvec.join(c1_c2_r, c1_c2_t.T)
            # cam_pose_dict[c1][c2]['r'] = c1_c2_r
            # cam_pose_dict[c1][c2]['t'] = c1_c2_t
            cam_pose_dict[c1][c2]['rt'] = rt

        if c1 != master_cam:
            rt_master = inv(cam_pose_dict[c1][master_cam]['rt'])
            for c2 in range(0, num_cam):
                rt_c2 = cam_pose_dict[c1][c2]['rt']
                rt = rt_c2 @ rt_master
                cam_pose_dict[c1][c2]['rt'] = rt

                # rot1 = cam_pose_dict[c1][master_cam]['r']
                # t1 = cam_pose_dict[c1][master_cam]['t'].T
                # rot2 = cam_pose_dict[c1][c2]['r']
                # t2 = cam_pose_dict[c1][c2]['t'].T
                # rvec1 = R.from_matrix(rot1).as_rotvec()
                # rvec2 = R.from_matrix(rot2).as_rotvec()
                # base_world_r, base_world_t, grip_cam_r, grip_cam_t = cv2.calibrateRobotWorldHandEye(
                #     rvec1, t1, rvec2, t2)



    ## Reprojection error for handeye
    pose_dict = {}
    for cam in range(0, num_cam):
        # cam_board_transformation[cam] = {}
        pose_dict[cam] = {}
        selected_board = int(board_selection_matrix[cam])
        # objpoints = []  # 3d point in real world space
        # imgpoints = []
        # img_obj_point_dict[cam] = {}
        pose_dict[cam][selected_board] = {}
        for im in range(0, num_img):
            pose_dict[cam][selected_board][im] = {}
            if im not in skip_img:
                img_obj_point_dict[cam][im] = {}
                if len(ws.detected_points[cam][im][selected_board]['ids']) > 4:
                    # c = np.zeros((objp.shape[0],2), dtype="float32")
                    # ids = ws.detected_points[cam][im][selected_board]['ids']
                    imgpoints = ws.detected_points[cam][im][selected_board]['corners']
                    objpoints = np.array(
                        [list(objp[i]) for i in ws.detected_points[cam][im][selected_board]['ids']])
                    valid, rvec1, tvec1 = cv2.solvePnP(objpoints, imgpoints,
                                                       cam_board_transformation[cam]['mtx'],
                                                       cam_board_transformation[cam]['dist'])

                    Rotation = np.zeros((3, 3))
                    rot = R.from_rotvec(rvec1.T)
                    Rotation = rot.as_matrix()
                    v = np.array([[0, 0, 0, 1]], dtype='float64')
                    h = np.hstack([Rotation[0], tvec1])
                    rt = np.concatenate((h, v), axis=0)
                    pose_dict[cam][selected_board][im]['rt'] = rt

                    ## test
                    # cameraMatrix = cam_board_transformation[cam]['mtx']
                    # distortion = cam_board_transformation[cam]['dist']
                    # rmatrix = rt[0:3, 0:3]
                    # tmatrix = rt[0:3, 3]
                    # rvec2 = R.from_matrix([rmatrix]).as_rotvec()
                    # tvec2 = tmatrix.T
                    # imagePoints, _ = cv2.projectPoints(objpoints, rvec2, tvec2, cameraMatrix, distortion)
                    # print(type(imagePoints))
                    # print(imagePoints.shape)
                    # error = imagePoints.reshape([-1, 2]) - imgpoints
                    # x = error
                    ## test end

    error_dict = {}
    objp = boards['cube2_0'].adjusted_points
    for c1 in range (0, num_cam):
        b1 = int(board_selection_matrix[c1])
        if c1 == master_cam:
            error_dict[c1] = {}
            for c2 in range (0, num_cam):
                b2 = int(board_selection_matrix[c2])
                error_dict[c1][c2] = {}
                for im in range (0, num_img):
                    if im not in skip_img:
                        # error_dict[c1][c2][im] = {}
                        transformation = inv(cam_pose_dict[c2]) @ (pose_dict[c1][b1][im]['rt']) @ (board_pose_dict[b2])
                        objectPoints = np.array([list(objp[i]) for i in ws.detected_points[c2][im][b2]['ids']])
                        cameraMatrix = cam_board_transformation[c2]['mtx']
                        distortion = cam_board_transformation[c2]['dist']
                        rmatrix = transformation[0:3, 0:3]
                        tmatrix = transformation[0:3, 3]
                        rvec = R.from_matrix([rmatrix]).as_rotvec()
                        tvec = tmatrix.T
                        imagePoints, _ = cv2.projectPoints(objectPoints, rvec, tvec, cameraMatrix, distortion)
                        error = imagePoints.reshape([-1, 2]) - ws.detected_points[c2][im][b2]['corners']
                        error_dict[c1][c2][im] = error
                # if b1 != b2:
                #     # base_world_r, base_world_t, gripper_cam_r, gripper_cam_t = cv2.calibrateRobotWorldHandEye(
                #     #     world_camera_r, world_camera_t, base_gripper_r, base_gripper_t)
                #     # c1_c2_r, c1_c2_t, gripper_cam_r, gripper_cam_t = cv2.calibrateRobotWorldHandEye(cam_board_transformation[c1]['rvecs'],
                #     #                                                                         cam_board_transformation[c1]['tvecs'],
                #     #                                                                         cam_board_transformation[c2]['rvecs'],
                #     #                                                                         cam_board_transformation[c2]['tvecs'])
                #
                #     base_world_r, base_world_t, c1_c2_r, c1_c2_t = cv2.calibrateRobotWorldHandEye(
                #         cam_board_transformation[c1]['rvecs'],
                #         cam_board_transformation[c1]['tvecs'],
                #         cam_board_transformation[c2]['rvecs'],
                #         cam_board_transformation[c2]['tvecs'])
                #     text = 'cam' + str(c1) + '_to_' + 'cam' + str(c2)
                #     cam_transformation_dict[text] = {}
                #     cam_transformation_dict[text]['R'] = c1_c2_r
                #     cam_transformation_dict[text]['T'] = c1_c2_t
                #
                # else:
                #     # stereo calibration
                #     obj_point = []
                #     img_point1 = []
                #     img_point2 = []
                #     b = int(b1)
                #     for im in range (0, num_img):
                #         if im not in skip_img:
                #             objp = []
                #             imgp1 = []
                #             imgp2 = []
                #             id1 = ws.detected_points[c1][im][b]['ids']
                #             id2 = ws.detected_points[c2][im][b]['ids']
                #             common_ids = np.intersect1d(id1, id2)
                #             if len(common_ids)>6:
                #                 for i in common_ids:
                #                     objp.append(img_obj_point_dict[c1][im][i]['obj_point'])
                #                     imgp1.append(img_obj_point_dict[c1][im][i]['corners'])
                #                     imgp2.append(img_obj_point_dict[c2][im][i]['corners'])
                #                 obj_point.append(np.array(objp))
                #                 img_point1.append(np.array(imgp1))
                #                 img_point2.append(np.array(imgp2))
                #
                #     print(c1, c2)
                #
                #     ret, CM1, dist1, CM2, dist2, R, T, E, F = cv2.stereoCalibrate(obj_point, img_point1, img_point2,
                #                                                                  cam_board_transformation[c1]['mtx'], cam_board_transformation[c1]['dist'],
                #                                                                  cam_board_transformation[c2]['mtx'], cam_board_transformation[c2]['dist'], (5472, 3648),
                #                                                                  criteria=criteria,
                #                                                                  flags=cv2.CALIB_FIX_INTRINSIC)
                #     text = 'cam' + str(c1) + '_to_' + 'cam' + str(c2)
                #     cam_transformation_dict[text] = {}
                #     cam_transformation_dict[text]['R'] = R
                #     cam_transformation_dict[text]['T'] = T
                #     pass


    print ('end')




