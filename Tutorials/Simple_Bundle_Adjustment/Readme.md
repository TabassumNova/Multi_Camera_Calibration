```
def final_reprojection_error(self):
        def fun(param_vec):
            error_list = []
            for img in range(0, 10):
                for slaveCam in range(self.workspace.sizes.camera):
                    cameraMatrix, distortion = self.workspace.cameras[slaveCam].intrinsic, self.workspace.cameras[
                        slaveCam].dist
                    masterCam_wrto_slaveCam = np.linalg.inv(rtvec.to_matrix(np.array(param_vec[slaveCam*6 : slaveCam*6+6])))
                    if slaveCam == self.master_cam:
                        master_boards = np.flatnonzero(self.workspace.pose_table.valid[slaveCam][img])
                    for boardS in range(self.workspace.sizes.board):
                        boardS_pose = np.array(param_vec[self.workspace.sizes.camera*6+boardS*6 : self.workspace.sizes.camera*6+boardS*6+6])
                        for boardM in master_boards:
                            boardM_pose = np.array(param_vec[self.workspace.sizes.camera*6+boardM*6 : self.workspace.sizes.camera*6+boardM*6+6])
                            slaveboard_wrto_boardM = rtvec.to_matrix(rtvec.relative_to(boardS_pose, boardM_pose))
                            boardM_wrto_masterCam = self.workspace.pose_table.poses[self.master_cam][img][boardM]
                            ZB = matrix.transform(boardM_wrto_masterCam, masterCam_wrto_slaveCam)
                            estimated_slaveboard_wrto_slaveCam = matrix.transform(slaveboard_wrto_boardM, ZB)
                            slaveboard_wrto_slaveCam = self.workspace.pose_table.poses[slaveCam][img][boardS]

                            point_ids = np.flatnonzero(self.workspace.point_table.valid[slaveCam][img][boardS])
                            if point_ids.size != 0:
                                imagePoints = self.workspace.point_table.points[slaveCam][img][boardS][point_ids]
                                objectPoints = np.array(
                                    [self.workspace.boards[boardS].adjusted_points[i] for i in point_ids])

                                rvec, tvec = rtvec.split(rtvec.from_matrix(estimated_slaveboard_wrto_slaveCam))
                                imP, _ = cv2.projectPoints(np.copy(objectPoints), rvec, tvec, cameraMatrix, distortion)
                                error = np.linalg.norm(imagePoints - imP.reshape([-1, 2]), axis=-1)
                                # point_errors.extend(imagePoints - imP.reshape([-1, 2]))
                                error_list.extend(list(error))
            return error_list

        def evaluation(param_vec, inliers, x):
            point_errors = fun(param_vec)
            errors = np.array([point_errors[i] for i, v in enumerate(inliers) if v == True])
            return errors

        master_cam = self.workspace.names.camera[self.master_cam]
        param_vec = self.campose_param[master_cam] + self.board_param
        point_errors = fun(param_vec)
        inliers = np.ones(len(point_errors))
        for i in range(1):
            info(f"Adjust_outliers {i}:")
            res = least_squares(evaluation, param_vec,
                                verbose=2, x_scale='jac', f_scale=1.0, ftol=1e-4, max_nfev=100,
                                method='trf', loss='linear', args=(inliers, 0))
            point_errors = fun(res.x)
            threshold = np.quantile(point_errors, 0.70)
            inliers = np.array(point_errors < threshold).flatten()
        pass


```
