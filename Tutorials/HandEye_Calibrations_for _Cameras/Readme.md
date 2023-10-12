<p align="right">
<img align="center" src="Images/hande.png" width="400"> 
<img align="center" src="Images/nonOverlapping.png" width="550">
</p>


# HandEye Calibration for Master-Slave Camera pair
 Here,

 base_wrt_gripper, $`_{b}^{g}\textrm{T}`$ -> $`_{Master\_camera}^{Master\_board}\textrm{T}`$
    
 cam_wrt_world, $`_{c}^{w}\textrm{T}`$ -> $`_{Slave\_camera}^{Slave\_board}\textrm{T}`$

 gripper_wrt_world, $`_{g}^{w}\textrm{T}`$ -> $`_{Master\_board}^{Slave\_board}\textrm{T}`$

 base_wrt_cam, $`_{b}^{c}\textrm{T}`$ -> $`_{Master\_camera}^{Slave\_camera}\textrm{T}`$

 ## Remarks
 $`_{Master\_camera}^{Master\_board}\textrm{T}`$ and $`_{Slave\_camera}^{Slave\_board}\textrm{T}`$ can be calculated using [solvePnP](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html) algorithm. But solvePnP generates $`_{board}^{camera}\textrm{T}`$. You need to inverse the matrix to get $`_{camera}^{board}\textrm{T}`$

 HandEye Calibration function from [handEye_final.py](https://gitlab.lrz.de/autotron-group/camera_calib_nova/-/blob/main/tx60l_moveit_config/image_acquisition_automation/src/multical_scripts/handEye_final.py?ref_type=heads)

 ```
def hand_eye_robot_world(cam_world_R, cam_world_t, base_gripper_R, base_gripper_t):
        base_cam_r, base_cam_t, gripper_world_r, gripper_world_t = \
            cv2.calibrateRobotWorldHandEye(cam_world_R, cam_world_t, base_gripper_R, base_gripper_t, method=cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH)

        base_wrt_cam = matrix.join(base_cam_r, base_cam_t.reshape(-1))
        gripper_wrt_world = matrix.join(gripper_world_r, gripper_world_t.reshape(-1))
        camera_wrt_world = matrix.join(cam_world_R, cam_world_t)
        base_wrt_gripper = matrix.join(base_gripper_R, base_gripper_t)

        err = matrix.transform(base_wrt_cam, camera_wrt_world) - matrix.transform(base_wrt_gripper, gripper_wrt_world)
        ZB = matrix.transform(base_wrt_cam, camera_wrt_world)
        error2 = base_wrt_gripper - matrix.transform(ZB, np.linalg.inv(gripper_wrt_world))
        estimated_gripper_base = np.linalg.inv(matrix.transform(ZB, np.linalg.inv(gripper_wrt_world)))

        return base_wrt_cam, gripper_wrt_world, camera_wrt_world, base_wrt_gripper, estimated_gripper_base, err, error2

 ```



