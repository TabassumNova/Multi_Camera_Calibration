import rospy
from src.box_attacher_3 import *
from handEye_check import *
from src.data_robot_mover2 import *
from src.aravis_image_acquisition import *
from src.helpers import make_directory

path = "D:\MY_DRIVE_N\Masters_thesis\Dataset\handEye_gripper/08320220/08320220"
poseJsonPath = "D:\MY_DRIVE_N\Masters_thesis\Dataset\handEye_gripper/08320220/08320220/stream_220.json"
board_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset\handEye_gripper/08320220/08320220/boards.yaml"
intrinsic_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset\handEye_gripper/08320220/08320220/calibration.json"
boardGripper_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset\handEye_gripper/08320220/08320220/boardGripper.json"
estimated_gripper_base_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset\handEye_gripper/08320220/08320220/estimated_gripper_base"

if __name__ == '__main__':
    rospy.init_node('box_attacher_3_node', anonymous=True)
    box_attacher = Box_Attacher_3()
    h = handEye(path, board_path, intrinsic_path, poseJsonPath)
    h.initiate_workspace()
    h.set_gripper_pose()
    handEye_struct = h.handEye_gripper(camera=0, board=1)
    estimated_gripper_base_list = h.test_robotMove(handEye_struct)
    enter = input('Press enter to move robot')
    reset_position(box_attacher)
    make_directory(estimated_gripper_base_path)
    for idx, pose in enumerate(estimated_gripper_base_list):
        move_robot(box_attacher, pose)
        print('Pose: ', idx)
        arv_get_image(estimated_gripper_base_path, idx)