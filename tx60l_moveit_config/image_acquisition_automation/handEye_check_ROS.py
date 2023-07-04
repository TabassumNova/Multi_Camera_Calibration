import rospy
from src.box_attacher_3 import *

from src.multical_scripts.handEye_check import *
from src.data_robot_mover2 import *
from src.aravis_image_acquisition import *
from src.helpers import make_directory
import os
from structs.struct import struct, to_dicts, transpose_lists

base_path = "/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/handEye_gripper/08320220"


def collect_files(base_path):
    for path, subdirs, files in os.walk(base_path):
        for name in files:
            if name == 'boards.yaml':
                board_path = os.path.join(base_path, name)
            elif name == 'calibration.json':
                intrinsic_path = os.path.join(base_path, name)
            elif name == 'gripper_pose.json':
                poseJsonPath = os.path.join(base_path, name)
    return board_path, intrinsic_path, poseJsonPath

def test_robotMove(box_attacher, handEye_struct):
    estimated_gripper_base_list = []
    p1 = handEye_struct.camera_wrt_world[0]
    cam_gripper1 = rtvec.from_matrix(matrix.transform(p1, np.linalg.inv(handEye_struct.gripper_wrt_world)))
    angle1 = rtvec.euler_angle(cam_gripper1[:3])
    base_gripper1 = handEye_struct.base_wrt_gripper[0]
    image1 = handEye_struct.images[0]


    diff_rvec = list(rtvec.euler_to_rvec([0,0,0]))
    diff_tvec = [0,0,0]
    diff = np.array((diff_rvec + diff_tvec), dtype=np.float64) # [rvec, tvec]
    cam_gripper_diff = rtvec.to_matrix(diff)
    estimated_base_gripper = np.linalg.inv(rtvec.to_matrix(rtvec.from_matrix(base_gripper1) + rtvec.from_matrix(np.linalg.inv(cam_gripper_diff))))
    
    
    move_robot(box_attacher, estimated_base_gripper)
    pose = 2
    print('Pose: ', pose)
    path = os.path.join(base_path, 'test_estimated_GripperBase')
    arv_get_image(path+'/', pose)

def export_handEye(filename, data):
    with open(filename, 'w') as outfile:
        json.dump(to_dicts(data), outfile, indent=2)
    pass


if __name__ == '__main__':
    rospy.init_node('box_attacher_3_node', anonymous=True)
    box_attacher = Box_Attacher_3()
    reset_position(box_attacher)

    board_path, intrinsic_path, poseJsonPath = collect_files(base_path)
    h = handEye(base_path, board_path, intrinsic_path, poseJsonPath)
    h.initiate_workspace()
    h.set_gripper_pose()
    handEye_struct = h.handEye_gripper(camera=0, board=1)
    # export_json = os.path.join(base_path, 'handEye_struct.json')
    # export_handEye(export_json, handEye_struct)
    
    # debug where camera_base relation unknown
    # estimated_gripper_base_list = test_robotMove(box_attacher, handEye_struct)

    # debug where camera_base relation known
    enter = input('Press enter to move robot')
    estimated_gripper_base_list = handEye_struct.estimated_gripper_base
    image_list = handEye_struct.images
    estimated_gripper_base_path = os.path.join(base_path, 'estimated_gripper_base2')
    make_directory(estimated_gripper_base_path)
    for idx, pose in enumerate(estimated_gripper_base_list):
        move_robot(box_attacher, pose)
        print('Pose: ', image_list[idx])
        arv_get_image(estimated_gripper_base_path+'/', image_list[idx])