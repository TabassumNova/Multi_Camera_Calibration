"""
Place one camera in front of Board 1 (at the place of CAM 220)
This program will take Gripper_to_Base position and orientation from a pre-saved json file
And move the gripper to the memorized poses
"""
import json
import numpy as np
from src.multical.transform import rtvec
from scipy.spatial.transform import Rotation as R

import rospy
from src.box_attacher_3 import *
from src.data_robot_mover2 import *
from src.aravis_image_acquisition import *
from src.helpers import make_directory

poseJsonPath = ""
image_path = ""

def set_gripper_pose(poseJsonPath):
    file = open(poseJsonPath)
    data = json.load(file)
    gripper_pose = {}
    for pose in data:
        position = [float(j) for j in data[str(pose)]["position (x,y,z)"]]
        orientation = [float(j) for j in data[str(pose)]["orintation (w,x,y,z)"]]
        r = R.from_quat([orientation[1], orientation[2], orientation[3], orientation[0]])
        rvec = r.as_rotvec()
        tvec = np.array(position)
        rt_matrix = rtvec.to_matrix(rtvec.join(rvec, tvec))
        gripper_pose[pose] = rt_matrix
    return gripper_pose

def main():
    rospy.init_node('box_attacher_3_node', anonymous=True)
    try:
        # default box size in m
        stl = False
        launch = False
        size = (0.25, 0.25, 0.25)
        myargv = rospy.myargv(argv=sys.argv)
        stl_file_name = ''
        # Inserting Boxsize at start
        if (len(myargv) == 2):
            stl = True
            print('stl file will be added')
            # name = String(myargv[1])
            stl_file_name = str(myargv[1])
        elif len(myargv) == 3:
            stl = True
            print('stl file will be added')
            stl_file_name = str(myargv[1])
            launch = True
            camera_name = str(myargv[2])
        elif len(myargv) > 5:
            print('Fehlerhafte Eingabe')
        elif len(myargv) == 4:
            print('Size of box set')
            size = (float(myargv[1]), float(myargv[2]), float(myargv[3]))
        elif len(myargv) == 5:
            print('Size of box set')
            size = (float(myargv[1]), float(myargv[2]), float(myargv[3]))
            launch = True
            camera_name = str(myargv[4])

        box_attacher = Box_Attacher_3()
        box_attacher.replace_box(stl, size, stl_file_name)
        box_attacher.add_obstacle("camera_wall_1")
        box_attacher.add_obstacle("camera_wall_2")
        box_attacher.add_obstacle("glass_wall")
        # box_attacher.add_obstacle("left_wall")
        box_attacher.add_obstacle("base")

        debug = False
        if debug:
            print(box_attacher.scene.get_attached_objects())
            print(box_attacher.scene.get_known_object_names())
            rospy.spin()

        # start
        reset_position(box_attacher)
        poseJsonPath = '/home/raptor//tx60_moveit/src/tx60l_moveit_config/python_program/image/board270/gripper_pose.json'
        image_path = '/home/raptor//tx60_moveit/src/tx60l_moveit_config/python_program/image/board270/'
        gripper_pose = set_gripper_pose(poseJsonPath)
        for key, pose in gripper_pose.items():
            move_robot(box_attacher, pose)
            print('Pose: ', key)
            arv_get_image(image_path, int(key))
        # end

        print('To prevent initialisation errors please press plan and then execute without moving the robot.')
        print('Do not trust this program blindly. Be always prepared to stop the robot with the big red button')

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
     main()
