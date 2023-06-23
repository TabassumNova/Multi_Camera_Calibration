from src.aravis_stream_acquisition import *
from src.aravis_image_acquisition import *
from src.box_attacher_3 import *
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf


path = '/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/'
poseJsonPath = '/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/stream_220.json'
rospy.init_node('box_attacher_3_node', anonymous=True)
box_attacher = Box_Attacher_3()
common_focus = [-61, -19, 117, 112, 5, -247]
plan = box_attacher.move_robot_joints(np.array(common_focus))

pose = 1
json_dict = {}
while True:
    print('pose: ', pose)
    # write pose
    json_dict[pose] = {}
    current_pose = box_attacher.move_group.get_current_pose().pose.position
    current_orientation = box_attacher.move_group.get_current_pose().pose.orientation
    json_dict[pose]['position (x,y,z)'] = [str(current_pose.x), str(current_pose.y), str(current_pose.z)]
    json_dict[pose]['orintation (w,x,y,z)'] = [str(current_orientation.w), str(current_orientation.x), str(current_orientation.y), str(current_orientation.z)]
    json_dict[pose]['joint_goal'] = [str(a) for a in box_attacher.move_group.get_current_joint_values()]
    
    # Serializing json
    json_object = json.dumps(json_dict, indent=4)
    # Writing to sample.json
    with open(poseJsonPath, "w") as outfile:
        outfile.write(json_object)
    ##

    arv_get_image(path, pose)
    enter = input("Hit ENTER if you want next pose: ")
    pose += 1
