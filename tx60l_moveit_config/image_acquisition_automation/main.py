import rospy
import sys
from src.box_attacher_3 import Box_Attacher_3
# from src.view_planning import view_Plan
# from src.viewPlan_RL import *
# from src.viewPlan_geo import *
# from src.multical_scripts.viewPlan2 import *

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
        box_attacher.add_obstacle("roof")



        debug = False
        if debug:
            print(box_attacher.scene.get_attached_objects())
            print(box_attacher.scene.get_known_object_names())
            rospy.spin()

        # box_attacher.write_valid_joint_points(plan_num=1000)

        # row_start (from excel row)
        # box_attacher.plan_xlxs_joint_goal(row_end=8, library = 'arv')
        # box_attacher.plan_cluster_point_goal(plan_num=10)
        path = '/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/V38/poses_29June_1.json'
        box_attacher.plan_from_pose_json(path)
        # view_Plan(box_attacher)
        # box_attacher.plan_box_param()
        # viewPlan_RL(box_attacher)
        # viewPlanGeo(box_attacher)

        # path = '/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/view_plan/viewPlan2/data/'
        # json_path = '/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/view_plan/viewPlan2/pose_viewPlan2.json'
        # board_path = '/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/view_plan/viewPlan2/boards.yaml'
        # intrinsic_path = '/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/view_plan/viewPlan2/all_camera_intrinsic_V24.json'
        #
        # v = viewPlan2(box_attacher, path, board_path, intrinsic_path, json_path)
        



        print('To prevent initialisation errors please press plan and then execute without moving the robot.')
        print('Do not trust this program blindly. Be always prepared to stop the robot with the big red button')

    except rospy.ROSInterruptException:
        return 
    except KeyboardInterrupt:
        return 

if __name__ == '__main__':
    main()