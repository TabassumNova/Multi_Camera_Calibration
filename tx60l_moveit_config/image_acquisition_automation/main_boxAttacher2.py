import rospy
from src.box_attacher_2 import Box_Attacher_2

def main():
    rospy.init_node('box_attacher_2_node', anonymous=True)
    try:
        box_attacher = Box_Attacher_2()

        stl_file_name = '/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/data/gripper_sample1.stl'
        box_attacher.replace_box(stl_file_name=stl_file_name) 

        box_attacher.replace_box()

        stl_file_name = '/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/data/gripper_sample.stl'
        box_attacher.replace_box(stl_file_name=stl_file_name) 



        # OGUZ use case 
        """
        - name of two stls is different 
        - i provide path to the stl files 

        - nothing attached 
        - I call function (replace)
        - program attaches part 
        - I call function again (the same one) 
        - program detects if something is attached and removes it (if it is not the desired) 
        - programm attaches new part 
        """

        # For obstacle
        # box_attacher.add_obstacle("camera_wall_1")
        # box_attacher.add_obstacle("camera_wall_2")
        # box_attacher.add_obstacle("glass_wall")
        # # box_attacher.add_obstacle("left_wall")
        # box_attacher.add_obstacle("base")
        # box_attacher.add_obstacle("roof")


        print('To prevent initialisation errors please press plan and then execute without moving the robot.')
        print('Do not trust this program blindly. Be always prepared to stop the robot with the big red button')

    except rospy.ROSInterruptException:
        return 
    except KeyboardInterrupt:
        return 

if __name__ == '__main__':
    main()