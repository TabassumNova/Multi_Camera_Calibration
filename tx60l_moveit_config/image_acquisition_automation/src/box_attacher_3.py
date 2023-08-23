import sys
import roslib
import xlsxwriter
import json
import numpy as np

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi

import tf
# import camStreamer.CamStreamer
from .camStreamer import CamStreamer
from .helpers import *
from .TIS_image_acquisition import start_tis_image_acquisition
from .aravis_image_acquisition import start_arv_image_acquisition
from .data_robot_mover2 import *

class Box_Attacher_3(object):
    """MoveGroupPythonIntefaceTutorial"""

    def __init__(self, node_name='box_attacher'):
        # super(MoveGroupPythonIntefaceTutorial, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        # Misc variables
        # self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group

        # set robot motion planner parameters 
        self.move_group.set_planner_id("TRRT") # good: "TRRT", "PDST"
        self.move_group.set_goal_orientation_tolerance(0.0001) 
        self.move_group.set_goal_position_tolerance(0.000001) # in m; default 1 µm 
        self.move_group.set_goal_joint_tolerance(0.00001)
        self.move_group.set_planning_time(10.0) # in seconds


        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = move_group.get_planning_frame()
        self.eef_link = move_group.get_end_effector_link()
        self.group_names = robot.get_group_names()

    def write_valid_joint_points(self, plan_num=50):
        workbook = xlsxwriter.Workbook(
            '/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/joint_values.xlsx')
        worksheet = workbook.add_worksheet()
        worksheet.write(1, 0, 'No.')
        worksheet.write(1, 1, 'joint_0')
        worksheet.write(1, 2, 'joint_1')
        worksheet.write(1, 3, 'joint_2')
        worksheet.write(1, 4, 'joint_3')
        worksheet.write(1, 5, 'joint_4')
        worksheet.write(1, 6, 'joint_5')

        joint_0 = np.arange(-180.0, 180.0, 5)
        joint_1 = np.arange(-127.5, 127.5, 5)
        joint_2 = np.arange(-152.5, 152.5, 5)
        joint_3 = np.arange(-270.0, 270.0, 2)
        joint_4 = np.arange(-121.0, 121.0, 2)
        joint_5 = np.arange(-270.0, 270.0, 2)

        i = plan_num
        p = 2
        num = 1
        while (i != 0):
            print('i: ', i)
            j0 = np.random.choice(joint_0, 1)[0]
            j1 = np.random.choice(joint_1, 1)[0]
            j2 = np.random.choice(joint_2, 1)[0]
            j3 = np.random.choice(joint_3, 1)[0]
            j4 = np.random.choice(joint_4, 1)[0]
            j5 = np.random.choice(joint_5, 1)[0]
            joint = (j0, j1, j2, j3, j4, j5)
            plan = self.move_robot_joints(np.array(joint))
            if plan == True:
                print('Valid joint added: ', num)
                worksheet.write(p, 0, num)
                worksheet.write(p, 1, j0)
                worksheet.write(p, 2, j1)
                worksheet.write(p, 3, j2)
                worksheet.write(p, 4, j3)
                worksheet.write(p, 5, j4)
                worksheet.write(p, 6, j5)
                num += 1
                p += 1
            i -= 1
        workbook.close()

    def move_ef_position(self, xPose, yPose, zPose):
        wpose = self.move_group.get_current_pose().pose
        wpose.position.x = xPose
        wpose.position.y = yPose
        wpose.position.z = zPose
        self.move_group.set_pose_target(wpose)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        return plan

    def plan_from_pose_json(self, pose_json_path):
        pose_file = open(pose_json_path)
        data = json.load(pose_file)
        
        for pose in data:
            pose_dict = {}
            pose_dict['position'] = [float(p) for p in data[pose]["position (x,y,z)"]]
            orientation = [float(p) for p in data[pose]["orintation (w,x,y,z)"]]
            pose_dict['orientation'] = [orientation[1], orientation[2], orientation[3], orientation[0]]
            move_robot(self, pose_dict)
            start_arv_image_acquisition(pose = int(pose))
        pass
    
    def plan_box_param(self, library = 'arv'):
        enter = input("Hit ENTER if you want to start planning: ")
        wpose = self.move_group.get_current_pose().pose
        initial_pose = get_pose(self, euler=True)
        pose_list = get_calib_poses_new(self, initial_pose)
        pose = 1
        json_dict = {}
        file_name = "/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/board_param_poses.json"
        if enter == '':
            for i in range(len(pose_list)):
                # Move to first point automatically
                if library == 'cvb':
                    cam_streamer.start_cvb_image_acquisition(pose)
                elif library == 'tis':
                    start_tis_image_acquisition(self, pose)
                elif library == 'arv':
                    start_arv_image_acquisition(self, pose)
                    pass
                print('Pose: ', pose)
                json_dict = self.write_json(json_dict, pose, file_name)
                motion_successful = move_robot(self, pose_list[i])
                pose += 1

    def write_json(self, json_dict, pose, file_name):
        json_dict[pose] = {}
        current_pose = self.move_group.get_current_pose().pose.position
        current_orientation = self.move_group.get_current_pose().pose.orientation
        json_dict[pose]['position (x,y,z)'] = [str(current_pose.x), str(current_pose.y), str(current_pose.z)]
        json_dict[pose]['orintation (w,x,y,z)'] = [str(current_orientation.w), str(current_orientation.x), str(current_orientation.y), str(current_orientation.z)]
        json_dict[pose]['joint_goal'] = [str(a) for a in self.move_group.get_current_joint_values()]

        # Serializing json
        json_object = json.dumps(json_dict, indent=4)
        # Writing to sample.json
        with open(file_name, "w") as outfile:
            outfile.write(json_object)

        return json_dict          

    def plan_cluster_point_goal(self, plan_num=10, library = 'arv'):
        enter = input("Hit ENTER if you want to start planning: ")
        ### new
        common_focus = [-61, -19, 117, 112, 5, -247]
        plan = self.move_robot_joints(np.array(common_focus))
        
        wpose = self.move_group.get_current_pose().pose
        pointx = np.array((wpose.position.x))
        pointy = np.array((wpose.position.y))
        pointz = np.array((wpose.position.z))
        newX, newY, newZ = create_points(pointx, pointy, pointz, planNum=plan_num)
        pointx = np.array(newX)
        pointy = np.array(newY)
        pointz = np.array(newZ)
        pose = 1
        json_dict = {}
        if library == 'cvb':
            cam_streamer = CamStreamer(-1)
        if enter == '':
            for j in range(pointx.shape[0]):
                plan = self.move_ef_position(pointx[j], pointy[j], pointz[j])
                initial_pose = get_pose(self, euler=True)
                pose_list = get_calib_poses(self, initial_pose, point_num=j)
                for i in range(len(pose_list)):
                    # Move to first point automatically
                    if library == 'cvb':
                        cam_streamer.start_cvb_image_acquisition(pose)
                    elif library == 'tis':
                        start_tis_image_acquisition(self, pose)
                    elif library == 'arv':
                        start_arv_image_acquisition(self, pose)
                        pass
                    print('Pose: ', pose)
                    json_dict[pose] = {}
                    current_pose = self.move_group.get_current_pose().pose.position
                    current_orientation = self.move_group.get_current_pose().pose.orientation
                    json_dict[pose]['position (x,y,z)'] = [str(current_pose.x), str(current_pose.y), str(current_pose.z)]
                    json_dict[pose]['orintation (w,x,y,z)'] = [str(current_orientation.w), str(current_orientation.x), str(current_orientation.y), str(current_orientation.z)]
                    json_dict[pose]['joint_goal'] = [str(a) for a in self.move_group.get_current_joint_values()]

                    motion_successful = move_robot(self, pose_list[i])
                    pose += 1
                
                    # Serializing json
                    json_object = json.dumps(json_dict, indent=4)
                    # Writing to sample.json
                    with open("/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/poses_10august.json", "w") as outfile:
                        outfile.write(json_object)
                plan = self.move_ef_position(pointx[j], pointy[j], pointz[j])

        ### new
        # pointx = np.array((0.096))
        # pointy = np.array((-0.25))
        # pointz = np.array((0.95))

        # newX, newY, newZ = create_points(pointx, pointy, pointz, planNum=plan_num)
        # t = np.arange(7, plan_num+7)
        # view3D(new_x=newX, new_y=newY, new_z=newZ, newtext=t)
        # pointx = np.append(pointx, newX)
        # pointy = np.append(pointy, newY)
        # pointz = np.append(pointz, newZ)

        # workbook = xlsxwriter.Workbook(
        #     '/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/EF_pose.xlsx')
        # worksheet = workbook.add_worksheet()
        # worksheet.write(0, 0, 'Pose No.')
        # worksheet.write(0, 1, 'xPose')
        # worksheet.write(0, 2, 'yPose')
        # worksheet.write(0, 3, 'zPose')
        # initial_pose = get_pose(self, euler=True)
        # pose_list = get_calib_poses(self, initial_pose)
        # pose = 1
        # if library == 'cvb':
        #     cam_streamer = CamStreamer(-1)
        # if enter == '':
        #     for j in range(pointx.shape[0]):
        #         print('Pose: ', pose)
        #         # print ('EF position: ', pointx[j], pointy[j], pointz[j])
        #         # plan = self.move_ef_position(pointx[j], pointy[j], pointz[j])
        #         # worksheet.write(j+1, 0, pose)
        #         # worksheet.write(j + 1, 1, pointx[j])
        #         # worksheet.write(j + 1, 2, pointy[j])
        #         # worksheet.write(j + 1, 3, pointz[j])
        #         # if library == 'cvb':
        #         #     cam_streamer.start_cvb_image_acquisition(pose)
        #         # elif library == 'tis':
        #         #     start_tis_image_acquisition(self, pose)
        #         # elif library == 'arv':
        #         #     start_arv_image_acquisition(self, pose)
                
        #         # pose = self.plan_diff_orientation(pose)
        #         # joint_goal = self.move_group.get_current_joint_values()
        #         # self.move_group.go(joint_goal, wait=True)
        #         start_arv_image_acquisition(self, pose)
        #         initial_pose = get_pose(self, euler=True) # orientation in euler
        #         # move_robot(self, initial_pose, extrinsic_euler=False) 
        #         pose += 1
        #         pose_list = get_calib_poses(self, initial_pose)
        #         for i in range(len(pose_list)):
        #             # Move to first point automatically
        #             motion_successful = move_robot(self, pose_list[i])
        #             start_arv_image_acquisition(self, pose)
        #             print('Pose: ', pose)
        #             pose += 1
            
        # workbook.close()

    # pose = 1

    def plan_diff_orientation(self, pose):
        """
        https://gitlab.lrz.de/autotron-group/oguz_raptor/-/blob/master/pose_estimation/calibration_and_data_acquisition/data_robot_mover.py 
        """
        prev_joint_values = self.move_group.get_current_joint_values()
        new_joint_values = np.rad2deg(prev_joint_values)
        j = -90
        for j in range(-90, 270, 20):
            print('Pose: ', pose)
            new_joint_values[5] = j
            plan = self.move_robot_joints(new_joint_values)
            start_arv_image_acquisition(self, pose)
            pose += 1
        j = -80
        prev = new_joint_values
        for j in range(-80, 80, 20):
            print('Pose: ', pose)
            new_joint_values[4] = j
            plan = self.move_robot_joints(new_joint_values)
            start_arv_image_acquisition(self, pose)
            pose += 1
        plan = self.move_robot_joints(prev)
        return pose

    def plan_xlxs_joint_goal(self, row_end, row_start=3, library = 'arv'):
        enter = input("Hit ENTER if you want to start planning: ")
        if library == 'cvb':
            cam_streamer = CamStreamer(-1)
        if enter == '':
            # pose = 1
            for j in range(row_start-1, row_end):
                prev_joint_values = self.move_group.get_current_joint_values()
                pose, joint = read_from_xlsx(row=j)
                plan = self.move_robot_joints(np.array(joint))
                new_joint_values = self.move_group.get_current_joint_values()
                if library == 'cvb':
                    cam_streamer.start_cvb_image_acquisition(pose)
                elif library == 'tis':
                    start_tis_image_acquisition(self, pose)
                elif library == 'arv':
                    start_arv_image_acquisition(self, pose)
                print('Pose: ', pose)
                xPose, yPose, zPose = self.print_pose()
                write_cartesian_position(j, (xPose, yPose, zPose))
                # pose += 1

    def move_robot_joints(self, angles, relative=False):
        """
        Relative movements possible (get current position and move relative to it).import os
import xlrd, xlsxwriter
import numpy as np
import plotly.graph_objects as go


def make_directory(path):
  isExist = os.path.exists(path)
  if not isExist:

    # Create a new directory because it does not exist
    os.makedirs(path)
    print("The new directory is created!")

# Input row as int
def read_from_xlsx(row):
  # Open the Workbook
  workbook = xlrd.open_workbook('/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/joint_values.xlsx')

  # Open the worksheet
  worksheet = workbook.sheet_by_index(0)

  joint = []
  for j in range(1, 7):
    joint.append(worksheet.cell_value(row, j))
  pose = int(worksheet.cell_value(row, 0))
  print('joint values of Pose ', pose, ' : ', joint)

  return pose, joint


def write_cartesian_position(row, position):
  workbook = xlsxwriter.Workbook(
            '/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/joint_values.xlsx')
  worksheet = workbook.add_worksheet()
  worksheet.write(row, 11, (str(position[0])+','+str(position[1])+','+str(position[2])))
  # workbook.close()

def create_points(planNum):
  xBound = np.arange(-0.15, 0.11, 0.02)
  yBound = np.arange(-0.31, 0, 0.02)
  zBound = np.arange(0.75, 1.1, 0.02)

  i = planNum
  x = []
  y = []
  z = []
  while (i != 0):
    print('i: ', i)
    px = np.random.choice(xBound, 1)[0]
    py = np.random.choice(yBound, 1)[0]
    pz = np.random.choice(zBound, 1)[0]
    x.append(px)new_joint_values
    i -= 1

  return x, y, z

def view3D(new_x, new_y, new_z, newtext):
  # Helix equation
  # t = np.linspace(0, 10, 50)
  pointx = np.array((-0.02, 0.11, 0.11, 0, 0.096, -0.12))
  pointy = np.array((-0.019, - 0.31, -0.17, -0.3, -0.25, -0.25))
  pointz = np.array((1.09, 0.94, 0.98, 0.95, 0.95, 0.78))
  # x, y, z = np.cos(t), np.sin(t), t

  all_fig = []
  fig1 = go.Scatter3d(new_joint_values
    x=np.array((0)),
    y=np.array((0)),
    z=np.array((0)),
    mode='markers',
    marker=dict(
      size=8,
      color='rgb(255,0,0)'
    )
  )
  all_fig.append(fig1)
  fig2 = go.Scatter3d(
    x=pointx,
    y=pointy,
    z=pointz,
    mode='markers',
    marker=dict(
      size=4,
      color='rgb(0,0,255)'
    ),
    text=['Cam233', 'Cam643', 'Cam241', 'Cam237', 'Cam218', 'Cam642']
  )
  all_fig.append(fig2)
  camx = np.array((1.2, 1, 1.2, 1.2, 1.2, -0.1))
  camy = np.array((0.14, -1.4, .14, -1.2, 0.14, -1.4))
  camz = np.array((0.62, 0.67, 1, 1, 0.1, 0.67))
  fig3 = go.Scatter3d(
    x=camx,
    y=camy,
    z=camz,
    mode='markers',
    marker=dict(
      size=6,
      color='rgb(0,255,0)'
    ),
    text=['Cam218', 'Cam233', 'Cam237', 'Cam241', 'Cam642', 'Cam643']
  )
  all_fig.append(fig3)

  # new_x, new_y, new_z = create_points()
  fig4 = go.Scatter3d(
    x=np.array(new_x),
    y=np.array(new_y),
    z=np.array(new_z),
    mode='markers',
    marker=dict(
      size=4,
      color='rgb(0,255,255)'
    ),
    text=list(newtext)
  )
  all_fig.append(fig4)

  final_layout = go.Figure(all_fig)

  final_layout.show()

  print("end")

        Moves the joints of the robot.

        Args:
        - angles (np, 6):       joint Euler angles for joint 1..6;
                                if you don't want to change a joint then set its value to None;
        - relative (bool):      if True: moves joints relative to their current value;
                                absolute values are used otherwise;

        Returns
        - plan (bool):          True if motion was successful

        """
        # get the current joint values from the robot
        joint_goal = self.move_group.get_current_joint_values()
        if relative == True:  # TODO fix euler to rad problem
            # move joints relative to current joint position
            # check if input is within joint limits
            if -180.0 <= np.rad2deg(joint_goal[0]) + angles[0] <= 180.0 and -127.5 <= np.rad2deg(joint_goal[1]) + \
                    angles[1] <= 127.5 and \
                    -152.5 <= np.rad2deg(joint_goal[2]) + angles[2] <= 152.5 and -270.0 <= np.rad2deg(joint_goal[3]) + \
                    angles[3] <= 270.0 and \
                    -121.0 <= np.rad2deg(joint_goal[4]) + angles[4] <= 121.0 and -270.0 <= np.rad2deg(joint_goal[5]) + \
                    angles[5] <= 270.0:
                # leave the old value if the new value is None
                joint_goal[0] = joint_goal[0] + np.deg2rad(angles[0]) if angles[0] != None else joint_goal[0]
                joint_goal[1] = joint_goal[1] + np.deg2rad(angles[1]) if angles[1] != None else joint_goal[1]
                joint_goal[2] = joint_goal[2] + np.deg2rad(angles[2]) if angles[2] != None else joint_goal[2]
                joint_goal[3] = joint_goal[3] + np.deg2rad(angles[3]) if angles[3] != None else joint_goal[3]
                joint_goal[4] = joint_goal[4] + np.deg2rad(angles[4]) if angles[4] != None else joint_goal[4]
                joint_goal[5] = joint_goal[5] + np.deg2rad(angles[5]) if angles[5] != None else joint_goal[5]
            else:
                raise ValueError('Joint angle limits exceeded. Limits: [+-180, +-127.5, +-152.5, +-270, +-121, +-270]')
        else:
            # move joints in absolute angle values
            # check if input is within joint limits
            if -180.0 <= angles[0] <= 180.0 and -127.5 <= angles[1] <= 127.5 and -152.5 <= angles[2] <= 152.5 and \
                    -270.0 <= angles[3] <= 270.0 and -121.0 <= angles[4] <= 121.0 and -270.0 <= angles[5] <= 270.0:
                # leave the old value if the new value is None
                joint_goal[0] = np.deg2rad(angles[0]) if angles[0] != None else joint_goal[0]
                joint_goal[1] = np.deg2rad(angles[1]) if angles[1] != None else joint_goal[1]
                joint_goal[2] = np.deg2rad(angles[2]) if angles[2] != None else joint_goal[2]
                joint_goal[3] = np.deg2rad(angles[3]) if angles[3] != None else joint_goal[3]
                joint_goal[4] = np.deg2rad(angles[4]) if angles[4] != None else joint_goal[4]
                joint_goal[5] = np.deg2rad(angles[5]) if angles[5] != None else joint_goal[5]
            else:
                raise ValueError(
                    'Only the following values are allowed: [+-180, +-127.5, +-152.5, +-270, +-121, +-270]')
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        plan = self.move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

        return plan

    def find_attached(self, box_name='raptor_box', stl_name='test_stl'):
        scene = self.scene
        attached_objects = scene.get_attached_objects([box_name])
        is_box_attached = len(attached_objects.keys()) > 0
        attached_objects = scene.get_attached_objects([stl_name])
        is_stl_attached = len(attached_objects.keys()) > 0

        if is_box_attached:
            return 1
        elif is_stl_attached:
            return 2
        else:
            return 0

    def check_if_attached(self, box_name='raptor_box', stl_name='test_stl'):
        scene = self.scene
        attached_objects = scene.get_attached_objects([box_name])
        is_attached = len(attached_objects.keys()) > 0

        if is_attached:
            print('Box with name %s already build' % box_name)
            return True
        else:
            return False

    def check_dimensions(self, size=(0.2, 0.2, 0.2), box_name='raptor_box'):
        scene = self.scene
        attached_objects = scene.get_attached_objects([box_name])
        is_attached = len(attached_objects.keys()) > 0
        if is_attached:
            primitives = attached_objects[box_name].object.primitives
            dims = primitives[0].dimensions
            dimensions = (float(dims[0]), float(dims[1]), float(dims[2]))
            if (dimensions == size):
                # print('Size of new and old box are equal --> no replacement necessary')
                return True

        return False

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=10, box_name='raptor_box'):
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Receieved
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node dies before publishing a collision object update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False

    def add_box(self, size=(0.2, 0.2, 0.3), box_name='raptor_box', offset=0.01, timeout=5):
        scene = self.scene

        # Without this delay adding the box only works in very few cases
        # Has apparently something to do with ros connections in the background
        rospy.sleep(2)

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "tool0"
        box_pose.pose.orientation.w = 1.0
        # ANpassung nach der Groesse der Box

        # Tutorial for boxes of size 0.1
        # box_pose.pose.position.z = 0.07 # slightly above the end effector
        box_pose.pose.position.z = size[2] / 2 + offset

        box_name = "raptor_box"
        scene.add_box(box_name, box_pose, size)

        # self.box_name=box_name
        print('Box added')
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def add_obstacle(self, name):

        obstacle_dict = {"camera_wall_1": {"orientation": [1.0, 1.2, 0, 1], "size": (0.1, 1.5, 2)},
                         "camera_wall_2": {"orientation": [1.0, 0, -1.4, 1], "size": (1.5, 0.1, 2)},
                         "glass_wall": {"orientation": [1.0, -1, 0, 1], "size": (0.1, 1.5, 2)},
                         "left_wall": {"orientation": [1.0, 0, 1, 1], "size": (1.5, 0.1, 2)},
                         "base": {"orientation": [1.0, 0, 0, -0.6], "size": (1, 1, 0.1)},
                         "roof": {"orientation": [1.0, 0, 0, 2.0], "size": (1, 1, 0.1)}}

        scene = self.scene
        rospy.sleep(2)
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.orientation.w = obstacle_dict[name]["orientation"][0]
        box_pose.pose.position.x = obstacle_dict[name]["orientation"][1]
        box_pose.pose.position.y = obstacle_dict[name]["orientation"][2]
        box_pose.pose.position.z = obstacle_dict[name]["orientation"][3]

        size = obstacle_dict[name]["size"]
        box_name = name

        scene.add_box(box_name, box_pose, size)
        print(name + ' Collision Object added')
        return self.wait_for_state_update(box_is_known=True, timeout=5, box_name=box_name)


    def attach_box(self, box_name='raptor_box', timeout=5):

        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        # Aenderung am 6.10.2021: interne Kollisionserkennung
        # grasping_group = 'manipulator'
        # touch_links = robot.get_link_names(group=grasping_group)

        touch_links = 'tool0'
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def add_stl(self, stl_name='test_stl', filename="Baugruppe.stl", size=(0.001, 0.001, 0.001), timeout=5):
        scene = self.scene

        # Without this delay adding the box only works in very few cases
        # Has apparently something to do with ros connections in the background
        rospy.sleep(2)

        stl_pose = geometry_msgs.msg.PoseStamped()
        stl_pose.header.frame_id = "tool0"
        quat = tf.transformations.quaternion_from_euler(0.37, -0.4, 0.09)
        stl_pose.pose.orientation.x = quat[0]
        stl_pose.pose.orientation.y = quat[1]
        stl_pose.pose.orientation.z = quat[2]
        stl_pose.pose.orientation.w = quat[3]
        stl_pose.pose.position.z -= 0.75
        stl_pose.pose.position.x += 0.2
        stl_pose.pose.position.y += 0.3
        # ANpassung nach der Groesse der Box
        # print (size)
        path = roslib.packages.get_pkg_dir('move_group') + '/src/' + filename

        # Tutorial for boxes of size 0.1
        # box_pose.pose.position.z = 0.07 # slightly above the end effector
        # stl_name = "test_stl"
        scene.add_mesh(stl_name, stl_pose, path, size)

        # self.box_name=box_name
        print('stl added')
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_stl(self, stl_name='test_stl', filename='Baugruppe.stl', size=(1, 1, 1), timeout=5):

        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        stl_pose = geometry_msgs.msg.PoseStamped()
        stl_pose.header.frame_id = "tool0"
        if filename == 'Baugruppe.stl':
            quat = tf.transformations.quaternion_from_euler(0.37, -0.4, 0.09)
            stl_pose.pose.orientation.x = quat[0]
            stl_pose.pose.orientation.y = quat[1]
            stl_pose.pose.orientation.z = quat[2]
            stl_pose.pose.orientation.w = quat[3]
            stl_pose.pose.position.z -= 0.75
            stl_pose.pose.position.x += 0.2
            stl_pose.pose.position.y += 0.3
        else:
            stl_pose.pose.position.z += 0.001
            quat = tf.transformations.quaternion_from_euler(0, 0, (45 / 360) * 2 * pi)
            stl_pose.pose.orientation.x = quat[0]
            stl_pose.pose.orientation.y = quat[1]
            stl_pose.pose.orientation.z = quat[2]
            stl_pose.pose.orientation.w = quat[3]

        path = roslib.packages.get_pkg_dir('move_group') + '/src/Baugruppe/' + filename

        # Aenderung vom 6.10.2021
        # grasping_group = 'manipulator'
        # touch_links = robot.get_link_names(group=grasping_group)

        touch_links = 'tool0'
        scene.attach_mesh(eef_link, stl_name, stl_pose, path, size, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, box_name='raptor_box', timeout=10):
        scene = self.scene
        eef_link = self.eef_link
        scene.remove_attached_object(eef_link, box_name)

        # TODO: Add maxfailcount
        success = False
        count = 0
        while not success and count < 5:
            scene.remove_attached_object(eef_link, name=box_name)
            success = True
            success = self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)
            count += 1
            print("Remove successfull: " + str(success) + " Type: " + str(type(success)) + str(count))

        # We wait for the planning scene to update.
        # return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)
        return True

    def remove_box(self, box_name='raptor_box', timeout=5):
        # box_name = self.box_name
        scene = self.scene
        scene.remove_world_object(box_name)
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

    def print_pose(self):
        move_group = self.move_group
        wpose = move_group.get_current_pose().pose
        xPose = wpose.position.x
        yPose = wpose.position.y
        zPose = wpose.position.z
        print("X-Pos: " + str(xPose))
        print("Y-Pos: " + str(yPose))
        print("Z-Pos: " + str(zPose))
        return xPose, yPose, zPose

    def replace_box(self, stl=False, size=(0.2, 0.2, 0.2), stl_file_name='Baugruppe.stl'):
        necessary = True
        if (self.find_attached() == 1):  # Box

            if (self.check_dimensions(size) and not stl):
                print('Old box has same dimensions --> No Replacement')
                necessary = False
            else:
                print('Old box will be replaced')
                self.detach_box()
                print('detached')
                self.remove_box()
                print('removed')
        elif (self.find_attached() == 2):  # STL
            print('Old Stl file gets detached and removed')
            self.detach_box('test_stl')
            self.remove_box('test_stl')

        if stl:
            # self.add_stl()
            self.attach_stl('test_stl', stl_file_name)

        if necessary and not size == (0, 0, 0) and not stl:
            if self.add_box(size):
                self.attach_box()
                print("Box should be attached right now!")
            else:
                self.remove_box()
                print("Error at Box attachement")
