#!/usr/bin/env python3

# Box_attacher Code
# Author: Markus Nikol
# Still TODO:
# - detect if box can/should be added in current pose

# Usage:
# Start the robot/simulation
# at Faps: open window and use source devel/setup.bash
# then use rosrun move_group box_attacher.py for default dimensions 0.2 0.2 0.2
# or type the dimensions by starting the node with eg. rosrun move_group box_attacher.py 0.3 0.4 0.4
# for the dimensions 0.3, 0.4 and 0.4
# This program currently DOES NOT CHECK for internal collisions

import sys
import os
import copy
import time
import roslib

from numpy.core.numeric import True_
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from math import radians
import tf

from time import sleep

# print Python version 
print(sys.version)


def all_close(goal, actual, tolerance):
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class Box_Attacher_2(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self, node_name = 'box_attacher'):
    #super(MoveGroupPythonIntefaceTutorial, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    # Misc variables
    #self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = move_group.get_planning_frame()
    self.eef_link = move_group.get_end_effector_link()
    self.group_names = robot.get_group_names()
    self.attached_list = []


  def find_attached(self, obj_name):
    scene = self.scene
    attached_objects = scene.get_attached_objects([obj_name])
    return attached_objects

  def check_if_attached(self, box_name = 'raptor_box', stl_name = 'test_stl'):
    scene = self.scene
    attached_objects = scene.get_attached_objects([box_name])
    is_attached = len(attached_objects.keys()) > 0
 
    if is_attached:
      print('Box with name %s already build' % box_name)
      return True
    else:
      return False

  def check_dimensions(self, size = (0.2, 0.2, 0.2), box_name = 'raptor_box'):
    scene = self.scene
    attached_objects = scene.get_attached_objects([box_name])
    is_attached = len(attached_objects.keys()) > 0
    if is_attached:
      primitives = attached_objects[box_name].object.primitives
      dims = primitives[0].dimensions
      dimensions = (float(dims[0]), float(dims[1]), float(dims[2]))
      if (dimensions == size):
        #print('Size of new and old box are equal --> no replacement necessary')
        return True

    return False
  
  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=10, box_name = 'raptor_box'):
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

  def add_box(self, size=(0.2, 0.2, 0.3), box_name = 'raptor_box', offset = 0.01, timeout=5):
    scene = self.scene

    # Without this delay adding the box only works in very few cases
    # Has apparently something to do with ros connections in the background
    rospy.sleep(2)

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "tool0"
    box_pose.pose.orientation.w = 1.0
    # ANpassung nach der Groesse der Box
    print (size)

    # Tutorial for boxes of size 0.1
    #box_pose.pose.position.z = 0.07 # slightly above the end effector
    box_pose.pose.position.z = size[2]/2 + offset
    # box_name = "raptor_box"
    scene.add_box(box_name, box_pose, size)

    #self.box_name=box_name
    print('[SCENE] Box added')
    return self.wait_for_state_update(box_is_known=True, timeout=timeout, box_name=box_name)

  def add_box_cameras(self, name):
    scene = self.scene
    rospy.sleep(2)
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 1
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = 1

    size = (0.1, 1.5, 2)
    box_name = name

    scene.add_box(box_name, box_pose, size)
    print('[SCENE] Camera Collision Object added')
    return self.wait_for_state_update(box_is_known=True, timeout=5, box_name = box_name)

  def add_box_glas(self, name):
    scene = self.scene
    rospy.sleep(2)
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = -1
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = 1

    size = (0.1, 1.5, 2)
    box_name = name

    scene.add_box(box_name, box_pose, size)
    print('[SCENE] Glas Collision Object added')
    return self.wait_for_state_update(box_is_known=True, timeout=5, box_name = box_name)

  def attach_box(self, box_name = 'raptor_box', timeout=5):

    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    # Aenderung am 6.10.2021: interne Kollisionserkennung
    #grasping_group = 'manipulator'
    # touch_links = robot.get_link_names(group=grasping_group)

    touch_links = 'tool0'
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout, box_name = box_name)


  def add_stl(self, stl_name = 'test_stl', filename="Baugruppe.stl", size=(0.001, 0.001, 0.001) ,timeout=5):
    scene = self.scene

    # Without this delay adding the box only works in very few cases
    # Has apparently something to do with ros connections in the background
    rospy.sleep(2)

    stl_pose = geometry_msgs.msg.PoseStamped()
    stl_pose.header.frame_id = "tool0"
    quat =tf.transformations.quaternion_from_euler(0.37, -0.4, 0.09)
    stl_pose.pose.orientation.x = quat[0]
    stl_pose.pose.orientation.y = quat[1]
    stl_pose.pose.orientation.z = quat[2]
    stl_pose.pose.orientation.w = quat[3]
    stl_pose.pose.position.z -= 0.75
    stl_pose.pose.position.x += 0.2
    stl_pose.pose.position.y += 0.3
    # ANpassung nach der Groesse der Box
    #print (size)
    path = roslib.packages.get_pkg_dir('move_group')+'/src/' + filename

    # Tutorial for boxes of size 0.1
    #box_pose.pose.position.z = 0.07 # slightly above the end effector
    #stl_name = "test_stl"
    print(filename)
    scene.add_mesh(stl_name, stl_pose, path, size)

    #self.box_name=box_name
    print('[SCENE] stl added')
    return self.wait_for_state_update(box_is_known=True, timeout=timeout, box_name=stl_name)


  def attach_stl(self, stl_name = 'test_stl', filename = 'Baugruppe.stl', size=(1, 1, 1), timeout=5):

    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    stl_pose = geometry_msgs.msg.PoseStamped()
    stl_pose.header.frame_id = "tool0"
    if filename == 'Baugruppe.stl':
      quat =tf.transformations.quaternion_from_euler(0.37, -0.4, 0.09)
      stl_pose.pose.orientation.x = quat[0]
      stl_pose.pose.orientation.y = quat[1]
      stl_pose.pose.orientation.z = quat[2]
      stl_pose.pose.orientation.w = quat[3]
      stl_pose.pose.position.z -= 0.75
      stl_pose.pose.position.x += 0.2
      stl_pose.pose.position.y += 0.3
    else:
      stl_pose.pose.position.z += 0.001
      quat =tf.transformations.quaternion_from_euler(0, 0, (45/360)*2*pi)
      stl_pose.pose.orientation.x = quat[0]
      stl_pose.pose.orientation.y = quat[1]
      stl_pose.pose.orientation.z = quat[2]
      stl_pose.pose.orientation.w = quat[3]

    # path = roslib.packages.get_pkg_dir('move_group')+'/src/Baugruppe/' + filename
    path = filename
    print("[SCENE] Attaching stl from "+path)

    # Aenderung vom 6.10.2021
    #grasping_group = 'manipulator'
    #touch_links = robot.get_link_names(group=grasping_group)

    touch_links = 'tool0' 
    scene.attach_mesh(eef_link, stl_name, stl_pose, path, size, touch_links=touch_links)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout, box_name=stl_name)

  def detach_box(self, box_name = 'raptor_box', timeout=10):
    scene = self.scene
    eef_link = self.eef_link
    scene.remove_attached_object(eef_link, box_name)

    # TODO: Add maxfailcount
    success = False
    count = 0
    while not success and count < 5:
      scene.remove_attached_object(eef_link, name = box_name)
      success = True
      success = self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout, box_name=box_name)
      count += 1
      print("[SCENE] Remove successfull: " + str(success) + " Type: " + str(type(success)) + str(count))

    # We wait for the planning scene to update.
    #return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)
    return True

  def remove_box(self, box_name = 'raptor_box', timeout=5):
    #box_name = self.box_name
    scene = self.scene
    scene.remove_world_object(box_name)
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout, box_name=box_name)

  def print_pose(self):
    move_group = self.move_group
    wpose = move_group.get_current_pose().pose
    print("[SCENE] X-Pos: " + str(wpose.position.x))
    print("[SCENE] Y-Pos: " + str(wpose.position.y))
    print("[SCENE] Z-Pos: " + str(wpose.position.z))

  def replace_box(self, box_name='raptor_box', size = (0.2, 0.2, 0.2), stl_file_name = ''):    
    necessary = True

    for obj in self.attached_list:
      attached_obj = self.find_attached(obj_name=obj)
      if obj in attached_obj:
        self.detach_box(obj)
        self.remove_box(obj)
        self.attached_list.remove(obj)

    if stl_file_name:
      obj = stl_file_name.split('/')[-1].split('.')[0]
      self.attached_list.append(obj)
      self.attach_stl(stl_name=obj, filename=stl_file_name)

    if necessary and not size == (0,0,0) and not stl_file_name:
      if self.add_box(size, box_name=box_name):
        self.attach_box(box_name=box_name)
        self.attached_list.append(box_name)
        print ("[SCENE] Box should be attached right now!")
      else:
        self.remove_box(box_name=box_name)
        print ("[SCENE] Error at Box attachement")

  def build_faps_obstacles(self): 
    """"
    Adds the obstacles for the robot at FAPS. 
    Obstacles: 
    - camera wall 
    - glas wall 
    """
    self.add_box_cameras('camera2')
    self.add_box_glas('glas2')

    return 0 



def main():
  rospy.init_node('box_attacher_2_node', anonymous=True)

  ##########################################################################################
  # experiement 1 
  ##########################################################################################
  box_attacher = Box_Attacher_2() 
  # add environment obstacles 
  box_attacher.add_box_cameras('camera2')
  box_attacher.add_box_glas('glas2')
  # add gripper stl 
  filedir = os.path.dirname(os.path.realpath(__file__))
  gripper_stl_path = os.path.join(filedir, "..", "data", "gripper_calibration_plate.stl")  
  box_attacher.replace_box(stl_file_name=gripper_stl_path)

  ##########################################################################################
  # experiment 2 
  ##########################################################################################
  # try:

    # # default box size in m
    # stl = False
    # launch = False
    # size=(0.2, 0.2, 0.2)
    # myargv = rospy.myargv(argv=sys.argv)
    # stl_file_name = ''
    # # Inserting Boxsize at start
    # if (len(myargv) == 2):
    #   stl = True
    #   print('stl file will be added')
    #   #name = String(myargv[1])
    #   stl_file_name = str(myargv[1])
    # elif len(myargv) == 3:
    #   stl = True
    #   print('stl file will be added')
    #   stl_file_name = str(myargv[1])
    #   launch = True
    #   camera_name = str(myargv[2])
    # elif len(myargv) > 5:
    #   print('Fehlerhafte Eingabe')
    # elif len(myargv) == 4:
    #   print('Size of box set')
    #   size=(float(myargv[1]), float(myargv[2]), float(myargv[3]))
    # elif len(myargv) == 5:
    #   print('Size of box set')
    #   size=(float(myargv[1]), float(myargv[2]), float(myargv[3]))
    #   launch = True
    #   camera_name = str(myargv[4])
    
    # box_attacher.replace_box(stl, size, stl_file_name)

    # if launch:
    #   box_attacher.add_box_cameras('camera'+camera_name)
    #   box_attacher.add_box_glas('glas_'+camera_name)

    # debug = False
    # if debug:
    #   print(box_attacher.scene.get_attached_objects())
    #   print(box_attacher.scene.get_known_object_names())
    #   rospy.spin()
    
    # print('To prevent initialisation errors please press plan and then execute without moving the robot.')
    # print('Do not trust this program blindly. Be always prepared to stop the robot with the big red button')

  # except rospy.ROSInterruptException:
  #   return
  # except KeyboardInterrupt:
    # return
  
  return 

if __name__ == '__main__':
  main()
