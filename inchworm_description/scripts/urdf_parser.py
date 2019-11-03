#!/usr/bin/env python

# Load the urdf_parser_py manifest, you use your own package
# name on the condition but in this case, you need to depend on
# urdf_parser_py.
# import roslib;
# roslib.load_manifest('urdf_parser_py')

###########################
# Set param for number of joints
###########################

import rospy
import rospkg
import sys

# Import the module

from urdf_parser_py.urdf import URDF
from urdf_parser_py import urdf

def scale_robot_inertia(robot, scale=1):
  """ Scale the inertia's of the robots with the scale value """
  for link in robot.links:
    link.inertial.inertia.ixx = scale * link.inertial.inertia.ixx
    link.inertial.inertia.ixy = scale * link.inertial.inertia.ixy
    link.inertial.inertia.ixz = scale * link.inertial.inertia.ixz
    link.inertial.inertia.iyy = scale * link.inertial.inertia.iyy
    link.inertial.inertia.iyz = scale * link.inertial.inertia.iyz
    link.inertial.inertia.izz = scale * link.inertial.inertia.izz
    link.inertial.mass = scale * link.inertial.mass

  for joint in robot.joints:
    if joint.type == 'revolute':
      joint.limit.effort = scale * joint.limit.effort

  return robot

def get_revolute_joints(robot):
  """ Get robot revolute joints """
  return [joint for joint in robot.joints if joint.type == "revolute"]

def get_continuous_joints(robot):
  """ Get robot revolute joints """
  return [joint for joint in robot.joints if joint.type == "continuous"]

def get_joints(robot):
  """ Get robot joints """
  joints = get_revolute_joints(robot)
  # print("Joints:\n{}".format([joint.name for joint in joints]))
  return joints

def get_all_joint_names(robot):
  """ Get all joints of the robot """
  joints = {
    typ: [joint.name for joint in robot.joints if joint.type == typ
          ] for typ in urdf.Joint.TYPES
  }
  return joints

def get_all_revolute_joints_names(robot):
  """ Get all joints of the robot """
  typ = "revolute"
  joints = {
    typ: [joint.name for joint in robot.joints if joint.type == "revolute"]
  }
  return joints

def get_all_joints(robot):

  joints = {
    typ: [joint.name for joint in robot.joints if joint.type == typ
          ] for typ in urdf.Joint.TYPES
  }

  return joints

def get_all_joint_objects(robot):

  return robot.joints

def get_p_and_c_links(robot, joint):
  """ Get parent and child link from joint in robot """
  p_and_c_link = []
  for jt in robot.joints:
    if jt.name == joint:
      p_and_c_link.append(jt.parent)
      p_and_c_link.append(jt.child)
      return p_and_c_link

def get_all_joint_types(robot):
  """ Get all joint types """
  types = {
    typ: [joint.type for joint in robot.joints if joint.type == typ
          ] for typ in urdf.Joint.TYPES
  }
  return types

def ros_param_properties(robot, robot_namespace):
  """ Save robot properties """
  properties = robot_properties(robot)
  rospy.set_param(robot_namespace + "properties", properties)
  return

def robot_properties(robot):
  """ Robot properties """
  return {"jointNames": get_all_joint_names(robot)}

def parse_model(model_path):

  # 1. Parse a string containing the robot description in urdf.
  # Pro: no need to have a roscore running.
  # Cons: n/a
  # Note: it is rare to receive the robot model as a string.
  desc = open(model_path, 'r')

  robot = URDF.from_xml_string(desc.read())

  # - OR -

  # 2. Load the module from a file.
  # Pro: no need to have a roscore running.
  # Cons: using hardcoded file location is not portable.
  #robot = urdf.from_xml_file()

  # - OR -

  # 3. Load the module from the parameter server.
  # Pro: automatic, no arguments are needed, consistent
  #      with other ROS nodes.
  # Cons: need roscore to be running and the parameter to
  #      to be set beforehand (through a roslaunch file for
  #      instance).
  #robot = urdf.from_parameter_server()

  # Print the robot
  # for element in robot.links:
  #     print(element)

  joint_types = get_all_joint_types(robot)

  print(get_all_revolute_joints_names(robot))

  return robot

def main():

  rospack = rospkg.RosPack()
  rospack.list()

  root_path = rospack.get_path('inchworm_description')
  model_path = root_path + "/urdf/inchworm_description.urdf"

  if len(sys.argv) > 1:
    if sys.argv[1] is not None:
      root_path = sys.argv[1]

    if sys.argv[2] is not None:
      model_path = sys.argv[2]

  robot = parse_model(model_path)

  print(robot)

if __name__ == "__main__":
  main()
