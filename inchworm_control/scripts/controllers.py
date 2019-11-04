#!/usr/bin/python
""" This program generates the controllers to allow devs to control the joints
of the robot
    # Add more documentation here.
"""

__author__ = "Dhruv Kool Rajamani"
__email__ = "dkoolrajamani@wpi.edu"
__copyright__ = "Copyright 2019"
__date__ = "8 October 2019"

import time

import rospy
import roslaunch
import rospkg
# import yaml

def generate_controllers(joints, effort=False, P=100, I=0.01, D=10):
  """ Generate inchworm_control.yaml file """
  d = {}
  state_controller = {
    "joint_state_controller":
      {
        "type": "joint_state_controller/JointStateController",
        "publish_rate": 10000
      }
  }
  controllers = []
  if effort:
    controllers = [
      {
        "type": "effort_controllers/JointEffortController",
        "joint": joint
      } for joint in joints
    ]

    position_controllers = {
      "joint_effort_controller_joint_{}".format(i): controller for i,
      controller in enumerate(controllers)
    }
  else:
    controllers = [
      {
        "type": "effort_controllers/JointPositionController",
        "joint": joint,
        "pid": {
          "p": P,
          "i": I,
          "d": D
        }
      } for joint in joints
    ]

    position_controllers = {
      "joint_position_controller_joint_{}".format(i): controller for i,
      controller in enumerate(controllers)
    }

  d.update(state_controller)
  d.update(position_controllers)

  rospack = rospkg.RosPack()
  rospack.list()

  root_path = rospack.get_path('inchworm_description')
  control_config_path = root_path + "/config/inchworm_control.yaml"

  # controllers_yaml = yaml.dump({"orthosis": d}, default_flow_style=False)
  # controllers_yaml = yaml.dump(d, default_flow_style=False)
  # stream = file(control_config_path, "w+")
  # yaml.dump({"orthosis": d}, stream, default_flow_style=False)

  return d

def run_controllers(namespace, effort=False):
  """ Run ROS controllers """

  rospy.set_param(namespace + "is_effort_enabled", effort)

  joints = rospy.get_param(namespace + "parameters/control/joints/")
  controllers_yaml = generate_controllers(joints, effort=effort)
  print(controllers_yaml)
  rospy.set_param(namespace + "control/config", controllers_yaml)
  add = namespace + "control/config/"
  if effort:
    node_control = roslaunch.core.Node(
      package="controller_manager",
      node_type="spawner",
      name="controller_spawner",
      namespace=namespace,
      respawn=False,
      output="screen",
      args=(
        add + "joint_state_controller " + " ".join(
          [
            add + "joint_effort_controller_joint_{}".format(joint_i)
            for joint_i,
            joint in enumerate(joints)
          ]
        )
      )
    )
  else:
    node_control = roslaunch.core.Node(
      package="controller_manager",
      node_type="spawner",
      name="controller_spawner",
      namespace=namespace,
      respawn=False,
      output="screen",
      args=(
        add + "joint_state_controller " + " ".join(
          [
            add + "joint_position_controller_joint_{}".format(joint_i)
            for joint_i,
            joint in enumerate(joints)
          ]
        )
      )
    )
  # print(node_control.to_xml())
  nodes = [node_control]
  return nodes

def main():
  """ Main """

  launch = roslaunch.scriptapi.ROSLaunch()
  launch.start()

  effort = True

  nodes = []
  islands = rospy.get_param("islands")
  for island in range(1, 1 + islands):
    robots = rospy.get_param("/island_{}/robots".format(island))
    for robot_id in range(1, 1 + robots):
      namespace = rospy.get_param("namespace_{}_{}".format(island, robot_id))
      nodes.extend(run_controllers(namespace, effort=effort))

  # print nodes
  processes = [launch.launch(n) for n in nodes]
  processes = processes
  # Loop
  while any([process.is_alive() for process in processes]):
    time.sleep(2)
  # Close
  for process in processes:
    process.stop()
  return

if __name__ == "__main__":
  main()
