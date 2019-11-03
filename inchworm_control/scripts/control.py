#!/usr/bin/python
""" This program subscribes and publishes to ros topics and has the controller.
    # Add more documentation here.
"""

__author__ = "Dhruv Kool Rajamani"
__email__ = "dkoolrajamani@wpi.edu"
__copyright__ = "Copyright 2019"
__date__ = "25 October 2019"

import rospy
from inchworm import Inchworm
from std_msgs.msg import Float64  # , String
from sensor_msgs.msg import JointState

import pybullet as p
import time
import pybullet_data

import numpy as np

class TorqueControl(Inchworm):
  """ Torque controller using M, C, G matrices """

  def __init__(self, namespace='/', timestep=0.01):
    """ Initialize default timestep size """

    super(TorqueControl, self).__init__(namespace=namespace, timestep=timestep)

    if namespace != '/':
      lower_index = namespace.find("/island")
      upper_index = namespace.find("/inchworm")
      self._island_namespace = namespace[lower_index:upper_index]
      self._gzserver_namespace = self._island_namespace + "/gzserver/"

    # self._gzserver_URI = int(rospy.get_param(self._gzserver_namespace + "URI"))

    # Try and establish TCP Connection with gazebo bullet physics instance
    # Currently the Gazebo/physicsmsgs doesn't sync with bullet physics' TCP
    # calls
    # physics_client = p.connect(p.TCP, "localhost", self._gzserver_URI)
    self._physics_client = p.connect(p.DIRECT)

    gravity_str = [
      self._gzserver_namespace + "gravity_x",
      self._gzserver_namespace + "gravity_y",
      self._gzserver_namespace + "gravity_z"
    ]

    p.setGravity(
      rospy.get_param(gravity_str[0]),
      rospy.get_param(gravity_str[1]),
      rospy.get_param(gravity_str[2])
    )

    # p.loadURDF(rospy.get_param(namespace + "robot_description"))

    rospy.Timer(rospy.Duration(self._timestep), self.control)
    return

  def control(self, event=None, verbose=False):
    if verbose:
      print(event)
    self.torqueControl()
    return

  def torqueControl(self):
    """ Body control """
    # t = rospy.get_time()

    # print "Time: {}".format(t)
    # Solve the dynamics here

    cmd = [0.0, 0.0, 0.0]
    q_desired = [np.pi, 0, 0]  # rads
    dq_desired = [0.0, 0.0, 0.0]

    Kp = 2
    Kv = 0.001

    cmd[0] = Kp * (q_desired[0] - self._joint_states[0][0]) + Kv * (
      dq_desired[0] - self._joint_states[0][1]
    )
    cmd[1] = Kp * (q_desired[1] - self._joint_states[1][0]) + Kv * (
      dq_desired[1] - self._joint_states[1][1]
    )
    cmd[2] = Kp * (q_desired[2] - self._joint_states[2][0]) + Kv * (
      dq_desired[2] - self._joint_states[2][1]
    )

    self.publishJointEfforts(effort=True, cmd=cmd)

    return

  def toString(self):
    """ Serializes attributes of this class into a string """

    this_string = """\nNamespace = {}\nStep size = {}\n""".format(
      self._namespace,
      self._timestep
    )

    return this_string

def generateControllerObjects():
  """ Generate objects of the controller specified """

  namespaces = []
  islands = rospy.get_param("islands")
  for island in range(1, 1 + islands):
    robots = rospy.get_param("/island_{}/robots".format(island))
    for robot_id in range(1, 1 + robots):
      namespaces.append(
        rospy.get_param("namespace_{}_{}".format(island,
                                                 robot_id))
      )

  robot_torque_controllers = {
    namespace: TorqueControl(namespace) for namespace in namespaces
  }

  return robot_torque_controllers

def main():
  """ main """

  rospy.init_node('torque_control_py', anonymous=False)
  robot_torque_controllers = generateControllerObjects()
  rospy.spin()
  del robot_torque_controllers

  return

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
