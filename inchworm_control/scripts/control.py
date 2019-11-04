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

import rbdl

import time

import numpy as np

class TorqueControl(Inchworm):
  """ Torque controller using M, C, G matrices """

  def __init__(self, namespace='/', timestep=0.0001):
    """ Initialize default timestep size """

    super(TorqueControl, self).__init__(namespace=namespace, timestep=timestep)

    if namespace != '/':
      lower_index = namespace.find("/island")
      upper_index = namespace.find("/inchworm")
      self._island_namespace = namespace[lower_index:upper_index]
      self._gzserver_namespace = self._island_namespace + "/gzserver/"

    # self._gzserver_URI = int(
    #     rospy.get_param(self._gzserver_namespace + "URI"))

    rospy.Timer(rospy.Duration(self._timestep), self.control)
    return

  def control(self, event=None, verbose=False):
    if verbose:
      print(event)
    self.torqueControl()
    return

  def torqueControl(self):
    """ Body control """

    Kp = 10 * self._scale
    Kv = 0.1 * self._scale

    if self._is_set_point_ctrl:
      if self._walk:
        # joint_vals = [
        #   -0.8067426250685097,
        #   1.2152737554267325,
        #   45.31001551257536
        # ]
        joint_vals = [0, 0, 0]
        self._q_des = {
          # self._joint_limits[joint][1] for joint in self._m_joints
          joint: joint_vals[i] for i,
          joint in enumerate(self._m_joints)
        }
      else:
        self._q_des = {
          joint: self._joint_limits[joint][0] for joint in self._m_joints
        }

    # Calculate Gravity
    for k, v in self._states_map.items():
      self._q[v] = self._joint_states[k][0]
      # self._qdot[v] = self._joint_states[k][1]
      self._qdot[v] = 0.0
      self._qddot[v] = 0.0
    rbdl.InverseDynamics(
      self._model,
      self._q,
      self._qdot,
      self._qddot,
      self._tau
    )
    self._G = self._tau

    tau_compensated = np.zeros(self._model.qdot_size)
    q_comp = np.zeros(self._model.q_size)
    qdot_comp = np.zeros(self._model.q_size)

    # Calculate error term and add to torque
    for k, v in self._states_map.items():
      tau_compensated[v] = self._G[v]
      if self._is_set_point_ctrl:
        q_comp[v] = Kp * (self._q_des[k] - self._joint_states[k][0])
        qdot_comp[v] = -Kv * self._joint_states[k][1]
        tau_compensated[v] += q_comp[v] + qdot_comp[v]

    if not self._is_set_point_ctrl:
      print("Tau = " + str(tau_compensated.transpose()))

    self.publishJointEfforts(effort=True, cmd=tau_compensated)

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
