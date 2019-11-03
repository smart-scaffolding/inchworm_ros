#!/usr/bin/python
""" This program subscribes and publishes to ros topics and creates a base robot
class that can be later created for other controllers
    # Add more documentation here.
"""

__author__ = "Dhruv Kool Rajamani"
__email__ = "dkoolrajamani@wpi.edu"
__copyright__ = "Copyright 2019"
__date__ = "25 October 2019"

import rospy
from std_msgs.msg import Float64  # , String
from sensor_msgs.msg import JointState

import numpy as np

class Inchworm(object):
  """ Default class to subscribe and publish to inchworm robots """

  def __init__(self, namespace="/", timestep=0.01):
    """ Initialize the class with the namespace and timestep as params """

    super(Inchworm, self).__init__()
    self._default_pub_rate = 100
    self._namespace = namespace
    self._timestep = timestep

    self._n_joints = int(rospy.get_param(namespace + "n_joints"))
    self._joint_names = [
      rospy.get_param(namespace + "joint/{}".format(i))
      for i in range(self._n_joints)
    ]

    self._M = np.zeros([self._n_joints, self._n_joints])
    self._C = np.zeros([self._n_joints, self._n_joints])
    self._G = np.zeros([self._n_joints, 1])

    # Init joint states to zero
    self._joint_states = [[1.0, 2.0, 3.0] for i in range(self._n_joints)]

    self.subToJointStates()

    # Setup effort publishers
    self._effort_flag = bool(rospy.get_param(namespace + "is_effort_enabled"))
    if self._effort_flag:
      self._joint_effort_pubs = [
        rospy.Publisher(
          (
            namespace +
            "control/config/joint_effort_controller_joint_{}/command"
            .format(joint_i)
          ),
          Float64,
          queue_size=10
        ) for joint_i in range(self._n_joints)
      ]
    else:
      self._joint_effort_pubs = [
        rospy.Publisher(
          (
            namespace +
            "control/config/joint_position_controller_joint_{}/command"
            .format(joint_i)
          ),
          Float64,
          queue_size=10
        ) for joint_i in range(self._n_joints)
      ]

    # Publish initial efforts to reach home position?
    # Publishing random values for now
    for i in range(self._n_joints):
      cmd = 0
      self._joint_effort_pubs[i].publish(cmd)

    return

  def jointStatesCb(self, data):
    """ Joint States Callback """

    for joint_i, joint in enumerate(self._joint_names):
      index = data.name.index(joint)
      msg = [data.position[index], data.velocity[index], data.effort[index]]
      self._joint_states[joint_i] = msg

    return

  def subToJointStates(self):
    """ Subscribe to joint state messages if published """

    self._joint_states_sub = rospy.Subscriber(
      self._namespace + "joint_states",
      JointState,
      self.jointStatesCb
    )

    return

  def publishJointEfforts(self, cmd=None, effort=True):
    """ Publish cmd[arr] values to joints """

    # if effort:
    if cmd is None:
      cmd = [0 for i in range(self._n_joints)]

      for i in range(self._n_joints):
        cmd[i] = np.sin(rospy.get_time() * np.pi / 2) * 1
        print str(i) + " : " + str(cmd[i])
        self._joint_effort_pubs[i].publish(cmd[i])

    else:
      for i in range(self._n_joints):
        print str(i) + " : " + str(cmd[i])
        self._joint_effort_pubs[i].publish(cmd[i])

    return