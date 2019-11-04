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
from urdf_parser_py import urdf

import rbdl
import rospkg

import numpy as np

class Inchworm(object):
  """ Default class to subscribe and publish to orthosis robots """

  def __init__(self, namespace="/", timestep=0.01):
    """ Initialize the class with the namespace and timestep as params """

    super(Inchworm, self).__init__()
    self._default_pub_rate = 10000
    self._namespace = namespace
    self._timestep = timestep

    self._is_set_point_ctrl = bool(
      rospy.get_param(namespace + "set_point_enable")
    )
    self._walk = bool(rospy.get_param(namespace + "default_conf"))

    self._n_joints = int(rospy.get_param(namespace + "n_joints"))

    # Movable joints
    self._m_joints = rospy.get_param(namespace + "parameters/control/joints")
    # self._n_joints = len(self._m_joints)

    self._m_joints_dict = {
      typ: rospy.get_param(namespace + "all_joints/{}".format(typ))
      for typ in urdf.Joint.TYPES
    }

    for k, v in self._m_joints_dict.items():
      if len(v) < 1 or k == 'fixed' or k == 'floating' or k == 'unknown':
        del self._m_joints_dict[k]

    self._links = {}
    for typ, joints in self._m_joints_dict.items():
      for joint_i, joint in enumerate(joints):
        if joint not in self._links.keys():
          self._links[joint] = rospy.get_param(
            namespace + "joints/{}/{}".format(typ,
                                              joint_i)
          )

    self.initializeRBDLModel()

    self._M = np.zeros([self._n_joints, self._n_joints])
    self._C = np.zeros([self._n_joints, self._n_joints])

    self._scale = rospy.get_param(namespace + "scale")

    # Init joint states to zero
    self._joint_states = {
      joint: [1.0,
              2.0,
              3.0] for joint_i,
      joint in enumerate(self._m_joints)
    }

    self._joint_limits = {
      joint: rospy.get_param(namespace + "joint/limits/{}".format(joint))
      for joint in self._m_joints
    }

    # print self._joint_limits

    self.subToJointStates()

    # Setup effort publishers
    self._effort_flag = bool(rospy.get_param(namespace + "is_effort_enabled"))
    if self._effort_flag:
      self._joint_effort_pubs = {
        joint: rospy.Publisher(
          (
            namespace +
            "control/config/joint_effort_controller_joint_{}/command"
            .format(joint_i)
          ),
          Float64,
          queue_size=10
        ) for joint_i,
        joint in enumerate(self._m_joints)
      }
    else:
      self._joint_effort_pubs = {
        joint: rospy.Publisher(
          (
            namespace +
            "control/config/joint_position_controller_joint{}/command"
            .format(joint_i)
          ),
          Float64,
          queue_size=10
        ) for joint_i,
        joint in enumerate(self._m_joints)
      }

    # Publish initial efforts to reach home position?
    for k, v in self._joint_effort_pubs.items():
      cmd = 0
      self._joint_effort_pubs[k].publish(cmd)

    return

  def jointStatesCb(self, data):
    """ Joint States Callback """

    for joint_i, joint in enumerate(self._m_joints):
      index = data.name.index(joint)
      msg = [data.position[index], data.velocity[index], data.effort[index]]
      self._joint_states[joint] = msg

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
      cmd = [0 for i, (k, v) in enumerate(self._joint_effort_pubs)]

      for i, (k, v) in enumerate(self._joint_effort_pubs.items()):
        cmd[i] = 0.01 * np.sin(rospy.get_time() * np.pi / 2) * 1
        # print str(i) + " : " + str(cmd[i])
        self._joint_effort_pubs[k].publish(cmd[i])

    else:
      for i, c in enumerate(cmd):
        k = self._states_map.keys()[self._states_map.values().index(i)]
        self._joint_effort_pubs[k].publish(cmd[i])

    return

  def initializeRBDLModel(self):
    """ Load the URDF model using RBDL """

    rospack = rospkg.RosPack()
    rospack.list()

    root_path = rospack.get_path('inchworm_description')
    model_path = root_path + "/urdf/inchworm_description.urdf"
    # Create a new model
    self._model = rbdl.loadModel(model_path)

    self._q = np.zeros(self._model.q_size)
    self._qdot = np.zeros(self._model.qdot_size)
    self._qddot = np.zeros(self._model.qdot_size)
    self._tau = np.zeros(self._model.qdot_size)
    self._G = np.zeros(self._model.qdot_size)

    self._states_map = {
      joint: self._model.mJoints[self._model.GetBodyId(link[1])].q_index
      for joint,
      link in self._links.items()
    }

    # print self._states_map

    return
