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
from std_msgs.msg import Float64, String, Header
from sensor_msgs.msg import JointState
from urdf_parser_py import urdf
from tf import TransformListener
from ambf_msgs.msg import ObjectState, ObjectCmd

import rbdl
import rospkg

import numpy as np


class Inchworm(object):
  """ Default class to subscribe and publish to orthosis robots """

  def __init__(self, ambf_flag, namespace="/", timestep=0.01):
    """ Initialize the class with the namespace and timestep as params """

    super(Inchworm, self).__init__()
    self._default_pub_rate = 1 / timestep
    self._namespace = namespace
    self._timestep = timestep
    self._ambf_flag = ambf_flag

    self.base_types = ['fixed', 'floating']
    self.base_id = True

    self._is_set_point_ctrl = True

    self._is_joints_init = False

    if not ambf_flag:
      self.tf = TransformListener()

      self._is_set_point_ctrl = bool(
        rospy.get_param(self._namespace + "set_point_enable")
      )

      self._walk = bool(rospy.get_param(self._namespace + "default_conf"))
      self._n_joints = int(rospy.get_param(self._namespace + "n_joints"))
      # Movable joints
      self._m_joints = rospy.get_param(
        self._namespace + "parameters/control/joints"
      )
      # self._n_joints = len(self._m_joints)

      self._m_joints_dict = {
        typ: rospy.get_param(self._namespace + "all_joints/{}".format(typ))
        for typ in urdf.Joint.TYPES
      }

      self._base_type_joint_names = []
      self._unknown_joint_names = []
      for k, v in self._m_joints_dict.items():
        if len(v) < 1:
          del self._m_joints_dict[k]
        else:
          if k == self.returnBaseType():
            self._base_type_joint_names = v
            del self._m_joints_dict[k]
          elif k == self.returnBaseType(opposite=True) or k == 'unknown':
            self._unknown_joint_names = v
            del self._m_joints_dict[k]

      # del self._floating_and_unknown_joint_names[0]

      self._links = {}
      for typ, joints in self._m_joints_dict.items():
        for joint_i, joint in enumerate(joints):
          if joint not in self._links.keys():
            self._links[joint] = rospy.get_param(
              self._namespace + "joints/{}/{}".format(typ,
                                                      joint_i)
            )

      self._base_type_links_and_joints = {
        joint: rospy.get_param(
          self._namespace +
          "joints/{}/{}".format(self.returnBaseType(),
                                joint_i)
        ) for joint_i,
        joint in enumerate(self._base_type_joint_names)
      }

      self._scale = rospy.get_param(self._namespace + "scale")

      self._joint_limits = {
        joint:
        rospy.get_param(self._namespace + "joint/limits/{}".format(joint))
        for joint in self._m_joints
      }

      self.subToJointStates()

    else:
      self._walk = False
      self._scale = 100

      self._n_joints = 3
      self._m_joints = ['' for i in range(self._n_joints)]
      # Can be made generic

    # Init joint states to zero
    self._joint_states = {
      joint: [0.0,
              0.0,
              0.0] for joint_i,
      joint in enumerate(self._m_joints)
    }

    self.subToJointStates()
    self.setupPublishers()

    self._M = np.zeros([self._n_joints, self._n_joints])
    self._C = np.zeros([self._n_joints, self._n_joints])

    # Publish initial efforts to reach home position?
    if self._ambf_flag:
      self.publishJointEfforts(init=True)
    else:
      self.initializeRBDLModel()

    return

  def returnBaseType(self, opposite=False):
    if not opposite:
      if not self.base_id:
        return self.base_types[0]
      else:
        return self.base_types[1]
    else:
      if not self.base_id:
        return self.base_types[1]
      else:
        return self.base_types[0]

  def setupPublishers(self):

    if not self._ambf_flag:
      # Setup effort publishers
      self._effort_flag = bool(
        rospy.get_param(self._namespace + "is_effort_enabled")
      )
      if self._effort_flag:
        self._joint_effort_pubs = {
          joint: rospy.Publisher(
            (
              self._namespace +
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
              self._namespace +
              "control/config/joint_position_controller_joint{}/command"
              .format(joint_i)
            ),
            Float64,
            queue_size=10
          ) for joint_i,
          joint in enumerate(self._m_joints)
        }
    else:
      self._joint_effort_pubs = rospy.Publisher(
        self._namespace + "link_1/Command",
        ObjectCmd,
        queue_size=10
      )

    return

  def jointStatesCb(self, data):
    """ Joint States Callback """

    if self._ambf_flag and not self._is_joints_init:
      self._m_joints = data.joint_names
      self._n_joints = len(self._m_joints)
      self._is_joints_init = True

    for joint_i, joint in enumerate(self._m_joints):
      if not self._ambf_flag:
        index = data.name.index(joint)
        msg = [data.position[index], data.velocity[index], data.effort[index]]
      else:
        msg = [data.joint_positions[joint_i], 0.0, 0.0]

      self._joint_states[joint] = msg

    return

  def subToJointStates(self):
    """ Subscribe to joint state messages if published """

    if not self._ambf_flag:
      self._joint_states_sub = rospy.Subscriber(
        self._namespace + "link_1/State",
        ObjectState,
        self.jointStatesCb
      )
    else:
      self._joint_states_sub = rospy.Subscriber(
        self._namespace + "joint_states",
        JointState,
        self.jointStatesCb
      )

    return

  def publishJointEfforts(self, cmd=None, effort=True, init=False):
    """ Publish cmd[arr] values to joints """
    # if effort:
    if not self._ambf_flag:
      if cmd is None:
        cmd = [0 for i, (k, v) in enumerate(self._joint_effort_pubs.items())]

        for i, (k, v) in enumerate(self._joint_effort_pubs.items()):
          cmd[i] = 0.01 * np.sin(rospy.get_time() * np.pi / 2) * 1
          print str(i) + " : " + str(cmd[i])
          self._joint_effort_pubs[k].publish(cmd[i])

      else:
        for i, c in enumerate(cmd):
          k = self._states_map.keys()[self._states_map.values().index(i)]
          print '{}: {}'.format(k, cmd[i])
          self._joint_effort_pubs[k].publish(cmd[i])
    else:
      if init:
        enable_position_controller = True
        position_controller_mask = [True, True, True]
        cmd = [0 for i, v in enumerate(self._m_joints)]
      if cmd is None:
        cmd = [0 for i, v in enumerate(self._m_joints)]

        for i, v in enumerate(self._m_joints):
          cmd[i] = 10 * np.sin(rospy.get_time() * np.pi / 2) * 1
          # print str(i) + " : " + str(cmd[i])

      # else:
      #   for i, c in enumerate(cmd):
      #     k = self._states_map.keys()[self._states_map.values().index(i)]
      #     self._joint_effort_pubs[k].publish(cmd[i])

      cmd_msg = ObjectCmd()
      header = Header()
      header.stamp = rospy.Time.now()
      cmd_msg.header = header
      cmd_msg.enable_position_controller = enable_position_controller
      cmd_msg.position_controller_mask = position_controller_mask
      cmd_msg.joint_cmds = cmd
      self._joint_effort_pubs.publish(cmd_msg)

    return

  def initializeRBDLModel(self):
    """ Load the URDF model using RBDL """

    rospack = rospkg.RosPack()
    rospack.list()

    root_path = rospack.get_path('inchworm_description')
    model_path = root_path + "/urdf/inchworm_description.urdf"
    # Create a new model
    self._model = rbdl.loadModel(
      model_path,
      kwargs={
        "floating_base": self.base_id,
        "verbose": True
      }
    )

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

    self._unique_links = list()
    for link in self._links.values():
      if len(self._unique_links) == 0:
        self._unique_links.append(link[0])
      else:
        for i in range(2):
          if link[i] in self._unique_links:
            continue
          else:
            self._unique_links.append(link[i])

    flajv = self._base_type_links_and_joints.values()
    flajv_ = []
    for i in flajv:
      flajv_.append(i[1])

    for i, link in enumerate(self._unique_links):
      if link in flajv_:
        del self._unique_links[i]

    self._bodies_map = {
      link: (
        self._model.GetBodyId(link),
        self._model.mBodies[self._model.GetBodyId(link)]
      ) for link in self._unique_links
    }

    self._end_effector_positions = []
    for frames in self._base_type_links_and_joints.values():
      if self.tf.frameExists(frames[0]) and self.tf.frameExists(frames[1]):
        t = self.tf.getLatestCommonTime(frames[0], frames[1])
        position, quaternion = self.tf.lookupTransform(frames[0], frames[1], t)
        self._end_effector_positions.extend(position)

    # print self._end_effector_positions

    return
