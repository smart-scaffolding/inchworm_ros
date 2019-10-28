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

import numpy as np

class TorqueControl(Inchworm):
  """ Torque controller using M, C, G matrices """

  def __init__(self, namespace='/', timestep=0.01):
    """ Initialize default timestep size """

    super(TorqueControl, self).__init__(namespace=namespace, timestep=timestep)

    rospy.Timer(rospy.Duration(self._timestep), self.control)
    return

  def control(self, event=None, verbose=False):
    if verbose:
      print(event)
    self.bodyControl()
    return

  def bodyControl(self):
    """ Body control """
    t = rospy.get_time()

    # print "Time: {}".format(t)

    self.publishJointEfforts()

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
