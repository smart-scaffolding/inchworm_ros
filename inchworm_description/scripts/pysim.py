#!/usr/bin/python
""" This script spawns multiple models based on the number of islands and robots specified.

"""

__author__ = "Dhruv Kool Rajamani"
__email__ = "dkoolrajamani@wpi.edu"
__copyright__ = "Copyright 2019"
__date__ = "8 October 2019"

from urdf_parser import parse_model
import time
import rospy
import rospkg
import roslaunch
import subprocess
import os
import sys

from spawn import spawn_node
from generate_urdf import get_continous_joints, generate_robot, generate_urdf
from generate_urdf import get_all_continuous_joints_names, add_gazebo_plugins
from generate_urdf import add_transmission_plugins, add_rgbd_camera
from urdf_parser import get_all_joints, get_p_and_c_links, scale_robot_inertia

def setupRobots(islandNamespace, robotNamespace, island_id, robot_id):
  """ Setup nodes and params """
  islandNs = islandNamespace + "_{}/".format(island_id)
  island = "island_{}".format(island_id)
  namespace = islandNs + robotNamespace + "_{}/".format(robot_id)

  rospy.set_param("namespace_{}_{}".format(island_id, robot_id), namespace)

  rospack = rospkg.RosPack()
  rospack.list()

  root_path = rospack.get_path('inchworm_description')
  unscaled_model_path = root_path + "/urdf/inchworm_description_unscaled.urdf"
  model_path = root_path + "/urdf/inchworm_description.urdf"

  robot = parse_model(unscaled_model_path)

  scale = 1
  # tof_cam_link = "base_plate"
  # (robot, camera_link_name) = add_rgbd_camera(robot=robot,
  #                                             scale=scale,
  #                                             parent_link_name=tof_cam_link)
  # camera_link_name = tof_cam_link + "_camera"
  robot = scale_robot_inertia(robot, scale)

  rospy.set_param(namespace + "scale", scale)

  # print robot.to_xml_string()

  if os.path.isfile(model_path):
    os.remove(model_path)

  with open(model_path, "w+") as f:
    f.write(robot.to_xml_string())
    f.close()

  # print(robot.to_xml_string())

  robot = add_transmission_plugins(robot)

  robot = add_gazebo_plugins(
    robot=robot,
    namespace=namespace,
    use_ft_sensors=True,
    use_p3d_sensors=True,
    fix_base_joint={
      "child": "link_1",
      "origin": {
        "xyz": [0,
                0,
                0],
        "rpy": [0,
                0,
                0]
      }
    }
  )

  robot.name = robotNamespace

  # print(robot.to_xml_string())

  rospy.set_param(namespace + "robot_description", robot.to_xml_string())

  # log_path = rospack.get_path('inverted-pendulum_analysis')

  # rospy.set_param(
  #     "log_path",
  #     log_path
  # )

  joints = get_all_joints(robot)

  for joint in robot.joints:
    if joint.limit is not None:
      rospy.set_param(
        namespace + "joint/limits/{}".format(joint.name),
        [
          joint.limit.lower,
          joint.limit.upper,
          joint.limit.velocity,
          joint.limit.effort
        ]
      )

  rospy.set_param(namespace + "all_joints", joints)

  mov_joints = [
    joint for typ,
    joint in joints.items() if typ in ['revolute',
                                       'continuous',
                                       'prismatic']
  ]

  m_joints = []
  for ls in mov_joints:
    if ls is not None:
      for item in ls:
        m_joints.append(item)
    else:
      continue

  for typ, joints in joints.items():
    if joints is not None:
      for joint_i, joint in enumerate(joints):
        rospy.set_param(
          namespace + "joints/{}/{}".format(typ,
                                            joint_i),
          get_p_and_c_links(robot,
                            joint)
        )

  arg = namespace + "n_joints"
  n_joints = len(m_joints)
  rospy.set_param(arg, n_joints)

  # cam_robot = cam_generate_robot(namespace)
  # rospy.set_param(
  #         namespace + "camera_description",
  #         cam_robot.to_xml_string()
  # )

  # for joint_i, joint in enumerate(m_joints):
  #     rospy.set_param(
  #         )
  rospy.set_param(namespace + "parameters/control/joints/", m_joints)

  node_state = roslaunch.core.Node(
    package="robot_state_publisher",
    node_type="state_publisher",
    name="robot_state_publisher",
    namespace=namespace,
    machine_name=island
  )

  # i = 0
  # args_rqt_plot = ""
  # for i in range(n_joints):
  #   args_rqt_plot += args_rqt.format(i)
  # node_rqt_plot_pos = roslaunch.core.Node(
  #   package="rqt_plot",
  #   node_type="rqt_plot",
  #   name="JointStatePos_plot",
  #   namespace=namespace,
  #   output="screen",
  #   args=(args_rqt_plot)
  # )

  i = 0
  args_rqt_pos = namespace + "joint_states/position[{}] "
  args_rqt = namespace + "joint_states/effort[{}] "
  args_rqt_plot = ""
  args_rqt_pos_plot = ""
  for i in range(n_joints):
    args_rqt_plot += args_rqt.format(i)
    args_rqt_pos_plot += args_rqt_pos.format(i)
  node_rqt_plot = roslaunch.core.Node(
    package="rqt_plot",
    node_type="rqt_plot",
    name="JointState_plot",
    namespace=namespace,
    output="screen",
    args=(args_rqt_plot + args_rqt_pos_plot)
  )

  spawnNodes = spawn_node(
    islandNamespace=islandNs,
    namespace=namespace,
    island_id=island_id,
    robot_id=robot_id,
    model_location=[0,
                    robot_id - 1,
                    0],
    robot_name=robotNamespace
    # , camera_location=[1.2, (robot_id), 0.2]
  )
  # robot node
  spawnRobotNode = spawnNodes[0]

  # # camera node
  # # spawnCameraNode = spawnNodes[1]

  # node_tf = roslaunch.core.Node(
  #   package="tf",
  #   node_type="static_transform_publisher",
  #   name="base_link_to_world_tf_pub",
  #   args="0 0 0 0 0 0 world base_link 1000",
  #   namespace=namespace,
  #   machine_name=island
  # )

  nodes = [
    spawnRobotNode,
    # spawnCameraNode,
    node_state
    # node_tf
    # ,
    # node_rqt_plot
  ]

  is_set_point_ctrl = rospy.set_param(namespace + "set_point_enable", True)
  walk = rospy.set_param(namespace + "default_conf", True)

  return nodes

def setupIslands(islandNamespace, island_id, robots):
  """ Setup nodes and params """
  nodes = []
  # Load model pramaters
  namespace = islandNamespace + "_{}/".format(island_id)

  island = "{}_{}".format("island", island_id)

  gazebo_env_args = "http://localhost:1134{}".format(4 + island_id)
  ros_gazebo_env_args = "/{}".format(island)

  gz_URI = 11344 + island_id

  rospy.set_param(ros_gazebo_env_args + "/URI", gz_URI)

  node_gzserver = roslaunch.core.Node(
      package="gazebo_ros",
      node_type="gzserver",
      name="gzserver",
      respawn=False,
      namespace=namespace,
      machine_name=island,
      args="--verbose",  # switch to bullet if shared_msgs can be used
      env_args=[("GAZEBO_MASTER_URI", gazebo_env_args),
                ("ROS_NAMESPACE", ros_gazebo_env_args)])
  node_gzclient = roslaunch.core.Node(
    package="gazebo_ros",
    node_type="gzclient",
    name="gzclient",
    namespace=namespace,
    respawn=True,
    machine_name=island,
    env_args=[
      ("GAZEBO_MASTER_URI",
       gazebo_env_args),
      ("ROS_NAMESPACE",
       ros_gazebo_env_args)
    ]
  )
  arg_gazebo = "call --wait {}/pause_physics".format(namespace + "gzserver")
  print(arg_gazebo)
  node_gzstate = roslaunch.core.Node(
    # rosservice call gazebo/pause_physics
    package="rosservice",
    node_type="rosservice",
    name="gazebo_properties",
    namespace=namespace,
    args=(arg_gazebo)
  )
  nodes.append(node_gzserver)
  nodes.append(node_gzclient)
  # nodes.append(node_gzstate)

  model_name = "inchworm"

  rospy.set_param(namespace + "robots", robots)

  for robot_id in range(1, robots + 1):
    nodes.extend(setupRobots(islandNamespace, model_name, island_id, robot_id))

  return nodes

def launch_sim(islands, robots):
  """ Launch simulation with Gazebo """
  nodes = []
  processes = []
  launch = roslaunch.scriptapi.ROSLaunch()
  launch.start()

  rospy.set_param("islands", islands)

  for island_id in range(1, islands + 1):
    robot = robots[island_id - 1]
    nodes.extend(setupIslands("/island", island_id, robot))

  for node in nodes:
    print(node.to_xml())

  processes = [launch.launch(node) for node in nodes]

  # Loop
  while any([process.is_alive() for process in processes]):
    time.sleep(2)
  # Close
  for process in processes:
    process.stop()
  return

def main():
  """ Main """

  # @param1 number of islands
  # @param2 number of robots per island
  # @example: launch_sim(2, [1, 2]) : island 1 has 1 model and island 2 has
  #  2 models

  islands = 1
  robots = [1]

  if len(sys.argv) > 1:
    if sys.argv[1] is not None:
      islands = int(sys.argv[1])

    if sys.argv[2] is not None:
      robots = []
      robots_str = sys.argv[2]
      robots_str_split = robots_str.split(',')
      print robots_str_split
      for rob in robots_str_split:
        print rob
        print type(rob)
        robots.extend([int(rob)])

  launch_sim(islands, robots)
  return

if __name__ == '__main__':
  main()
