#!/usr/bin/env python
""" Add Gazebo plugins to urdf generation """

import rospy
import rospkg

from urdf_parser_py import urdf
import lxml.etree as etree
import yaml

from urdf_parser import get_all_joints

import numpy as np

name = "orthosis"
n_links = 2

n_len = 2.0 / n_links
n_offset = 0.5 / n_links

l_f_rad = 0.05
l_f_len = 2.2

l_link_len = n_len
l_link_rad = 0.05

use_ftSensors = True


def orange():
  col_orange = urdf.Material(
    name="orange",
    color=urdf.Color([1.0,
                      0.423529411765,
                      0.0392156862745,
                      1.0])
  )
  return col_orange


def black():
  col_black = urdf.Material(name="black", color=urdf.Color([0, 0, 0, 1.0]))
  return col_black


def white():
  col_white = urdf.Material(
    name="white",
    color=urdf.Color([1.0,
                      1.0,
                      1.0,
                      1.0])
  )
  return col_white


def frame_inertial(scale=1):
  """ Return default inertial """
  _mass = 0.1 * scale
  _x = (0.033 / 2)
  _y = (0.022 / 2)
  _z = (0.103 / 2)
  ixx = _mass * _x * _x
  iyy = _mass * _y * _y
  izz = _mass * _z * _z
  return urdf.Inertial(
    mass=_mass,
    inertia=urdf.Inertia(ixx=ixx,
                         iyy=iyy,
                         izz=izz),
    origin=urdf.Pose(xyz=[_x,
                          _y,
                          _z],
                     rpy=[0,
                          0,
                          0])
  )


def frame_collision(scale=1):
  """ Return default collision """
  # if geometry is None:
  _x = (0.033/2) * scale
  _y = (0.022/2) * scale
  _z = (0.103/2) * scale
  geometry = urdf.Cylinder(length=_z, radius=_x)
  # if origin is None:
  origin = urdf.Pose(xyz=[0, 0, 0], rpy=[0, 0, 0])
  return urdf.Collision(geometry=geometry, origin=origin)


def frame_visual(scale=10):
  """ Return default collision """
  # if geometry is None:
  _x = (0.033/2) * scale
  _y = (0.022/2) * scale
  _z = (0.103/2) * scale
  # geometry = urdf.Box(size=[_x, _y,
  #                           _z])
  geometry = urdf.Cylinder(length=_z, radius=_x)
  # if origin is None:
  origin = urdf.Pose(xyz=[0, 0, 0], rpy=[0, 0, 0])
  return urdf.Visual(geometry=geometry, origin=origin, material=orange())


def default_inertial():
  """ Return default inertial """
  return urdf.Inertial(
    mass=1,
    inertia=urdf.Inertia(ixx=1,
                         iyy=1,
                         izz=1),
    origin=urdf.Pose(xyz=[0,
                          0,
                          n_len / 2],
                     rpy=[0,
                          0,
                          0])
  )


def default_collision(geometry=None, origin=None):
  """ Return default collision """
  if geometry is None:
    geometry = urdf.Cylinder(radius=0.05, length=n_len)
  if origin is None:
    origin = urdf.Pose(xyz=[0, 0, n_len / 2], rpy=[0, 0, 0])
  return urdf.Collision(geometry=geometry, origin=origin)


def default_visual(geometry=None, origin=None):
  if geometry is None:
    geometry = urdf.Cylinder(radius=0.05, length=n_len)
  if origin is None:
    origin = urdf.Pose(xyz=[0, 0, n_len / 2], rpy=[0, 0, 0])
  return urdf.Visual(
    geometry=geometry,
    origin=origin,
    material=urdf.Material("Gazebo/Black")
  )


def default_dynamics(damping=None, friction=None):
  if damping is None:
    damping = 0.7
  if friction is None:
    friction = 0.
  return urdf.JointDynamics(damping=damping, friction=friction)


def generate_robot(namespace):
  """ Generate Inverted Pendulum URDF """
  # Robot
  robot = urdf.Robot(name)

  # Base
  l_world = urdf.Link("world")
  robot.add_link(l_world)

  # Frame
  frame_geometry = urdf.Cylinder(radius=l_f_rad, length=l_f_len)

  l_frame = urdf.Link("l_frame")
  l_frame.collision = frame_collision(geometry=frame_geometry)
  l_frame.inertial = frame_inertial()
  l_frame.visual = frame_visual(geometry=frame_geometry)
  robot.add_link(l_frame)

  # World Joint
  j_world = urdf.Joint(
    name="fixed",
    parent="world",
    child="l_frame",
    joint_type="fixed",
    origin=urdf.Pose([0,
                      0,
                      0],
                     [0,
                      0,
                      0])
  )
  robot.add_joint(j_world)

  # Links
  l_link = [urdf.Link("l_{}".format(i)) for i in range(n_links)]
  j_link = [urdf.Joint("j_{}".format(i)) for i in range(n_links)]
  for i in range(n_links):
    l_y = (0.1 if i == 0 else 2 * l_link_rad)
    l_z = ((l_f_len - 0.05) if i == 0 else (l_link_len - 0.05))
    l_pos = urdf.Pose(xyz=[0, l_y, l_z], rpy=[0, 0, 0])
    l_link[i].visual = default_visual()
    l_link[i].inertial = default_inertial()
    l_link[i].collision = default_collision()
    robot.add_link(l_link[i])

    j_link[i] = urdf.Joint(
      name="j_{}".format(i),
      parent=("l_frame" if i == 0 else "l_{}".format(i - 1)),
      child="l_{}".format(i),
      origin=l_pos,
      joint_type="continuous",
      axis=[0,
            1,
            0],
      dynamics=default_dynamics()
    )
    robot.add_joint(j_link[i])

  for joint in j_link:
    # Transmission
    t = urdf.Transmission(joint.name + "_trans")
    t.type = "transmission_interface/SimpleTransmission"
    transjoint = urdf.TransmissionJoint(name=joint.name)
    transjoint.add_aggregate("hardwareInterface", "EffortJointInterface")
    t.add_aggregate("joint", transjoint)
    actuator = urdf.Actuator(joint.name + "_motor")
    actuator.mechanicalReduction = 1
    t.add_aggregate("actuator", actuator)
    robot.add_aggregate("transmission", t)

  # joint force-torque sensors
  if use_ftSensors:
    joints = get_continous_joints(robot)
    for joint_i, joint in enumerate(joints):
      # Provide feedback
      a = etree.Element("gazebo", {"reference": joint.name})
      b = etree.SubElement(a, "provideFeedback")
      b.text = "true"
      robot.add_aggregate("gazebo", a)
      # Sensor
      a = etree.Element("gazebo")
      b = etree.SubElement(
        a,
        "plugin",
        {
          "name": "force_torque_plugin_" + joint.name,
          "filename": "libgazebo_ros_ft_sensor.so"
        }
      )
      c = etree.SubElement(b, "alwaysOn")
      c.text = "true"
      d = etree.SubElement(b, "updateRate")
      d.text = "100"
      d = etree.SubElement(b, "jointName")
      d.text = joint.name
      d = etree.SubElement(b, "topicName")
      d.text = namespace + "ftSensors/" + joint.name
      robot.add_aggregate("gazebo", a)

  # Gazebo Color Plugin for all links
  for link_i, link in enumerate(get_all_links(robot)):
    a = etree.Element("gazebo", {"reference": link.name})
    b = etree.SubElement(a, "material")
    b.text = "Gazebo/Black"
    robot.add_aggregate("gazebo", a)

  # Gazebo Color Plugin for Frame
  a = etree.Element("gazebo", {"reference": "l_frame"})
  b = etree.SubElement(a, "material")
  b.text = "Gazebo/Orange"
  robot.add_aggregate("gazebo", a)

  # Gazebo plugin
  a = etree.Element("gazebo")
  b = etree.SubElement(
    a,
    "plugin",
    {
      "name": "gazebo_ros_control",
      "filename": "libgazebo_ros_control.so"
    }
  )
  c = etree.SubElement(b, "robotNamespace")
  c.text = namespace
  # d = etree.SubElement(b, "robotParam")
  # d.text = "/robot_description"
  robot.add_aggregate("gazebo", a)

  return robot


def add_transmission_plugins(robot, effort=True):

  joints = get_all_joints(robot)

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

  for joint in m_joints:
    # Transmission
    t = urdf.Transmission(joint + "_trans")
    t.type = "transmission_interface/SimpleTransmission"
    transjoint = urdf.TransmissionJoint(name=joint)
    transjoint.add_aggregate(
      "hardwareInterface",
      "hardware_interface/EffortJointInterface"
    )
    t.add_aggregate("joint", transjoint)
    actuator = urdf.Actuator(joint + "_motor")
    actuator.mechanicalReduction = 1
    t.add_aggregate("actuator", actuator)
    robot.add_aggregate("transmission", t)

  return robot


def add_rgbd_camera(robot, scale=1, parent_link_name="base_plate"):
  _x = (0.033 / 2)
  _y = (0.022 / 2)
  _z = (0.103 / 2)

  l_frame = urdf.Link(
    name=parent_link_name + "_camera",
    visual=frame_visual(),
    collision=frame_collision(),
    inertial=frame_inertial()
  )
  l_frame.visual = frame_visual()
  l_frame.collision = frame_collision()
  robot.add_link(l_frame)

  for link in robot.links:
    if link.name == parent_link_name:
      origin_xyz = link.inertial.origin.xyz
      origin_xyz[2] += _z
      origin_rpy = link.inertial.origin.rpy

  j_world = urdf.Joint(
    name=l_frame.name + "_joint",
    parent=parent_link_name,
    child=l_frame.name,
    joint_type="fixed",
    origin=urdf.Pose(origin_xyz,
                     origin_rpy)
  )
  robot.add_joint(j_world)
  return (robot, l_frame.name)


def add_gazebo_plugins(
  robot,
  namespace="/",
  use_ft_sensors=False,
  use_p3d_sensors=False,
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
  },
  use_rgbd_camera=None
):

  joints = get_all_joints(robot)

  if fix_base_joint is not None:
    l_world = urdf.Link("world")
    robot.add_link(l_world)
    origin = fix_base_joint.get("origin", 0)
    if origin is not 0:
      origin_xyz = origin.get("xyz")
      origin_rpy = origin.get("rpy")

    else:
      origin_xyz = [0, 0, 0]
      origin_rpy = [0, 0, 0]

    child_frame = fix_base_joint.get("child", 0)

    if child_frame is 0:
      child_frame = "base_link"

    fixed_or_floating = 'fixed'
    j_world = urdf.Joint(
      name=fixed_or_floating,
      parent="world",
      child=child_frame,
      joint_type=fixed_or_floating,
      origin=urdf.Pose(origin_xyz,
                       origin_rpy)
    )
    robot.add_joint(j_world)

  for link_i, link in enumerate(get_all_links(robot)):
    a = etree.Element("gazebo", {"reference": link.name})
    b = etree.SubElement(a, "material")
    b.text = "Gazebo/Grey"
    robot.add_aggregate("gazebo", a)

  if use_ftSensors:
    joints = get_continous_joints(robot)
    for joint_i, joint in enumerate(joints):
      # Provide feedback
      a = etree.Element("gazebo", {"reference": joint.name})
      b = etree.SubElement(a, "provideFeedback")
      b.text = "true"
      c = etree.SubElement(a, "implicitSpringDamper")
      c.text = "1"
      robot.add_aggregate("gazebo", a)
      # Sensor
      a = etree.Element("gazebo")
      b = etree.SubElement(
        a,
        "plugin",
        {
          "name": "force_torque_plugin_" + joint.name,
          "filename": "libgazebo_ros_ft_sensor.so"
        }
      )
      c = etree.SubElement(b, "alwaysOn")
      c.text = "true"
      d = etree.SubElement(b, "updateRate")
      d.text = "100"
      d = etree.SubElement(b, "jointName")
      d.text = joint.name
      d = etree.SubElement(b, "topicName")
      d.text = namespace + "ftSensors/" + joint.name
      robot.add_aggregate("gazebo", a)

  if use_rgbd_camera is not None:
    link_name = use_rgbd_camera.get("link")
    camera_name = link_name + "_tof"

    camera_frame_name = camera_name + "_optical_frame"
    camera_link = urdf.Link(camera_frame_name)
    robot.add_link(camera_link)

    j_camera_frame = urdf.Joint(
      name=camera_frame_name + "_joint",
      parent=link_name,
      child=camera_frame_name,
      joint_type="fixed",
      origin=urdf.Pose(xyz=[0,
                            0,
                            0],
                       rpy=[0,
                            0,
                            -np.pi / 2])
    )  #-np.pi / 2
    robot.add_joint(j_camera_frame)

    # TOF Sensor
    a = etree.Element(
      "gazebo",
      {"reference": camera_frame_name}
    )  # Try and automate this
    b = etree.SubElement(a, "sensor", {"name": camera_name, "type": "ray"})
    b1 = etree.SubElement(b, "visualize")
    b1.text = "true"
    c1 = etree.SubElement(b, "always_on")
    c1.text = "true"
    c = etree.SubElement(b, "update_rate")
    c.text = "20.0"
    d = etree.SubElement(b, "ray")
    e = etree.SubElement(d, "scan")
    g = etree.SubElement(e, "horizontal")
    h1 = etree.SubElement(g, "samples")
    h1.text = "16"
    h2 = etree.SubElement(g, "resolution")
    h2.text = "1"
    h3 = etree.SubElement(g, "min_angle")
    h3.text = str(-1.20428 / 2)
    h4 = etree.SubElement(g, "max_angle")
    h4.text = str(1.20428 / 2)
    i = g = etree.SubElement(e, "vertical")
    v1 = etree.SubElement(i, "samples")
    v1.text = "16"
    v2 = etree.SubElement(i, "resolution")
    v2.text = "1"
    v3 = etree.SubElement(i, "min_angle")
    v3.text = str(-0.890118 / 2)
    v4 = etree.SubElement(i, "max_angle")
    v4.text = str(0.890118 / 2)
    j = etree.SubElement(d, "range")
    k = etree.SubElement(j, "min")
    k.text = "0.2"
    l = etree.SubElement(j, "max")
    l.text = "5.0"
    n = etree.SubElement(
      b,
      "plugin",
      {
        "name": camera_name + "_plugin",
        "filename": "libsim_tof.so"
      }
    )
    o = etree.SubElement(n, "alwaysOn")
    o.text = "true"
    p = etree.SubElement(n, "updateRate")
    p.text = "20.0"
    q = etree.SubElement(n, "topicName")
    q.text = namespace + camera_name + "/depth/points"
    r = etree.SubElement(n, "frameName")
    r.text = camera_frame_name
    s = etree.SubElement(n, "zone")
    t1 = etree.SubElement(s, "id")
    t1.text = "0"
    t2 = etree.SubElement(s, "startX")
    t2.text = "0"
    t3 = etree.SubElement(s, "startY")
    t3.text = "0"
    t4 = etree.SubElement(s, "endX")
    t4.text = "16"
    t5 = etree.SubElement(s, "endY")
    t5.text = "16"

    # Camera
    u = etree.SubElement(
      a,
      "sensor",
      {
        "name": camera_frame_name,
        "type": "camera"
      }
    )
    v = etree.SubElement(u, "update_rate")
    v.text = "20.0"
    w = etree.SubElement(u, "camera")  #, {"name": link_name + "_image"})
    x = etree.SubElement(w, "horizontal_fov")
    x.text = str(1.20428)
    y = etree.SubElement(w, "image")
    y1 = etree.SubElement(y, "width")
    y1.text = "640"
    y2 = etree.SubElement(y, "height")
    y2.text = "480"
    y3 = etree.SubElement(y, "format")
    y3.text = "R8G8B8"
    z = etree.SubElement(w, "clip")
    z1 = etree.SubElement(z, "near")
    z1.text = "0.2"
    z2 = etree.SubElement(z, "far")
    z2.text = "5"
    pg = etree.SubElement(
      u,
      "plugin",
      {
        "name": link_name + "_controller",
        "filename": "libgazebo_ros_camera.so"
      }
    )
    pg1 = etree.SubElement(pg, "alwaysOn")
    pg1.text = "true"
    pg2 = etree.SubElement(pg, "updateRate")
    pg2.text = "0.0"
    pg3 = etree.SubElement(pg, "cameraName")
    pg3.text = link_name
    pg4 = etree.SubElement(pg, "imageTopicName")
    pg4.text = namespace + link_name + "/image_raw"
    pg5 = etree.SubElement(pg, "cameraInfoTopicName")
    pg5.text = namespace + link_name + "/camera_info"
    pg6 = etree.SubElement(pg, "frameName")
    pg6.text = camera_frame_name
    pg7 = etree.SubElement(pg, "distortionK1")
    pg7.text = "0.00000001"
    pg8 = etree.SubElement(pg, "distortionK2")
    pg8.text = "0.00000001"
    pg9 = etree.SubElement(pg, "distortionK3")
    pg9.text = "0.00000001"
    pg10 = etree.SubElement(pg, "distortionT1")
    pg10.text = "0.00000001"
    pg11 = etree.SubElement(pg, "distortionT2")
    pg11.text = "0.00000001"
    pg12 = etree.SubElement(pg, "hackBaseline")
    pg12.text = "0"

    robot.add_aggregate("gazebo", a)

  # Gazebo plugin
  a = etree.Element("gazebo")
  b = etree.SubElement(
    a,
    "plugin",
    {
      "name": "gazebo_ros_control",
      "filename": "libgazebo_ros_control.so"
    }
  )
  c = etree.SubElement(b, "robotNamespace")
  c.text = namespace
  # d = etree.SubElement(b, "robotParam")
  # d.text = "/robot_description"
  robot.add_aggregate("gazebo", a)

  return robot


def generate_urdf(robot):
  """ Save robot URDF file """
  xml_text = robot.to_xml_string()
  print(xml_text)
  with open("inverted-pendulum.urdf", "w+") as urdf_file:
    urdf_file.write(xml_text)
  return


def get_continous_joints(robot):
  """ Get robot revolute joints """
  return [joint for joint in robot.joints if joint.type == "continuous"]


def get_all_continuous_joints_names(robot):
  """ Get all joints of the robot """
  typ = "continuous"
  joints = {
    typ: [joint.name for joint in robot.joints if joint.type == "continuous"]
  }
  return joints


def get_all_links(robot):
  """ Get all link names of the robot """
  return [link for link in robot.links if link.name != "base_link"]


def main():
  """ Main """
  robot = generate_robot("inverted_pendulum")
  generate_urdf(robot)
  return


if __name__ == "__main__":
  main()
