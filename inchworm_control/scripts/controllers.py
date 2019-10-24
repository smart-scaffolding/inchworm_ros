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
import yaml

def generate_controllers(joints):
	""" Generate inchworm_control.yaml file """
	d = {}
	state_controller = {
		"joint_state_controller":
			{
				"type": "joint_state_controller/JointStateController",
				"publish_rate": 100
			}
	}
	controllers = [
		{
			"type": "effort_controllers/JointPositionController",
			"joint": joint,
			"pid": {
				"p": 100.0,
				"i": 0.01,
				"d": 10.0
			}
		} for joint in joints
	]
	position_controllers = {
		"joint_effort_controller_{}".format(joints[i]): controller for i,
		controller in enumerate(controllers)
	}
	d.update(state_controller)
	d.update(position_controllers)

	controllers_yaml = yaml.dump({"inchworm": d}, default_flow_style=False)
	controllers_yaml = yaml.dump(d, default_flow_style=False)
	stream = file("inchworm_control.yaml", "w+")
	yaml.dump({"inchworm": d}, stream, default_flow_style=False)
	return controllers_yaml

def run_controllers(namespace="/inchworm/"):
	""" Run ROS controllers """
	joints = rospy.get_param("/inchworm/parameters/control/joints/revolute")
	controllers_yaml = generate_controllers(joints)
	print(controllers_yaml)
	rospy.set_param(namespace + "control/config", controllers_yaml)
	add = namespace + "control/config/"
	node_control = roslaunch.core.Node(
		package="controller_manager",
		node_type="spawner",
		name="controller_spawner",
		namespace=namespace,
		respawn=False,
		output="screen",
		args=(
			add + "joint_state_controller " + " ".join(
				[add + "joint_effort_controller_" + joint for joint in joints]
			)
		)
	)
	nodes = [node_control]
	return nodes

def main():
	""" Main """
	nodes = run_controllers()
	launch = roslaunch.scriptapi.ROSLaunch()
	launch.start()
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
