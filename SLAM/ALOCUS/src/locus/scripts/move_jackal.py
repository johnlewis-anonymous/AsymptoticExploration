#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
import argparse

def move_robot(x=1,y=1):
    # Initialize the ROS node
    rospy.init_node('move_robot_node', anonymous=True)

    # Create an action client for the move_base action server
    client = actionlib.SimpleActionClient('J1/move_base', MoveBaseAction)
    
    # Wait for the action server to start
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base action server")

    # Define the goal position and orientation
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "J1/map_locus"  # Use "base_link" to move relative to the robot's current position
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = Pose(Point(x, y, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))

    # Send the goal to the action server
    rospy.loginfo(f"Sending goal to move: {x}, Variable2: {y} in the frame {goal.target_pose.header.frame_id}")
    client.send_goal(goal)

    # Wait for the result
    client.wait_for_result()

    # Check the result
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("The robot moved 1 meter in the x direction successfully!")
    else:
        rospy.logwarn("The robot failed to move 1 meter in the x direction")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Example usage: rosrun locus move_jackal.py x y')
    parser.add_argument('x', type=float, help='x')
    parser.add_argument('y', type=float, help='y')
    args, unknown = parser.parse_known_args()
    try:
        move_robot(args.x,args.y)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")