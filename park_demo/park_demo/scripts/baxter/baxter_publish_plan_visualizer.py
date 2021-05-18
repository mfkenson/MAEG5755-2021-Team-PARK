#!/usr/bin/env python


import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
import moveit_msgs


def fill_goal_message_array(frame_id, position, orientation):
    goal = PoseStamped()
    goal.header.frame_id = frame_id
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = position[0]
    goal.pose.position.y = position[1]
    goal.pose.position.z = position[2]
    goal.pose.orientation.x = orientation[0]
    goal.pose.orientation.y = orientation[1]
    goal.pose.orientation.z = orientation[2]
    goal.pose.orientation.w = orientation[3]
    return goal


def moveit_baxter_example():
    joint_state_topic = ['joint_states:=/robot/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    rospy.init_node('moveit_baxter_example', anonymous=True)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("left_arm")
    group.clear_pose_targets()

    # Planning to a Pose goal
    left_current_pose = group.get_current_pose(end_effector_link='left_gripper').pose
    print(left_current_pose)
    #left_target_pose = left_current_pose
    left_target_pose = fill_goal_message_array('world', [0.79, 0.2, 0.885], [-0.06, 0.703, -0.0666, 0.7039])
    left_target_pose = fill_goal_message_array('world', [0, 0.5, 0.6], [-0.06, 0.703, -0.0666, 0.7039])

    group.set_pose_target(left_target_pose, end_effector_link='left_gripper')
    #group.set_pose_target(right_target_pose, end_effector_link='right_gripper')

    plan = group.plan()

    #if not plan.joint_trajectory.points:
    #    print ("[ERROR] No trajectory found")
    #else:
    #    group.plan(wait=True)
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan[1])
    # Publish
    display_trajectory_publisher.publish(display_trajectory);
    # When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)


if __name__ == '__main__':
    try:
        moveit_baxter_example()
    except rospy.ROSInterruptException:
        pass
