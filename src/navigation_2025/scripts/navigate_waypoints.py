#! /usr/bin/env python3
import time
import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # ---------------------------------------------------------
    # 1. FORCE SET INITIAL POSE (Fixes the "Waiting..." hang)
    # ---------------------------------------------------------
    # Matches your nav2_params.yaml: x=0.0, y=0.0, yaw=-1.4
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = -0.6442 # Approx for -1.4 rad
    initial_pose.pose.orientation.w = 0.7648
    
    print("Setting initial pose to match config...")
    navigator.setInitialPose(initial_pose)
    
    # Wait strictly for Nav2 to be fully ready
    print("Waiting for Nav2 to activate...")
    navigator.waitUntilNav2Active()

    # ---------------------------------------------------------
    # 2. DEFINE GOALS (From your provided coordinates)
    # ---------------------------------------------------------
    goal_poses = []

    # Waypoint 1
    wp1 = PoseStamped()
    wp1.header.frame_id = 'map'
    wp1.header.stamp = navigator.get_clock().now().to_msg()
    wp1.pose.position.x = 48.36534881591797
    wp1.pose.position.y = -55.31733703613281
    wp1.pose.orientation.z = 0.11203491605641613
    wp1.pose.orientation.w = 0.993704270688333
    goal_poses.append(wp1)

    # Waypoint 2
    wp2 = PoseStamped()
    wp2.header.frame_id = 'map'
    wp2.header.stamp = navigator.get_clock().now().to_msg()
    wp2.pose.position.x = 68.55014038085938
    wp2.pose.position.y = -34.091941833496094
    wp2.pose.orientation.z = 0.7723275999382105
    wp2.pose.orientation.w = 0.6352244314993586
    goal_poses.append(wp2)

    # Waypoint 3
    wp3 = PoseStamped()
    wp3.header.frame_id = 'map'
    wp3.header.stamp = navigator.get_clock().now().to_msg()
    wp3.pose.position.x = 31.628843307495117
    wp3.pose.position.y = -18.040040969848633
    wp3.pose.orientation.z = -0.9954245458906165
    wp3.pose.orientation.w = 0.09555089449324947
    goal_poses.append(wp3)

    # ---------------------------------------------------------
    # 3. EXECUTE PATH
    # ---------------------------------------------------------
    # We use goThroughPoses because your Behavior Tree is designed for it.
    print(f"Sending {len(goal_poses)} waypoints to the planner...")
    navigator.goThroughPoses(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(f'Distance remaining: {feedback.distance_remaining:.2f} meters')
            # Check if navigation is stuck for too long (optional safety)
            # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
            #     navigator.cancelTask()

    # ---------------------------------------------------------
    # 4. RESULT HANDLING
    # ---------------------------------------------------------
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Route complete! All waypoints reached.')
    elif result == TaskResult.CANCELED:
        print('Route was canceled!')
    elif result == TaskResult.FAILED:
        print('Route failed!')

    # Cleanup
    exit(0)

if __name__ == '__main__':
    main()