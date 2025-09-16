#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
import time

class Nav2GoalNode(Node):
    def __init__(self):
        super().__init__('nav2_goal_node')
        self.declare_parameter('set_initial_pose', True)
        self.declare_parameter('follow_waypoints', False)
        self.declare_parameter('set_custom_goal', False)
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('w', 0.0)

        self.x = self.get_parameter('x').get_parameter_value().double_value
        self.y = self.get_parameter('y').get_parameter_value().double_value
        self.w = self.get_parameter('w').get_parameter_value().double_value
        self.set_initial_pose = self.get_parameter('set_initial_pose').get_parameter_value().bool_value
        self.follow_waypoints = self.get_parameter('follow_waypoints').get_parameter_value().bool_value
        self.set_custom_goal  = self.get_parameter('set_custom_goal').get_parameter_value().bool_value

        self.get_logger().info(f"set_initial_pose={self.set_initial_pose}, "
                               f"follow_waypoints={self.follow_waypoints}, "
                               f"set_custom_goal={self.set_custom_goal}, "
                               f"x={self.x}, y={self.y}, w={self.w}")
    
    def create_pose_stamped(self, navigator: BasicNavigator, position_x, position_y, orientation_z):
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0, 0, orientation_z)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = position_x
        pose.pose.position.y = position_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = Nav2GoalNode()
    
    navigator = BasicNavigator()

    if node.set_initial_pose:
        initial_pose = node.create_pose_stamped(navigator, 0.0, 0.0, 0.0)
        navigator.setInitialPose(initial_pose)
        navigator.waitUntilNav2Active()

    if node.set_custom_goal:
        goal_pose = node.create_pose_stamped(navigator, node.x, node.y, node.w)
        navigator.goToPose(goal_pose)
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            time.sleep(0.5)

        print(navigator.getResult())
    
    if node.follow_waypoints:
        way_points = [
            node.create_pose_stamped(navigator, 2.0, -1.5, 0.0),
            node.create_pose_stamped(navigator, 2.0, 2.5, 1.57),
            node.create_pose_stamped(navigator, 0.0, 0.0, 0.0)
        ]
        navigator.followWaypoints(way_points)
        
        while not navigator.isTaskComplete():
            time.sleep(0.5)
            
        print(navigator.getResult())
    
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
