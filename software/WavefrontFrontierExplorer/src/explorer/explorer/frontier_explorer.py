#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import qos_profile_sensor_data

import numpy as np
from scipy.ndimage import label

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # Map from SLAM
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_profile_sensor_data
        )

        # Publish goals for navigation
        self.goal_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)

        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.05  # default, will update from message

        self.timer = self.create_timer(1.0, self.exploration_loop)
        self.current_goal = None

    def map_callback(self, msg: OccupancyGrid):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)

    def find_frontiers(self):
        """
        Detect frontier cells: free cells (0) with unknown neighbors (-1)
        """
        if self.map_data is None:
            return []

        frontiers = []
        free_cells = np.where(self.map_data == 0)
        for y, x in zip(*free_cells):
            neighbors = self.map_data[max(y-1,0):y+2, max(x-1,0):x+2]
            if -1 in neighbors:
                frontiers.append((y, x))
        return frontiers

    def cluster_frontiers(self, frontiers):
        """
        Cluster frontier cells into regions
        """
        frontier_map = np.zeros_like(self.map_data, dtype=int)
        for y, x in frontiers:
            frontier_map[y, x] = 1
        labeled, n = label(frontier_map)
        clusters = []
        for i in range(1, n+1):
            cells = np.argwhere(labeled == i)
            clusters.append(cells)
        return clusters

    def select_frontier(self, clusters):
        """
        Select the largest cluster as target
        """
        if not clusters:
            return None
        largest = max(clusters, key=lambda c: len(c))
        # Compute centroid
        y_mean, x_mean = np.mean(largest, axis=0)
        # Convert to world coordinates
        x_world = self.map_origin[0] + x_mean * self.map_resolution
        y_world = self.map_origin[1] + y_mean * self.map_resolution
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x_world
        goal.pose.position.y = y_world
        goal.pose.orientation.w = 1.0
        return goal

    def exploration_loop(self):
        if self.current_goal is not None:
            return  # already have a goal

        frontiers = self.find_frontiers()
        clusters = self.cluster_frontiers(frontiers)
        goal = self.select_frontier(clusters)
        if goal:
            self.goal_pub.publish(goal)
            self.current_goal = goal
            self.get_logger().info(f"New frontier goal published at x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()