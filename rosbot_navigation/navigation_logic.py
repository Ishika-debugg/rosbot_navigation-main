
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion, Twist
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import NavigateToPose, Spin
from std_msgs.msg import String, Empty
from tf2_ros import TransformListener, Buffer
from visualization_msgs.msg import Marker
import numpy as np
import time
import math
import random
from enum import Enum
from threading import Lock

from rosbot_navigation.utils import (
    euclidean_distance, 
    is_close_to_point, 
    find_frontiers,
    cluster_frontiers
)

class RobotState(Enum):
    IDLE = 0
    WAITING_FOR_START = 1
    EXPLORING = 2
    NAVIGATING_TO_FRONTIER = 3
    SPINNING = 4
    RETURNING_HOME = 5
    FINISHED = 6

class NavigationLogicNode(Node):
    def __init__(self):
        super().__init__('navigation_logic_node')
        
        # Initialize parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('exploration_timeout', 240.0),  # 4 minutes
                ('total_timeout', 420.0),        # 7 minutes
                ('spin_time', 5.0),              # Spin duration in seconds
                ('min_frontier_size', 5),        # Minimum size of a valid frontier
                ('frontiers_sample_size', 10),   # Number of frontier points to sample
                ('start_position_tolerance', 0.3), # Distance to consider as "at start position"
                ('max_linear_velocity', 0.3),    # Maximum linear velocity
                ('max_angular_velocity', 0.5),   # Maximum angular velocity
            ]
        )
        
        # Get parameters
        self.exploration_timeout = self.get_parameter('exploration_timeout').value
        self.total_timeout = self.get_parameter('total_timeout').value
        self.spin_time = self.get_parameter('spin_time').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.frontiers_sample_size = self.get_parameter('frontiers_sample_size').value
        self.start_position_tolerance = self.get_parameter('start_position_tolerance').value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        
        # Initialize variables
        self.state = RobotState.WAITING_FOR_START
        self.start_time = None
        self.start_position = None
        self.current_position = None
        self.current_orientation = None
        self.map_data = None
        self.map_info = None
        self.path_points = []
        self.start_marker_detected = False
        self.hazards_detected = set()
        self.exploration_target = None
        self.state_lock = Lock()
        
        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create publishers
        self.status_pub = self.create_publisher(String, '/snc_status', 10)
        self.path_explore_pub = self.create_publisher(Path, '/path_explore', 10)
        self.path_return_pub = self.create_publisher(Path, '/path_return', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create subscribers
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, 
            '/map', 
            self.map_callback, 
            qos_profile=map_qos
        )
        
        self.hazards_sub = self.create_subscription(
            Marker,
            '/hazards',
            self.hazards_callback,
            10
        )
        
        # Create trigger subscribers
        self.start_trigger_sub = self.create_subscription(
            Empty,
            '/trigger_start',
            self.trigger_start_callback,
            10
        )
        
        self.return_home_trigger_sub = self.create_subscription(
            Empty,
            '/trigger_return_home',
            self.trigger_return_home_callback,
            10
        )
        
        # Create action clients
        self.navigate_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, 'spin')
        
        # Wait for action servers
        self.get_logger().info('Waiting for navigate_to_pose action server...')
        self.navigate_to_pose_client.wait_for_server()
        self.get_logger().info('Navigate to pose server is available')
        
        self.get_logger().info('Waiting for spin action server...')
        self.spin_client.wait_for_server()
        self.get_logger().info('Spin server is available')
        
        # Create timers
        self.state_machine_timer = self.create_timer(0.5, self.state_machine_callback)
        self.position_update_timer = self.create_timer(0.1, self.update_position)
        self.path_publisher_timer = self.create_timer(1.0, self.publish_path)
        
        self.publish_status("Waiting for start signal")
        self.get_logger().info('Navigation logic node initialized. Waiting for start signal.')
        
    def map_callback(self, msg):
        """Callback for the map subscription."""
        self.map_data = msg.data
        self.map_info = msg.info
        
    def hazards_callback(self, msg):
        """Callback for the hazards subscription."""
        # Keep track of detected hazards by ID
        self.hazards_detected.add(msg.id)
        self.get_logger().info(f'Hazard detected: ID {msg.id}. Total hazards: {len(self.hazards_detected)}')
        
    def trigger_start_callback(self, msg):
        """Callback for the start trigger."""
        if self.state == RobotState.WAITING_FOR_START:
            self.get_logger().info('Start trigger received!')
            self.start_exploration()
        
    def trigger_return_home_callback(self, msg):
        """Callback for the return home trigger."""
        with self.state_lock:
            if self.state in [RobotState.EXPLORING, RobotState.NAVIGATING_TO_FRONTIER, RobotState.SPINNING]:
                self.get_logger().info('Return home trigger received!')
                self.start_returning_home()
    
    def update_position(self):
        """Update the current position of the robot from TF."""
        try:
            # Get transform from base_link to map
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            
            # Store current position
            position = transform.transform.translation
            orientation = transform.transform.rotation
            self.current_position = (position.x, position.y)
            self.current_orientation = (orientation.x, orientation.y, orientation.z, orientation.w)
            
            # Set start position if not set yet
            if self.start_position is None:
                self.start_position = self.current_position
                self.get_logger().info(f'Start position set: {self.start_position}')
            
            # Add current position to path points if exploring
            if self.state in [RobotState.EXPLORING, RobotState.NAVIGATING_TO_FRONTIER, RobotState.SPINNING]:
                # Add position to path points list (don't add duplicates)
                if not self.path_points or euclidean_distance(
                    self.path_points[-1][0], self.path_points[-1][1],
                    self.current_position[0], self.current_position[1]
                ) > 0.05:  # Only add if moved at least 5 cm
                    self.path_points.append(self.current_position)
            
        except Exception as e:
            self.get_logger().warning(f'Could not get robot position: {e}')
    
    def publish_path(self):
        """Publish the robot's path during exploration."""
        if not self.path_points:
            return
            
        # Create path message for exploration path
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for point in self.path_points:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            
            # Default orientation (not important for visualization)
            pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)
        
        # Publish exploration path
        self.path_explore_pub.publish(path_msg)
        
        # If returning home, publish return path too
        if self.state == RobotState.RETURNING_HOME:
            # For return path, we use the reversed path points
            return_path_msg = Path()
            return_path_msg.header.stamp = self.get_clock().now().to_msg()
            return_path_msg.header.frame_id = 'map'
            
            # Use reversed path points
            reversed_path = list(reversed(self.path_points))
            
            for point in reversed_path:
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = 'map'
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                pose.pose.position.z = 0.0
                
                # Default orientation
                pose.pose.orientation.w = 1.0
                
                return_path_msg.poses.append(pose)
            
            self.path_return_pub.publish(return_path_msg)
    
    def publish_status(self, status_text):
        """Publish the current status of the robot."""
        msg = String()
        msg.data = status_text
        self.status_pub.publish(msg)
    
    def start_exploration(self):
        """Start the exploration phase."""
        with self.state_lock:
            self.state = RobotState.EXPLORING
            self.start_time = time.time()
            self.publish_status("Starting exploration")
            self.get_logger().info('Starting exploration')
    
    def start_returning_home(self):
        """Start the return-to-home phase."""
        with self.state_lock:
            # Cancel any active goals
            if hasattr(self, 'current_nav_goal'):
                self.current_nav_goal.cancel_goal_async()
            
            self.state = RobotState.RETURNING_HOME
            self.publish_status("Returning to home position")
            self.get_logger().info('Returning to home position')
            
            # Start navigating through the path points in reverse order
            self.return_path_index = len(self.path_points) - 1
            
            # Check if we're already at home position
            if is_close_to_point(
                self.current_position[0], self.current_position[1],
                self.start_position[0], self.start_position[1],
                self.start_position_tolerance
            ):
                self.state = RobotState.FINISHED
                self.publish_status("Finished: Already at home position")
                self.get_logger().info('Already at home position. Mission completed!')
                return
            
            # Navigate to the next waypoint
            self.navigate_to_next_return_waypoint()
    
    def navigate_to_next_return_waypoint(self):
        """Navigate to the next waypoint when returning home."""
        if self.return_path_index < 0:
            # We've gone through all waypoints
            self.state = RobotState.FINISHED
            self.publish_status("Finished: Reached home position")
            self.get_logger().info('Reached home position. Mission completed!')
            return
        
        # Get next waypoint
        waypoint = self.path_points[self.return_path_index]
        
        # Skip waypoints that are close to each other (reduce the number of goals)
        while self.return_path_index > 0 and euclidean_distance(
            waypoint[0], waypoint[1],
            self.path_points[self.return_path_index - 1][0], self.path_points[self.return_path_index - 1][1]
        ) < 0.5:
            self.return_path_index -= 1
            waypoint = self.path_points[self.return_path_index]
        
        # Check if we're already at this waypoint
        if is_close_to_point(
            self.current_position[0], self.current_position[1],
            waypoint[0], waypoint[1],
            0.3
        ):
            # Skip to next waypoint
            self.return_path_index -= 1
            self.navigate_to_next_return_waypoint()
            return
        
        # Navigate to waypoint
        self.publish_status(f"Returning home: Navigating to waypoint {self.return_path_index}")
        self.send_navigation_goal(waypoint[0], waypoint[1])
    
    def find_next_exploration_target(self):
        """Find the next frontier to explore."""
        if self.map_data is None or self.map_info is None:
            self.get_logger().warning('Cannot find exploration target: No map data available')
            return None
        
        # Find frontiers in the map
        frontiers = find_frontiers(
            self.map_data, 
            self.map_info.width, 
            self.map_info.height, 
            self.map_info.resolution, 
            self.map_info.origin
        )
        
        if not frontiers:
            self.get_logger().info('No frontiers found')
            return None
            
        # Cluster frontiers to get better exploration targets
        clustered_frontiers = cluster_frontiers(frontiers)
        
        if not clustered_frontiers:
            self.get_logger().info('No clustered frontiers found')
            return None
        
        # Find the closest frontier to the current position
        closest_frontier = None
        min_distance = float('inf')
        
        for frontier in clustered_frontiers:
            dist = euclidean_distance(
                self.current_position[0], self.current_position[1],
                frontier[0], frontier[1]
            )
            
            if dist < min_distance:
                min_distance = dist
                closest_frontier = frontier
        
        if closest_frontier:
            self.get_logger().info(f'Found exploration target at {closest_frontier}')
            return closest_frontier
        
        return None
    
    def send_navigation_goal(self, x, y):
        """Send a navigation goal to the action server."""
        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0  # Default orientation
        
        # Send goal
        self.current_nav_goal = self.navigate_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        )
        
        # Set result callback
        self.current_nav_goal.add_done_callback(self.navigation_response_callback)
    
    def send_spin_goal(self):
        """Send a spin goal to the action server."""
        # Create goal
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = math.pi * 2  # Spin 360 degrees
        
        # Send goal
        self.current_spin_goal = self.spin_client.send_goal_async(
            goal_msg,
            feedback_callback=self.spin_feedback_callback
        )
        
        # Set result callback
        self.current_spin_goal.add_done_callback(self.spin_response_callback)
    
    def navigation_feedback_callback(self, feedback):
        """Handle feedback from the navigation action server."""
        pass  # Not needed for this implementation
    
    def navigation_response_callback(self, future):
        """Handle response from the navigation action server."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal was rejected')
            return
        
        # Get the result
        self.nav_result_future = goal_handle.get_result_async()
        self.nav_result_future.add_done_callback(self.navigation_result_callback)
    
    def navigation_result_callback(self, future):
        """Handle result from the navigation action server."""
        with self.state_lock:
            # Check if the state has changed while we were navigating
            if self.state not in [RobotState.NAVIGATING_TO_FRONTIER, RobotState.RETURNING_HOME]:
                return
                
            result = future.result().result
            
            if self.state == RobotState.NAVIGATING_TO_FRONTIER:
                # After reaching a frontier, do a 360-degree spin to look for hazards
                self.state = RobotState.SPINNING
                self.publish_status("Spinning to look for hazards")
                self.send_spin_goal()
            
            elif self.state == RobotState.RETURNING_HOME:
                # Move to the next waypoint
                self.return_path_index -= 1
                
                # Check if we're close to home
                if self.return_path_index <= 0 or is_close_to_point(
                    self.current_position[0], self.current_position[1],
                    self.start_position[0], self.start_position[1],
                    self.start_position_tolerance
                ):
                    # We're home or close enough to home
                    self.state = RobotState.FINISHED
                    self.publish_status("Finished: Reached home position")
                    self.get_logger().info('Reached home position. Mission completed!')
                else:
                    # Navigate to the next waypoint
                    self.navigate_to_next_return_waypoint()
    
    def spin_feedback_callback(self, feedback):
        """Handle feedback from the spin action server."""
        pass  # Not needed for this implementation
    
    def spin_response_callback(self, future):
        """Handle response from the spin action server."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Spin goal was rejected')
            return
        
        # Get the result
        self.spin_result_future = goal_handle.get_result_async()
        self.spin_result_future.add_done_callback(self.spin_result_callback)
    
    def spin_result_callback(self, future):
        """Handle result from the spin action server."""
        with self.state_lock:
            if self.state != RobotState.SPINNING:
                return
                
            # After spinning, continue exploration
            self.state = RobotState.EXPLORING
            self.publish_status("Continuing exploration")
    
    def stop_robot(self):
        """Send zero velocity command to stop the robot."""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
    
    def state_machine_callback(self):
        """Main state machine for the robot behavior."""
        with self.state_lock:
            # Check timeouts
            current_time = time.time()
            
            if self.start_time is not None:
                elapsed_time = current_time - self.start_time
                
                # Check if we need to return home after exploration timeout
                if (self.state in [RobotState.EXPLORING, RobotState.NAVIGATING_TO_FRONTIER, RobotState.SPINNING] and 
                    elapsed_time > self.exploration_timeout):
                    self.get_logger().info('Exploration timeout reached. Returning home.')
                    self.start_returning_home()
                    return
                
                # Check if total timeout has been reached
                if elapsed_time > self.total_timeout:
                    self.get_logger().info('Total timeout reached. Stopping robot.')
                    self.stop_robot()
                    self.state = RobotState.FINISHED
                    self.publish_status("Finished: Total timeout reached")
                    return
            
            # Handle different states
            if self.state == RobotState.WAITING_FOR_START:
                # Waiting for start marker
                # Logic is handled in trigger_start_callback
                pass
                
            elif self.state == RobotState.EXPLORING:
                # Find the next frontier to explore
                target = self.find_next_exploration_target()
                
                if target is None:
                    self.get_logger().info('No more frontiers to explore. All area explored!')
                    # If all hazards have been found, return home
                    if len(self.hazards_detected) >= 5:
                        self.get_logger().info('All hazards found. Returning home.')
                        self.start_returning_home()
                    else:
                        # Keep exploring by doing a spin
                        self.state = RobotState.SPINNING
                        self.publish_status("Spinning to look for more hazards")
                        self.send_spin_goal()
                else:
                    # Navigate to the found frontier
                    self.exploration_target = target
                    self.state = RobotState.NAVIGATING_TO_FRONTIER
                    self.publish_status(f"Navigating to frontier at ({target[0]:.2f}, {target[1]:.2f})")
                    self.send_navigation_goal(target[0], target[1])
            
            elif self.state == RobotState.NAVIGATING_TO_FRONTIER:
                # Logic is handled in navigation_result_callback
                pass
                
            elif self.state == RobotState.SPINNING:
                # Logic is handled in spin_result_callback
                pass
                
            elif self.state == RobotState.RETURNING_HOME:
                # Check if we've reached home
                if self.current_position and self.start_position:
                    if is_close_to_point(
                        self.current_position[0], self.current_position[1],
                        self.start_position[0], self.start_position[1],
                        self.start_position_tolerance
                    ):
                        self.state = RobotState.FINISHED
                        self.stop_robot()
                        self.publish_status("Finished: Reached home position")
                        self.get_logger().info('Reached home position. Mission completed!')
                
                # Rest of the logic is handled in navigation_result_callback
                
            elif self.state == RobotState.FINISHED:
                # Make sure robot is stopped
                self.stop_robot()

def main(args=None):
    rclpy.init(args=args)
    node = NavigationLogicNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()