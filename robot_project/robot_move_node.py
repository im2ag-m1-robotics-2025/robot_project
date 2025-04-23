#!/usr/bin/env python3
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from irobot_create_msgs.action import Dock, Undock, RotateAngle
from irobot_create_msgs.msg import HazardDetectionVector, HazardDetection, DockStatus
from nav_msgs.msg import Odometry
from group1_interfaces.msg import Interrupt


class RobotState:
    """Enum class for robot states"""
    EXPLORE = 0
    AVOID_OBSTACLE = 1
    MANUAL_CONTROL = 2
    SHOULD_DOCK = 3
    DOCKED = 4
    RETURN = 5


class Create3Controller(Node):
    def __init__(self):
        super().__init__("create3_controller")
        
        # Initialize parameters
        self.declare_parameter("topicPrefix", "")
        self.topic_prefix = self.get_parameter("topicPrefix").value
        
        # Initialize robot state
        self.state = RobotState.DOCKED
        self.prev_state = RobotState.DOCKED
        
        # Initialize robot position
        self.current_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        
        # Initialize collision detection
        self.last_bump_side = "front"
        self.bump_detected = False
        
        # Setup timers and flags
        self.timeout = 30
        self.timeout_happened = False
        self.undocked = False
        self.start_time = self.get_clock().now()
        
        # Setup callback groups
        self.hazard_callback_group = ReentrantCallbackGroup()
        self.action_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Initialize publishers, subscribers, and clients
        self._setup_publishers()
        self._setup_action_clients()
        self._setup_subscribers()
        
        self.get_logger().info("Node initialized and subscribed to hazard detection")

    def _setup_publishers(self):
        """Initialize all publishers"""
        self.cmd_vel_publisher = self.create_publisher(
            Twist, self.topic_prefix + "/cmd_vel", QoSProfile(depth=10)
        )

    def _setup_action_clients(self):
        """Initialize all action clients"""
        self.undock_client = ActionClient(
            self,
            Undock,
            self.topic_prefix + "/undock",
            callback_group=self.action_callback_group,
        )
        
        self.dock_client = ActionClient(
            self,
            Dock,
            self.topic_prefix + "/dock",
            callback_group=self.action_callback_group,
        )
        
        self.rotate_client = ActionClient(
            self,
            RotateAngle,
            self.topic_prefix + "/rotate_angle",
            callback_group=self.action_callback_group,
        )

    def _setup_subscribers(self):
        """Initialize all subscribers with appropriate QoS"""
        # Hazard detection subscription
        hazard_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        
        self.hazard_subscription = self.create_subscription(
            HazardDetectionVector,
            self.topic_prefix + "/hazard_detection",
            self.hazard_callback,
            hazard_qos,
            callback_group=self.hazard_callback_group,
        )

        
        # Odometry subscription
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.topic_prefix + "/odom",
            self.odom_callback,
            QoSProfile(depth=10),
            callback_group=self.hazard_callback_group
        )
        
        # Interrupt subscription
        self.interrupt_subscription = self.create_subscription(
            Interrupt,
            "/interrupt_topic",
            self.interrupt_callback,
            10
        )
        
        # Dock status subscription
        self.dock_status_subscription = self.create_subscription(
            DockStatus,
            self.topic_prefix + '/dock_status',
            self.dock_status_callback,
            10  
        )

    # Callback functions
    def dock_status_callback(self, msg: DockStatus):
        """Handle dock status updates"""
        if (self.state != RobotState.MANUAL_CONTROL and 
            self.state != RobotState.DOCKED and 
            self.state == RobotState.RETURN and 
            msg.dock_visible):
            self.state = RobotState.SHOULD_DOCK

    def hazard_callback(self, msg):
        """Handle hazard detection events"""
        if self.state == RobotState.MANUAL_CONTROL or self.state == RobotState.DOCKED:
            return
            
        for detection in msg.detections:
            if detection.type == HazardDetection.BUMP:
                self.prev_state = self.state
                self.state = RobotState.AVOID_OBSTACLE
                
                # Identify which side bumped
                fid = detection.header.frame_id.lower()
                if "left" in fid:
                    self.last_bump_side = "left"
                elif "right" in fid:
                    self.last_bump_side = "right"
                else:
                    self.last_bump_side = "front"
                    
                self.bump_detected = True
                self.get_logger().info(f"Bump detected in: {detection.header.frame_id}")

    def interrupt_callback(self, msg):
        """Handle interrupt messages for manual control"""
        self.state = RobotState.MANUAL_CONTROL
        self.cmd_vel_publisher.publish(Twist())
        time.sleep(0.1)

    def odom_callback(self, msg):
        """Update current position based on odometry data"""
        # Extract position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Calculate yaw from quaternion
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny, cosy)

    # Action functions
    def undock(self):
        """Execute undocking procedure"""
        self.get_logger().info("Undocking...")
        
        if not self.undock_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Undock server not available")
            return False

        # Send undock goal
        goal_msg = Undock.Goal()
        future = self.undock_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Undock goal not accepted")
            return False

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        # Check result
        if result_future.result():
            self.get_logger().info("Undocked successfully")
            time.sleep(2.0)  # Give time to clear dock
            return True
        else:
            self.get_logger().error("Undocking failed")
            return False

    def dock(self):
        """Execute docking procedure"""
        self.get_logger().info("Docking...")
        
        if not self.dock_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Dock server not available")
            return False

        # Send dock goal
        goal_msg = Dock.Goal()
        future = self.dock_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Dock goal not accepted")
            return False

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        return True

    def rotate(self, angle):
        """Rotate by the specified angle"""
        if self.state == RobotState.MANUAL_CONTROL:
            return
            
        if not self.rotate_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("RotateAngle action server not available!")
            return

        goal_msg = RotateAngle.Goal()
        goal_msg.angle = angle
        future = self.rotate_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(1.0)  # Allow time to complete rotation

    # Utility functions
    def _angle_diff(self, target, current):
        """Calculate the shortest angular distance between two angles"""
        return ((target - current + math.pi) % (2 * math.pi)) - math.pi

    def compute_input(self, input):
        """Process manual control input"""
        message = Twist()
        if input == 'a':
            message.angular.z = 1.0
        elif input == 'd':
            message.angular.z = -1.0
        elif input == 'w':
            message.linear.x = 3.0
        elif input == 's':
            message.linear.x = -1.0
        else:
            return
        self.cmd_vel_publisher.publish(message)

    def control_loop(self):
        """Main control loop that handles robot state transitions"""
        print(self.state)
        
        # Handle manual control mode
        if self.state == RobotState.MANUAL_CONTROL:
            input = sys.stdin.read(1)
            self.compute_input(input)
            return
            
        # Check for timeout
        if (self.get_clock().now() - self.start_time >= Duration(seconds=self.timeout) and 
            not self.timeout_happened):
            self.state = RobotState.RETURN
            self.timeout_happened = True
            return
            
        # Handle different states
        if self.state == RobotState.RETURN:
            self._handle_return_state()
        elif self.state == RobotState.AVOID_OBSTACLE:
            self._handle_avoid_obstacle_state()
        elif self.state == RobotState.DOCKED and not self.undocked:
            self._handle_docked_state()
        elif self.state == RobotState.EXPLORE:
            self._handle_explore_state()
        elif self.state == RobotState.SHOULD_DOCK:
            self._handle_should_dock_state()

    def _handle_return_state(self):
        """Handle the RETURN state logic"""
        # Calculate vector to origin (dock)
        dx = -self.current_x
        dy = -self.current_y
        
        # Face dock
        target_yaw = math.atan2(dy, dx)
        if self._angle_diff(target_yaw, self.current_yaw) > 0.01:
            self.rotate(self._angle_diff(target_yaw, self.current_yaw))
            
        # Move toward dock
        twist = Twist()
        twist.linear.x = 1.0
        self.cmd_vel_publisher.publish(twist)
        time.sleep(0.1)

    def _handle_avoid_obstacle_state(self):
        """Handle the AVOID_OBSTACLE state logic"""
        # Stop and back up
        twist = Twist()
        twist.linear.x = 0.0
        self.cmd_vel_publisher.publish(twist)
        
        twist.linear.x = -1.0
        dur = 0.8
        t0 = self.get_clock().now().nanoseconds / 1e9
        while self.get_clock().now().nanoseconds / 1e9 - t0 < dur:
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.1)
            
        time.sleep(0.4)
        
        # Turn based on bump side
        if self.last_bump_side == "left":
            self.rotate(-math.pi / 2)
        else:
            # Front or right turns left
            self.rotate(math.pi / 2)
            
        # Return to previous state
        self.state = self.prev_state

    def _handle_docked_state(self):
        """Handle the DOCKED state logic"""
        self.undock()
        self.state = RobotState.EXPLORE
        self.undocked = True

    def _handle_explore_state(self):
        """Handle the EXPLORE state logic"""
        message = Twist()
        message.linear.x = 1.0
        self.cmd_vel_publisher.publish(message)
        time.sleep(0.5)

    def _handle_should_dock_state(self):
        """Handle the SHOULD_DOCK state logic"""
        self.dock()
        self.state = RobotState.DOCKED


def main(args=None):
    rclpy.init(args=args)
    node = Create3Controller()
    node.create_timer(0.05, node.control_loop)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()