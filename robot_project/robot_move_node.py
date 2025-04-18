#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.msg import HazardDetectionVector, HazardDetection
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import time
import math
import threading
from nav_msgs.msg import Odometry
from irobot_create_msgs.action import RotateAngle


class Create3Controller(Node):
    def __init__(self):
        super().__init__("create3_controller")

        self.declare_parameter("topicPrefix","")
        self.topic_prefix = self.get_parameter("topicPrefix").value
        
        # Create callback groups
        self.hazard_callback_group = ReentrantCallbackGroup()
        self.action_callback_group = MutuallyExclusiveCallbackGroup()
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist, self.topic_prefix+"/cmd_vel", QoSProfile(depth=10)
        )
        self.undock_client = ActionClient(
            self, 
            Undock, 
            self.topic_prefix+"/undock", 
            callback_group=self.action_callback_group
        )
        self.dock_client = ActionClient(
            self, 
            Dock, 
            self.topic_prefix+"/dock", 
            callback_group=self.action_callback_group
        )
        
        self.rotate_client = ActionClient(
            self,
            RotateAngle,
            self.topic_prefix+"/rotate_angle",
            callback_group=self.action_callback_group
        )

        hazard_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.hazard_subscription = self.create_subscription(
            HazardDetectionVector,
            self.topic_prefix+'/hazard_detection',
            self.hazard_callback,
            hazard_qos,
            callback_group=self.hazard_callback_group
        )
        self.current_yaw = 0.0
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.topic_prefix + '/odom',
            self.odom_callback,
            QoSProfile(depth=10),
            callback_group=self.hazard_callback_group
        )
        self.bump_detected = False
        self.get_logger().info("Node initialized and subscribed to hazard detection")

    def hazard_callback(self, msg):
        self.get_logger().info(f"Received hazard detection with {len(msg.detections)} detections")
        for detection in msg.detections:
            if detection.type == HazardDetection.BUMP:
                self.bump_detected = True
                # Check if the bump is in the front center
                if "front_center" in detection.header.frame_id:
                    self.front_center_bump = True
                    self.get_logger().info(f"Front center bump detected! Frame: {detection.header.frame_id}")
                else:
                    self.get_logger().info(f"Bump detected in: {detection.header.frame_id}")
    
    def undock(self):
        self.get_logger().info("Undocking...")
        if not self.undock_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Undock action server not available!")
            return False  # Return False to indicate failure

        goal_msg = Undock.Goal()
        future = self.undock_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Undock goal rejected!")
            return False

        self.get_logger().info("Undock goal accepted.")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        # Check if undocking was successful
        if result_future.result():
            self.get_logger().info("Successfully undocked")
            # Wait a moment after undocking
            time.sleep(2.0)
            return True
        else:
            self.get_logger().error("Failed to undock")
            return False

    def dock(self):
        self.get_logger().info("Docking...")
        if not self.dock_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Dock action server not available!")
            return

        goal_msg = Dock.Goal()
        future = self.dock_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Dock goal rejected!")
            return

        self.get_logger().info("Dock goal accepted.")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

    def move_forward(self, distance, speed=0.2):
        self.get_logger().info(f"Moving forward {distance} meters...")
        twist = Twist()
        twist.linear.x = speed
        duration = distance / speed
        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        while self.get_clock().now().seconds_nanoseconds()[0] - start_time < duration:
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.1)
        twist.linear.x = 0.0
        self.cmd_vel_publisher.publish(twist)

    def move_until_bump(self, speed=0.2, timeout=30.0):
        self.get_logger().info("Moving forward until any bump detected...")
        self.bump_detected = False

        twist = Twist()
        twist.linear.x = speed

        start = self.get_clock().now().nanoseconds / 1e9
        last_log = start

        while not self.bump_detected:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - start > timeout:
                self.get_logger().info("Timeout reached without bump")
                break

            self.cmd_vel_publisher.publish(twist)
            # let any incoming hazard_callback run
            rclpy.spin_once(self, timeout_sec=0)

            # periodic status
            if now - last_log > 2.0:
                self.get_logger().info(f"Still moving… bump_detected={self.bump_detected}")
                last_log = now

            time.sleep(0.05)

        # stop motion
        twist.linear.x = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Stopped after bump or timeout")

    def odom_callback(self, msg):
        # extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny, cosy)

    def _angle_diff(self, target, current):
        # shortest angular difference
        a = (target - current + math.pi) % (2*math.pi) - math.pi
        return a
    
    def rotate(self, angle, angular_speed=0.5):   
        if not self.rotate_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("RotateAngle action server not available!")
            return

        goal_msg = RotateAngle.Goal()
        goal_msg.angle = angle
        future = self.rotate_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(1.0)

'''
    def rotate(self, angle, angular_speed=0.5):
        #self.get_logger().info(f"Rotating {angle} radians (feedback)...")
        # accumulate actual yaw change
        #prev_yaw = self.current_yaw
        #turned = 0.0
        #twist = Twist()
        #twist.angular.z = angular_speed if angle > 0 else -angular_speed

        #while abs(turned) < abs(angle):
        #    self.cmd_vel_publisher.publish(twist)
        #    rclpy.spin_once(self, timeout_sec=0)
        #    # compute incremental yaw (handles wrapping)
        ###    delta = self._angle_diff(self.current_yaw, prev_yaw)
            turned += delta
            prev_yaw = self.current_yaw
            time.sleep(0.02)

        # stop rotation
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Rotation complete")'''
        

def main(args=None):
    rclpy.init(args=args)
    node = Create3Controller()
    
    # Set up the executor and spin in a separate thread
    executor = MultiThreadedExecutor(num_threads=4)  # Use more threads
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    # Test if bumper callback is working properly
    node.get_logger().info("Waiting to verify hazard detection subscription...")
    time.sleep(2.0)  # Wait for subscriptions to be established

    try:
        node.get_logger().info("Starting robot sequence...")
        
        if node.undock():
            # Test movement
            node.get_logger().info("Testing movement...")
            test_twist = Twist()
            test_twist.linear.x = 0.2
            for i in range(5):
                node.cmd_vel_publisher.publish(test_twist)
                time.sleep(0.1)
            test_twist.linear.x = 0.0
            node.cmd_vel_publisher.publish(test_twist)
            
            # Move until bump with shorter timeout for testing
            node.get_logger().info("Moving until bump...")
            node.move_until_bump(speed=0.2, timeout=15.0)
            
            # Complete the sequence
            node.rotate(math.pi)
            node.move_forward(1.0)
            node.dock()
        else:
            node.get_logger().error("Failed to undock, aborting sequence")
    except Exception as e:
        node.get_logger().error(f"An error occurred: {e}")
        import traceback
        node.get_logger().error(traceback.format_exc())
    finally:
        # Stop the robot
        stop_cmd = Twist()
        node.cmd_vel_publisher.publish(stop_cmd)
        
        time.sleep(1.0)  # Give time for final commands to be processed
        rclpy.shutdown()
        executor_thread.join(timeout=3.0)  # Add timeout to prevent hanging
        
if __name__ == "__main__":
    main()