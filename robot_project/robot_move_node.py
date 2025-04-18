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
        self.current_x = 0.0
        self.current_y = 0.0
        self.last_bump_side = 'front'
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
        for detection in msg.detections:
            if detection.type == HazardDetection.BUMP:
                # detect which side bumped
                fid = detection.header.frame_id.lower()
                if 'left' in fid: self.last_bump_side = 'left'
                elif 'right' in fid: self.last_bump_side = 'right'
                else: self.last_bump_side = 'front'
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
        """Move forward (or backward if distance<0) by distance (meters)."""
        sign = 1.0 if distance >= 0 else -1.0
        twist = Twist()
        twist.linear.x = sign * abs(speed)
        duration = abs(distance) / abs(speed)
        start = self.get_clock().now().nanoseconds / 1e9
        while self.get_clock().now().nanoseconds / 1e9 - start < duration:
            self.cmd_vel_publisher.publish(twist)
            rclpy.spin_once(self, timeout_sec=0)   # allow callbacks
            time.sleep(0.1)
        twist.linear.x = 0.0
        self.cmd_vel_publisher.publish(twist)

    def move_until_bump(self, speed=0.2, timeout=30.0, backup_dist=0.1):
        # record start pose
        x_start, y_start = self.current_x, self.current_y
        self.bump_detected = False

        twist = Twist()
        twist.linear.x = speed
        start = self.get_clock().now().nanoseconds / 1e9

        # move forward until bump or timeout
        while not self.bump_detected:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - start > timeout:
                break
            self.cmd_vel_publisher.publish(twist)
            rclpy.spin_once(self, timeout_sec=0)
            time.sleep(0.05)

        # stop forward motion
        twist.linear.x = 0.0
        self.cmd_vel_publisher.publish(twist)

        # compute forward distance travelled
        fwd_dist = math.hypot(self.current_x - x_start, self.current_y - y_start)

        # back up a bit to clear obstacle
        if self.bump_detected:
            twist.linear.x = -speed
            dur = backup_dist / speed
            t0 = self.get_clock().now().nanoseconds / 1e9
            while self.get_clock().now().nanoseconds / 1e9 - t0 < dur:
                self.cmd_vel_publisher.publish(twist)
                rclpy.spin_once(self, timeout_sec=0)
                time.sleep(0.05)
            twist.linear.x = 0.0
            self.cmd_vel_publisher.publish(twist)

        return fwd_dist

    def odom_callback(self, msg):
        # extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny, cosy)
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

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

    def return_to_dock(self, speed=0.2, backup_dist=0.1):
        self.get_logger().info("Returning to dock")
        while True:
            dx = -self.current_x; dy = -self.current_y
            dist = math.hypot(dx, dy)
            if dist <= 0.5:
                # face dock before final adjustment
                target_yaw = math.atan2(dy, dx)
                self.rotate(self._angle_diff(target_yaw, self.current_yaw))
                # final move to exactly 0.5 m away
                delta = dist - 0.5
                if abs(delta) > 0.01:
                    self.move_forward(delta)
                break

            # face dock
            target_yaw = math.atan2(dy, dx)
            self.rotate(self._angle_diff(target_yaw, self.current_yaw))

            # move toward dock, avoid obstacles
            self.move_until_bump(speed=speed, timeout=dist/speed + 1, backup_dist=backup_dist)
            if self.bump_detected:
                # pick turn direction
                if self.last_bump_side == 'left':
                    turn = -math.pi/2
                else:
                    turn = math.pi/2
                self.rotate(turn)

        self.get_logger().info("Positioned for docking")
        self.dock()

    def discover_room(self, duration, speed=0.2):
        if not self.undock():
            self.get_logger().error("Undock failed, aborting discovery")
            return
        self.get_logger().info(f"Starting discovery for {duration}s")
        start_t = self.get_clock().now().nanoseconds / 1e9
        while self.get_clock().now().nanoseconds / 1e9 - start_t < duration:
            self.move_until_bump(speed=speed, timeout=duration, backup_dist=0.1)
            # turn based on last bump side
            if self.last_bump_side == 'left':
                self.rotate(-math.pi/2)
            else:
                # front or right => left
                self.rotate(math.pi/2)
        # time up: navigate back to dock
        self.return_to_dock(speed=speed, backup_dist=0.1)
        self.get_logger().info("Discovery complete")

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
        node.discover_room(duration=10.0)
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