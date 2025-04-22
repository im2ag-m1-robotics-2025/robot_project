#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.msg import HazardDetectionVector, HazardDetection, DockStatus
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import time
import math
import threading
from nav_msgs.msg import Odometry
from irobot_create_msgs.action import RotateAngle
from group1_interfaces.msg import Interrupt
from rclpy.duration import Duration
import sys

class Create3Controller(Node):
    def __init__(self):
        super().__init__("create3_controller")

        self.declare_parameter("topicPrefix", "")
        self.topic_prefix = self.get_parameter("topicPrefix").value

        self.EXPLORE = 0
        self.AVOID_OBSTACLE = 1
        self.MANUAL_CONTROL = 2
        self.SHOULD_DOCK = 3
        self.DOCKED = 4
        self.RETURN = 5
        self.state = self.DOCKED
        self.prev_state = self.DOCKED
        
        # Create callback groups
        self.hazard_callback_group = ReentrantCallbackGroup()
        self.action_callback_group = MutuallyExclusiveCallbackGroup()

        self.cmd_vel_publisher = self.create_publisher(
            Twist, self.topic_prefix + "/cmd_vel", QoSProfile(depth=10)
        )
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
        self.current_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.last_bump_side = "front"
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.topic_prefix + "/odom",
            self.odom_callback,
            QoSProfile(depth=10),
            callback_group=self.hazard_callback_group
        )

        self.interrupt_subscription = self.create_subscription(
           Interrupt,
            "/interrupt_topic",
            self.interrupt_callback,
            10
        )

        self.dock_status_subscription = self.create_subscription(
            DockStatus,
            self.topic_prefix+'/dock_status',
            self.dock_status_callback,
            10  
        )
        self.timeout = 30
        self.timeout_happened = False
        self.undocked = False
        self.start_time = self.get_clock().now()
        self.get_logger().info("Node initialized and subscribed to hazard detection")

    def dock_status_callback(self,msg: DockStatus):
        if self.state != self.MANUAL_CONTROL and self.state != self.DOCKED and self.state==self.RETURN:
            if msg.dock_visible:
                self.state = self.SHOULD_DOCK

    def hazard_callback(self, msg):
        if self.state == self.MANUAL_CONTROL or self.state == self.DOCKED: return
        for detection in msg.detections:
            if detection.type == HazardDetection.BUMP:
                self.prev_state = self.state
                self.state = self.AVOID_OBSTACLE
                # detect which side bumped
                fid = detection.header.frame_id.lower()
                if "left" in fid:
                    self.last_bump_side = "left"
                elif "right" in fid:
                    self.last_bump_side = "right"
                else:
                    self.last_bump_side = "front"
                self.bump_detected = True
                # Check if the bump is in the front center
                if "front_center" in detection.header.frame_id:
                    self.front_center_bump = True
                    self.get_logger().info(
                        f"Front center bump detected! Frame: {detection.header.frame_id}"
                    )
                else:
                    self.get_logger().info(
                        f"Bump detected in: {detection.header.frame_id}"
                    )

    def interrupt_callback(self, msg):
        self.state = self.MANUAL_CONTROL
        self.cmd_vel_publisher.publish(Twist())
        time.sleep(0.1)

    def undock(self):
        self.get_logger().info("Undocking...")
        if not self.undock_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("undock error")
            return False

        goal_msg = Undock.Goal()
        future = self.undock_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("not undocked")
            return False

        self.get_logger().info("going to undock")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        # Check if undocking was successful
        if result_future.result():
            self.get_logger().info("undocked with success")
            # Wait a moment after undocking
            time.sleep(2.0)
            return True
        else:
            self.get_logger().error("not undocked")
            return False

    def dock(self):
        self.get_logger().info("Docking...")
        if not self.dock_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("dock error")
            return

        goal_msg = Dock.Goal()
        future = self.dock_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("not docked")
            return

        self.get_logger().info("going to dock")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

    def move_forward(self, distance, speed=0.2):
        if self.state == self.MANUAL_CONTROL:
            return
        sign = 1.0 if distance >= 0 else -1.0
        twist = Twist()
        twist.linear.x = sign * abs(speed)
        duration = abs(distance) / abs(speed)
        start = self.get_clock().now().nanoseconds / 1e9
        while self.get_clock().now().nanoseconds / 1e9 - start < duration:
            self.cmd_vel_publisher.publish(twist)
            rclpy.spin_once(self, timeout_sec=0)
            time.sleep(0.1)
        twist.linear.x = 0.0
        self.cmd_vel_publisher.publish(twist)

    def move_until_bump(self, speed=0.2, timeout=30.0,backup_dist = 0.1):
        if self.state == self.MANUAL_CONTROL: return
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

    # rotate to a given angle
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny, cosy)
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    # get the difference between two angles
    def _angle_diff(self, target, current):
        a = (target - current + math.pi) % (2 * math.pi) - math.pi
        return a

    def rotate(self, angle):
        if self.state == self.MANUAL_CONTROL: return
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
            dx = -self.current_x
            dy = -self.current_y
            dist = math.hypot(dx, dy)
            if dist <= 1:
                # face dock directly
                target_yaw = math.atan2(dy, dx)
                self.rotate(self._angle_diff(target_yaw, self.current_yaw))
                # final move to exactly 1 m away
                delta = dist - 1
                if abs(delta) > 0.01:
                    self.move_forward(delta)
                break

            # face dock
            target_yaw = math.atan2(dy, dx)
            self.rotate(self._angle_diff(target_yaw, self.current_yaw))

            # move toward dock while avoiding obstacles
            self.move_until_bump(
                speed=speed, timeout=dist / speed + 1, backup_dist=backup_dist
            )
            if self.bump_detected:
                # pick turn direction
                if self.last_bump_side == "left":
                    turn = -math.pi / 2
                else:
                    turn = math.pi / 2
                self.rotate(turn)

        self.get_logger().info("ready to docking")
        self.dock()

    def discover_room(self, duration, speed=0.2):
        if not self.undock():
            self.get_logger().error("Undock failed")
            return
        self.get_logger().info(f"discovery is {duration}s")
        start_t = self.get_clock().now().nanoseconds / 1e9
        while self.get_clock().now().nanoseconds / 1e9 - start_t < duration:
            self.move_until_bump(speed=speed, timeout=duration, backup_dist=0.1)
            # turn based on last bump side
            if self.last_bump_side == "left":
                self.rotate(-math.pi / 2)
            else:
                # front or right turns left
                self.rotate(math.pi / 2)
        # time up: navigate back to dock
        self.return_to_dock(speed=speed, backup_dist=0.1)
        self.get_logger().info("Discovery complete")

    def compute_input(self,input):
        message = Twist()
        if input == 'a':
            message.angular.z = 1.0
        elif input == 'd':
            message.angular.z = -1.0
        elif input == 'w':
            message.linear.x = 3.0
        elif input == 's':
            message.linear.x = -1.0
        else : return
        self.cmd_vel_publisher.publish(message)


    def control_loop(self):
        print(self.state)
        if self.state == self.MANUAL_CONTROL: 
            input = sys.stdin.read(1)
            self.compute_input(input)
            return
        if self.get_clock().now()-self.start_time >= Duration(seconds=self.timeout) and not self.timeout_happened:
            self.state = self.RETURN
            self.timeout_happened = True
            return
        if self.state == self.RETURN:
            dx = -self.current_x
            dy = -self.current_y
            dist = math.hypot(dx, dy)
            # face dock
            target_yaw = math.atan2(dy, dx)
            if self._angle_diff(target_yaw, self.current_yaw) > 0.01:
                self.rotate(self._angle_diff(target_yaw, self.current_yaw))
            #if dist <= 1:
                # final move to exactly 1 m away
            #    delta = dist - 1
            #    if abs(delta) > 0.01:
            #        self.move_forward(delta)
            #    return

            # move toward dock while avoiding obstacles
            twist = Twist()
            twist.linear.x = 1.0
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.1)
            return
        if self.state == self.AVOID_OBSTACLE:
            twist = Twist()
            twist.linear.x = 0.0
            self.cmd_vel_publisher.publish(twist)
            twist.linear.x = -1.0
            dur = 0.8
            t0 = self.get_clock().now().nanoseconds / 1e9
            while self.get_clock().now().nanoseconds / 1e9 - t0 < dur:
                self.cmd_vel_publisher.publish(twist)
                time.sleep(0.1)
            #twist.linear.x = 0.0
            #self.cmd_vel_publisher.publish(twist)
            time.sleep(0.4)
            if self.last_bump_side == "left":
                self.rotate(-math.pi / 2)
            else:
                # front or right turns left
                self.rotate(math.pi / 2)
            self.state = self.prev_state
            return
        if self.state == self.DOCKED and not self.undocked:
            self.undock()
            self.state = self.EXPLORE
            self.undocked = True
            return
        if self.state == self.EXPLORE:
            message = Twist()
            message.linear.x = 1.0
            self.cmd_vel_publisher.publish(message)
            time.sleep(0.5)
            return
        if self.state == self.SHOULD_DOCK:
            self.dock()
            self.state = self.DOCKED

def main(args=None):
    rclpy.init(args=args)
    node = Create3Controller()

    # we had to use MultiThreading, the project was very choppy and laggy without the threads
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    time.sleep(1.0)  # wait for subscriptions to be established

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

        time.sleep(1.0)  # wait time for final commands to be processed
        rclpy.shutdown()
        executor_thread.join(timeout=3.0)  # Add timeout to prevent hanging
    
def main(args=None):
    rclpy.init(args=args)
    node = Create3Controller()
    node.create_timer(0.05,node.control_loop)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
        
if __name__ == "__main__":
    main()
