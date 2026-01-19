#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class MultiFloorManager(Node):
    def __init__(self):
        super().__init__("multi_floor_manager")

        self.map_frame = 'map'
        self.goal_topic = '/goal_pose'
        self.amcl_topic = '/amcl_pose'
        self.initialpose_topic = '/initialpose'
        self.navigate_action_name = '/navigate_to_pose'
        self.arrival_threshold = 3.0

        self.floors = [
            {'name': 'floor_1', 'x_min': -28.98, 'x_max': 36.97, 'y_min': 1.85, 'y_max': 34.65, 'lift_x': -3.35, 'lift_y': 23.94},
            {'name': 'floor_2', 'x_min': 0.46, 'x_max': 53.80, 'y_min': -32.78, 'y_max': -2.37, 'lift_x': -7.79, 'lift_y': 16.7}
        ]
        self.lift_positions = {
            'floor_1': {'x': -3.35, 'y': 23.94}, #'x': 1.77, 'y': 9.88
            'floor_2': {'x': 17.94, 'y': -7.64} # 'x': -7.79, 'y': 16.7
        }


        self.STATE_IDLE = 'IDLE'
        self.STATE_GOING_TO_GOAL = 'GOING_TO_GOAL'
        self.STATE_GOING_TO_LIFT = 'GOING_TO_LIFT'
        self.STATE_WAITING_FOR_NEW_FLOOR = 'WAITING_FOR_NEW_FLOOR'
        self.STATE_GOING_TO_FINAL_GOAL = 'GOING_TO_FINAL_GOAL'

        self.state = self.STATE_IDLE

        self.current_pose = None
        self.current_floor = None
        self.original_goal = None
        self.original_goal_floor = None
        self.stored_final_goal = None
        self.current_sent_floor = None
        self.current_sent_goal = None
        self._current_goal_handle = None

        self._action_client = ActionClient(self, NavigateToPose, self.navigate_action_name)

        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.create_subscription(PoseStamped, self.goal_topic, self.goal_callback, qos_profile=qos)
        self.create_subscription(PoseWithCovarianceStamped, self.initialpose_topic, self.initialpose_callback, qos_profile=qos)
        self.create_subscription(PoseStamped, self.amcl_topic, self.amcl_pose_callback, qos_profile=qos)

        self.create_timer(0.5, self._timer_cb)

        self.get_logger().info('🚀 MultiFloorManager started with embedded floor and lift data')

    def point_in_region(self, x, y, region):
        in_region = (region['x_min'] <= x <= region['x_max']) and (region['y_min'] <= y <= region['y_max'])
        self.get_logger().debug(f'Checking point ({x:.2f}, {y:.2f}) in region {region["name"]}: {in_region}')
        return in_region

    def get_floor_for_pose(self, pose_stamped: PoseStamped):
        if pose_stamped is None:
            self.get_logger().debug('PoseStamped is None, returning None for floor')
            return None
        x = pose_stamped.pose.position.x
        y = pose_stamped.pose.position.y
        self.get_logger().debug(f'Finding floor for pose ({x:.2f}, {y:.2f})')
        for f in self.floors:
            if self.point_in_region(x, y, f):
                self.get_logger().debug(f'Pose is inside floor {f["name"]}')
                return f['name']
        self.get_logger().debug('Pose does not belong to any defined floor')
        return None

    def dist(self, a: PoseStamped, b: PoseStamped):
        if a is None or b is None:
            return float('inf')
        ax, ay = a.pose.position.x, a.pose.position.y
        bx, by = b.pose.position.x, b.pose.position.y
        d = math.hypot(ax - bx, ay - by)
        self.get_logger().debug(f'Distance between ({ax:.2f},{ay:.2f}) and ({bx:.2f},{by:.2f}) = {d:.2f}')
        return d

    def goal_callback(self, msg: PoseStamped):
        self.get_logger().info(f'[GOAL CALLBACK] Received goal at ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}) in frame {msg.header.frame_id}')

        if msg.header.frame_id != self.map_frame:
            self.get_logger().warn(f'[GOAL CALLBACK] Goal frame mismatch: {msg.header.frame_id} != {self.map_frame}, ignoring goal')
            return

        self.original_goal = msg
        self.original_goal_floor = self.get_floor_for_pose(msg)
        self.get_logger().info(f'[GOAL CALLBACK] Goal floor: {self.original_goal_floor}')

        if self.original_goal_floor is None:
            self.get_logger().warn('[GOAL CALLBACK] Goal is outside defined floors, rejecting')
            return

        if self.current_floor is None:
            self.get_logger().info('[GOAL CALLBACK] Current floor unknown, storing goal and waiting')
            self.state = self.STATE_IDLE
            return

        if self.original_goal_floor == self.current_floor:
            self.get_logger().info('[GOAL CALLBACK] Goal on same floor -> sending directly')
            self.send_goal_to_nav2(self.original_goal)
            self.state = self.STATE_GOING_TO_GOAL
            self.current_sent_goal = self.original_goal
        else:
            self.get_logger().info(f'[GOAL CALLBACK] Goal on different floor ({self.original_goal_floor}) -> computing lift')
            lift_pose = self._get_lift_pose_for_floor(self.current_floor)
            if lift_pose is None:
                self.get_logger().error('[GOAL CALLBACK] No lift pose for current floor, cannot proceed')
                return
            self.stored_final_goal = self.original_goal
            self.send_goal_to_nav2(lift_pose)
            self.current_sent_goal = lift_pose
            self.state = self.STATE_GOING_TO_LIFT

    def amcl_pose_callback(self, msg: PoseStamped):
        self.current_pose = msg
        prev_floor = self.current_floor
        self.current_floor = self.get_floor_for_pose(msg)
        if prev_floor != self.current_floor:
            self.get_logger().info(f'[AMCL CALLBACK] Floor change: {prev_floor} -> {self.current_floor}')
        if self.state == self.STATE_WAITING_FOR_NEW_FLOOR and self.current_floor == self.original_goal_floor:
            self.get_logger().info('[AMCL CALLBACK] Arrived at target floor, sending stored final goal')
            if self.stored_final_goal:
                self.send_goal_to_nav2(self.stored_final_goal)
                self.current_sent_goal = self.stored_final_goal
                self.state = self.STATE_GOING_TO_FINAL_GOAL
                self.stored_final_goal = None

    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        floor = self.get_floor_for_pose(pose)
        self.get_logger().info(f'[INITIALPOSE CALLBACK] Initialpose received, floor: {floor}')
        if floor is not None:
            prev_floor = self.current_floor
            self.current_floor = floor
            self.get_logger().info(f'[INITIALPOSE CALLBACK] Floor set: {self.current_floor} (prev {prev_floor})')
        if self.state == self.STATE_WAITING_FOR_NEW_FLOOR and self.current_floor == self.original_goal_floor:
            self.get_logger().info('[INITIALPOSE CALLBACK] Robot placed on target floor, sending stored final goal')
            if self.stored_final_goal:
                self.send_goal_to_nav2(self.stored_final_goal)
                self.current_sent_goal = self.stored_final_goal
                self.state = self.STATE_GOING_TO_FINAL_GOAL
                self.stored_final_goal = None

    def _get_lift_pose_for_floor(self, floor_name):
        self.get_logger().info(f'[LIFT] Getting lift pose for floor {floor_name}')
        if floor_name not in self.lift_positions:
            self.get_logger().warn(f'[LIFT] No lift position found for floor {floor_name}')
            return None
        p = self.lift_positions[floor_name]
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(p['x'])
        pose.pose.position.y = float(p['y'])
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        return pose

    def send_goal_to_nav2(self, pose_stamped: PoseStamped):
        self.get_logger().info(f'[NAV2] Sending goal at ({pose_stamped.pose.position.x:.2f}, {pose_stamped.pose.position.y:.2f}), state={self.state}')
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('[NAV2] Action server not available')
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self._feedback_cb)
        send_goal_future.add_done_callback(self._goal_response_callback)
        self._current_goal_handle = None

    def _feedback_cb(self, feedback_msg):
        self.get_logger().debug('[NAV2] Feedback received')

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('[NAV2] Goal rejected')
            return
        self.get_logger().info('[NAV2] Goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._get_result_callback)
        self._current_goal_handle = goal_handle

    def _get_result_callback(self, future):
        status = future.result().status
        self.get_logger().info(f'[NAV2] Goal result received: status={status}')
        if self.state == self.STATE_GOING_TO_LIFT:
            self.get_logger().info('[NAV2] Reached lift -> entering WAITING_FOR_NEW_FLOOR')
            self.state = self.STATE_WAITING_FOR_NEW_FLOOR
        elif self.state in [self.STATE_GOING_TO_GOAL, self.STATE_GOING_TO_FINAL_GOAL]:
            self.get_logger().info('[NAV2] Reached final goal -> returning to IDLE')
            self.state = self.STATE_IDLE
            self.original_goal = None
            self.original_goal_floor = None
            self.current_sent_goal = None

    def _timer_cb(self):
        try:
            if self.current_sent_goal and self.current_pose:
                d = self.dist(self.current_sent_goal, self.current_pose)
                self.get_logger().debug(f'[TIMER] Distance to goal: {d:.2f}')
                if d < self.arrival_threshold:
                    self.get_logger().info(f'[TIMER] Within arrival threshold ({self.arrival_threshold})')
                    if self.state == self.STATE_GOING_TO_LIFT:
                        self.get_logger().info('[TIMER] Proximity to lift detected -> entering WAITING_FOR_NEW_FLOOR')
                        try:
                            if self._current_goal_handle:
                                self._current_goal_handle.cancel_goal_async()
                        except Exception:
                            pass
                        self.state = self.STATE_WAITING_FOR_NEW_FLOOR
                    elif self.state == self.STATE_GOING_TO_GOAL:
                        self.get_logger().info('[TIMER] Proximity to final goal detected -> returning to IDLE')
                        self.state = self.STATE_IDLE
                        self.original_goal = None
                        self.original_goal_floor = None
                        self.current_sent_goal = None
        except Exception as e:
            self.get_logger().error(f'[TIMER] Exception: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MultiFloorManager()
    node.get_logger().info("[MAIN] Node started")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
