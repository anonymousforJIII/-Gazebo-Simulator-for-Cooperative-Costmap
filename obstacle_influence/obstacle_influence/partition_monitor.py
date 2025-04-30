#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
from action_msgs.msg import GoalStatusArray

def quaternion_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

def in_range(v, lo, hi): return lo <= v <= hi
def _uuid_to_hex(u):     return ''.join(f'{b:02x}' for b in u)

class PartitionMonitor(Node):
    def __init__(self):
        super().__init__('partition_monitor')

        self.robots = [f'tb{i}' for i in range(1, 7)]
        self.pose:        Dict[str, Optional[PoseWithCovarianceStamped]] = {r: None for r in self.robots}
        self.goal:        Dict[str, Optional[PoseStamped]]               = {r: None for r in self.robots}
        self.navigating:  Dict[str, bool]                                = {r: False for r in self.robots}
        self.latched:     Dict[str, bool]                                = {r: False for r in self.robots}
        self.cur_goal_id: Dict[str, Optional[str]]                       = {r: None for r in self.robots}
        self.latched_id:  Dict[str, Optional[str]]                       = {r: None for r in self.robots}

        for ns in self.robots:
            self.create_subscription(
                PoseWithCovarianceStamped, f'/{ns}/amcl_pose',
                lambda m, ns=ns: self._update_pose(ns, m), 10)
            self.create_subscription(
                PoseStamped, f'/{ns}/goal_pose',
                lambda m, ns=ns: self._update_goal(ns, m), 10)
            self.create_subscription(
                GoalStatusArray, f'/{ns}/navigate_to_pose/_action/status',
                lambda m, ns=ns: self._update_status(ns, m), 10)

        self.pub = self.create_publisher(String, 'partition_enable', 10)
        self.create_timer(1.0, self._timer_cb)
        self.get_logger().info('Partition Monitor Node Start')

    def _update_pose(self, ns, msg): self.pose[ns] = msg
    def _update_goal(self, ns, msg): self.goal[ns] = msg

    def _update_status(self, ns, msg):
        active_entry = next((st for st in msg.status_list if st.status in (1, 2, 3)), None)
        if active_entry:
            self.cur_goal_id[ns] = _uuid_to_hex(active_entry.goal_info.goal_id.uuid)

        if self.latched[ns] and self.latched_id[ns]:
            done = False
            for st in msg.status_list:
                if _uuid_to_hex(st.goal_info.goal_id.uuid) == self.latched_id[ns]:
                    done = (st.status == 4)
                    break
            else:
                done = True
            self.navigating[ns] = not done
        else:
            self.navigating[ns] = bool(active_entry)

    def _in_partition1(self, x, y):  return in_range(x, -4.0, 8.0)   and in_range(y, -20.0, 2.0)
    def _in_partition2(self, x, y):  return in_range(x, 20.0, 30.0) and in_range(y, -20.0, 2.0)
    def _facing_front(self, yaw):    return -math.pi/2 <= yaw <=  math.pi/2
    def _facing_back (self, yaw):    return  math.pi/2 <= yaw or yaw <= -math.pi/2

    def _timer_cb(self):
        flags = {ns: False for ns in self.robots}

        for ns in self.robots:
            if not (self.pose[ns] and self.goal[ns]):
                self.get_logger().info(f'{ns}: pose/goal receive X')
                continue

            cur_x = self.pose[ns].pose.pose.position.x
            cur_y = self.pose[ns].pose.pose.position.y
            goal_x = self.goal[ns].pose.position.x
            ori = self.pose[ns].pose.pose.orientation
            yaw    = quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)

            xgap_ok = abs(cur_x - goal_x) >= 16.0
            p1, p2  = self._in_partition1(cur_x, cur_y), self._in_partition2(cur_x, cur_y)
            front, back = self._facing_front(yaw), self._facing_back(yaw)

            if self.latched[ns]:
                if not self.navigating[ns]:
                    self.latched[ns] = False
                    self.latched_id[ns] = None
            else:
                if self.navigating[ns] and xgap_ok and ((p1 and front) or (p2 and back)):
                    self.latched[ns] = True
                    self.latched_id[ns] = self.cur_goal_id[ns]

            flags[ns] = self.latched[ns]

            self.get_logger().info(
                f'{ns}: pos=({cur_x:+.2f},{cur_y:+.2f})  goal_x={goal_x:+.2f}  '
                f'|Δx|={abs(cur_x-goal_x):.2f}  yaw={math.degrees(yaw):+.1f}°  '
                f'gapOK={xgap_ok}  P1={p1} P2={p2}  F={front} B={back}  '
                f'navigating={self.navigating[ns]}  latched={self.latched[ns]}'
            )

        msg           = String()
        msg.data      = ', '.join(f'{ns}: {str(flags[ns]).lower()}' for ns in self.robots)
        self.pub.publish(msg)
        self.get_logger().info(f'partition_enable → {msg.data}')

def main():
    rclpy.init()
    node = PartitionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

