#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial
import math
import csv
import os
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from nav2_msgs.srv import ClearEntireCostmap
from obstacle_influence.msg import CSVData, CSVPoint
from std_msgs.msg import String

REGION_BOUNDS = {
    'region2_1': {'x_min': 8.0,  'x_max': 12.0, 'y_min': -5.9,  'y_max': -3.0},
    'region2_2': {'x_min': 12.0, 'x_max': 16.0, 'y_min': -5.9,  'y_max': -3.0},
    'region2_3': {'x_min': 16.0, 'x_max': 20.0, 'y_min': -5.9,  'y_max': -3.0},
    'region2_4': {'x_min': 20.0, 'x_max': 24.0, 'y_min': -5.9,  'y_max': -3.0},
    'region4_1': {'x_min': 8.0,  'x_max': 12.0, 'y_min': -15.0, 'y_max': -12.1},
    'region4_2': {'x_min': 12.0, 'x_max': 16.0, 'y_min': -15.0, 'y_max': -12.1},
    'region4_3': {'x_min': 16.0, 'x_max': 20.0, 'y_min': -15.0, 'y_max': -12.1},
    'region4_4': {'x_min': 20.0, 'x_max': 24.0, 'y_min': -15.0, 'y_max': -12.1},
}

REGION_PUBLISH_DURATIONS = {
    'region2_1': 30,
    'region2_2': 20,
    'region2_3': 5,
    'region2_4': 4,
    'region4_1': 900,
    'region4_2': 900,
    'region4_3': 900,
    'region4_4': 900,
}

REGION_CSV_FILES = {
    'region2_1': 'cost2_1.csv',
    'region2_2': 'cost2_2.csv',
    'region2_3': 'cost2_3.csv',
    'region2_4': 'cost2_4.csv',
    'region4_1': 'cost4_1.csv',
    'region4_2': 'cost4_2.csv',
    'region4_3': 'cost4_3.csv',
    'region4_4': 'cost4_4.csv',
}

COST_NON_FILE = 'cost_non.csv'
CSV_FOLDER_PATH = ''

def check_main_region(x, y):
    if 4.0 <= x <= 24.0 and -15.3 <= y <= -11.8:
        return 4
    if 4.0 <= x <= 24.0 and -6.2 <= y <= -2.7:
        return 2
    return None

def find_sub_region(region_id, obs_x, obs_y):
    prefix = f"region{region_id}_"
    for sub_name, bounds in REGION_BOUNDS.items():
        if not sub_name.startswith(prefix):
            continue
        if (bounds['x_min'] <= obs_x <= bounds['x_max'] and
            bounds['y_min'] <= obs_y <= bounds['y_max']):
            return sub_name
    return None

class BlockedPathAndCostPublisher(Node):
    def __init__(self):
        super().__init__('blocked_path_and_cost_publisher')
        self.robots = ['tb1', 'tb2', 'tb3', 'tb4', 'tb5', 'tb6']
        self.partition_enabled = {r: False for r in self.robots}
        self.previous_path_length = {robot: 0.0 for robot in self.robots}
        self.current_pose = {robot: (0.0, 0.0) for robot in self.robots}
        self.csv_data_map = {}
        self.load_all_csv_files()
        self.active_regions = {}
        self.csv_pubs = {}
        for idx, robot in enumerate(self.robots, start=1):
            topic = f'/online{idx}'
            self.csv_pubs[robot] = self.create_publisher(CSVData, topic, 10)
            self.get_logger().info(f"Publisher created: {topic} for {robot}")
        for robot in self.robots:
            plan_topic = f'/{robot}/plan'
            amcl_topic = f'/{robot}/amcl_pose'
            self.create_subscription(Path, plan_topic,
                                     partial(self.path_callback, robot_name=robot),
                                     10)
            self.create_subscription(PoseWithCovarianceStamped, amcl_topic,
                                     partial(self.pose_callback, robot_name=robot),
                                     10)
        self.create_subscription(
            String,
            '/partition_enable',
            self.partition_enable_callback,
            10
        )
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("BlockedPathAndCostPublisher node initialized.")

    def path_callback(self, msg: Path, robot_name: str):
        current_length = self.compute_path_length(msg)
        old_length = self.previous_path_length[robot_name]
        if old_length > 0.0 and current_length >= 1.15 * old_length:
            x, y = self.current_pose[robot_name]
            region_id = check_main_region(x, y)
            self.get_logger().info(f"Check Robot={robot_name} Location, region={region_id}, X={x:.2f}, Y={y:.2f}")
            if region_id is not None:
                self.get_logger().info(f"[Blocked] Robot={robot_name}, region={region_id}, OldLen={old_length:.2f}, NewLen={current_length:.2f}")
                obs_x = x + 5.12
                obs_y = y
                sub_region = find_sub_region(region_id, obs_x, obs_y)
                if sub_region is not None:
                    self.get_logger().info(f" => Dynamic obstacle in {sub_region} (x={obs_x:.2f}, y={obs_y:.2f})")
                    self.activate_region(sub_region, robot_name)
                else:
                    self.get_logger().info(f" => But sub-region undefined. (x={obs_x:.2f}, y={obs_y:.2f})")
            else:
                self.get_logger().info(f"[Blocked] Robot={robot_name}, Region=Undefined, OldLen={old_length:.2f}, NewLen={current_length:.2f}")
            self.previous_path_length[robot_name] = current_length
        else:
            if old_length == 0.0:
                self.get_logger().info(f"[{robot_name}] Initialized path length={current_length:.2f}")
            self.previous_path_length[robot_name] = current_length

    def pose_callback(self, msg: PoseWithCovarianceStamped, robot_name: str):
        pose = msg.pose.pose
        self.current_pose[robot_name] = (pose.position.x, pose.position.y)

    def compute_path_length(self, path_msg: Path):
        total_dist = 0.0
        poses = path_msg.poses
        for i in range(len(poses) - 1):
            p1 = poses[i].pose.position
            p2 = poses[i+1].pose.position
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            dz = p2.z - p1.z
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            total_dist += dist
        return total_dist

    def activate_region(self, region_name, robot_name):
        if region_name not in REGION_PUBLISH_DURATIONS:
            self.get_logger().error(f"No publish duration defined for {region_name}")
            return
        duration = REGION_PUBLISH_DURATIONS[region_name]
        if region_name not in self.active_regions:
            self.active_regions[region_name] = {
                'countdown': duration,
                'robots_triggered': set([robot_name])
            }
            self.get_logger().info(f" => {region_name} activated for {duration} seconds. Trigger robot={robot_name}")
            self.schedule_delayed_clear(robot_name, delay_sec=20.0)
        else:
            data = self.active_regions[region_name]
            if robot_name not in data['robots_triggered']:
                data['robots_triggered'].add(robot_name)
                self.schedule_delayed_clear(robot_name, delay_sec=20.0)

    def clear_costmap_for_robot(self, robot_name: str):
        srv_name = f'/{robot_name}/global_costmap/clear_entirely_global_costmap'
        if not hasattr(self, 'costmap_clients'):
            self.costmap_clients = {}
        if robot_name not in self.costmap_clients:
            self.costmap_clients[robot_name] = self.create_client(
                ClearEntireCostmap, srv_name
            )
        client = self.costmap_clients[robot_name]
        self.get_logger().info(f"[{robot_name}] Before clearing the costmap")
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f"[{robot_name}] {srv_name} not available")
            return
        req = ClearEntireCostmap.Request()
        future = client.call_async(req)
        self.get_logger().info(f"[{robot_name}] Starting Clearing the costmap")
        def _done_cb(fut):
            try:
                fut.result()
                self.get_logger().info(f"[{robot_name}] Global costmap cleared ✔")
            except Exception as e:
                self.get_logger().warn(f"[{robot_name}] Clear costmap failed ✖  {e}")
        future.add_done_callback(_done_cb)

    def schedule_delayed_clear(self, robot_name: str, delay_sec: float = 10.0):
        timer_holder = {}
        def _timer_cb():
            self.clear_costmap_for_robot(robot_name)
            timer_holder['timer'].cancel()
        timer_holder['timer'] = self.create_timer(delay_sec, _timer_cb)

    def load_all_csv_files(self):
        for region_name, csv_file in REGION_CSV_FILES.items():
            full_path = os.path.join(CSV_FOLDER_PATH, csv_file)
            self.csv_data_map[region_name] = self.read_csv_file(full_path)
        non_path = os.path.join(CSV_FOLDER_PATH, COST_NON_FILE)
        self.csv_data_map['none'] = self.read_csv_file(non_path)
        self.get_logger().info("All region CSVs loaded.")

    def read_csv_file(self, file_path):
        points = []
        if not os.path.exists(file_path):
            self.get_logger().error(f"CSV file not found: {file_path}")
            return points
        try:
            with open(file_path, 'r') as f:
                reader = csv.reader(f)
                next(reader, None)
                for row in reader:
                    if len(row) < 3:
                        continue
                    x = float(row[0])
                    y = float(row[1])
                    c = int(row[2])
                    c = max(0, min(c, 255))
                    pt = CSVPoint(x=x, y=y, cost=c)
                    points.append(pt)
        except Exception as e:
            self.get_logger().error(f"Error reading CSV file {file_path}: {e}")
        return points

    def publish_csvdata(self, points):
        for robot, pub in self.csv_pubs.items():
            msg = CSVData()
            msg.enabled = self.partition_enabled.get(robot, False)
            msg.points  = points
            pub.publish(msg)
        self.get_logger().debug(
            f"Published {len(points)} points with enabled flags {self.partition_enabled}"
        )

    def timer_callback(self):
        to_remove = []
        for region_name, data in self.active_regions.items():
            data['countdown'] -= 1
            if data['countdown'] <= 0:
                to_remove.append(region_name)
        all_points = []
        for region_name, data in self.active_regions.items():
            region_points = self.csv_data_map.get(region_name, [])
            all_points.extend(region_points)
        if len(self.active_regions) > 0:
            self.publish_csvdata(all_points)
        else:
            empty_points = self.csv_data_map.get('none', [])
            self.publish_csvdata(empty_points)
        for region_name in to_remove:
            self.active_regions.pop(region_name, None)
            self.get_logger().info(f"{region_name} publish duration ended. -> Deactivated.")
            for robot in self.robots:
                self.schedule_delayed_clear(robot, delay_sec=0.0)
        if len(to_remove) > 0:
            if len(self.active_regions) == 0:
                empty_points = self.csv_data_map.get('none', [])
                self.publish_csvdata(empty_points)
                self.get_logger().info("No active region -> Published cost_non.csv once.")

    def partition_enable_callback(self, msg: String):
        old_state = self.partition_enabled.copy()
        try:
            pairs = [p.strip() for p in msg.data.split(',')]
            for pair in pairs:
                if ':' not in pair:
                    continue
                robot, val = [s.strip() for s in pair.split(':', 1)]
                if robot in self.partition_enabled:
                    self.partition_enabled[robot] = val.lower() == 'true'
        except Exception as e:
            self.get_logger().warn(f"Failed to parse /partition_enable: {e}")
            return
        for robot, new_flag in self.partition_enabled.items():
            if old_state.get(robot, False) and (not new_flag):
                self.get_logger().info(f"{robot} enabled→disabled → clearing costmap")
                self.schedule_delayed_clear(robot, delay_sec=0.0)
        self.get_logger().info(f"partition_enabled updated → {self.partition_enabled}")

def main(args=None):
    rclpy.init(args=args)
    node = BlockedPathAndCostPublisher()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

