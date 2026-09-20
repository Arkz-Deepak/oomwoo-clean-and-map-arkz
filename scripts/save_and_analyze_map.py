#!/usr/bin/env python3
"""
Save and Analyze SLAM Map for OOMWOO Clean-and-Map
==================================================
Subscribes to /map from slam_toolbox, saves standard PGM + YAML map files,
and performs geometrical and coverage analysis of the mapped space.
"""

import os
import sys
import time
import argparse
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool


def latched_qos():
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
    )


class MapSaverAndAnalyzer(Node):
    def __init__(self, output_dir: str, target_area: float = 36.0):
        super().__init__('map_saver_and_analyzer')
        self.output_dir = output_dir
        self.target_area = target_area
        self.map_msg = None
        self.cleaning_active = None
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            latched_qos()
        )
        self.cleaning_sub = self.create_subscription(
            Bool,
            '/coverage_planner/cleaning_active',
            self.cleaning_callback,
            10
        )
        self.get_logger().info('MapSaverAndAnalyzer waiting for /map...')

    def cleaning_callback(self, msg: Bool):
        self.cleaning_active = msg.data

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.get_logger().info(f'Received /map: {msg.info.width}x{msg.info.height} @ {msg.info.resolution:.3f}m')

    def save_and_analyze(self):
        if self.map_msg is None:
            self.get_logger().error('No map received!')
            return None

        info = self.map_msg.info
        w, h = info.width, info.height
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        data = np.array(self.map_msg.data, dtype=np.int8).reshape((h, w))

        os.makedirs(self.output_dir, exist_ok=True)
        pgm_path = os.path.join(self.output_dir, 'square_room_map.pgm')
        yaml_path = os.path.join(self.output_dir, 'square_room_map.yaml')
        report_path = os.path.join(self.output_dir, 'map_analysis.txt')

        # Convert to PGM grayscale (0=black=occupied, 254=white=free, 205=gray=unknown)
        pgm_img = np.full((h, w), 205, dtype=np.uint8)
        pgm_img[data == 0] = 254
        pgm_img[data >= 50] = 0

        # Invert row order for standard ROS map image (y=0 at bottom)
        pgm_img = np.flipud(pgm_img)

        # Write PGM (P5 binary)
        with open(pgm_path, 'wb') as f:
            header = f"P5\n{w} {h}\n255\n".encode('ascii')
            f.write(header)
            f.write(pgm_img.tobytes())

        # Write YAML
        yaml_content = f"""image: square_room_map.pgm
mode: trinary
resolution: {res}
origin: [{ox:.4f}, {oy:.4f}, 0.0000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
        with open(yaml_path, 'w') as f:
            f.write(yaml_content)

        # Statistical & Geometric Analysis
        total_cells = w * h
        free_cells = int(np.sum(data == 0))
        occupied_cells = int(np.sum(data >= 50))
        unknown_cells = int(np.sum(data == -1))

        free_area_m2 = free_cells * (res ** 2)
        occupied_area_m2 = occupied_cells * (res ** 2)

        # Find bounds of free and occupied space
        occupied_ys, occupied_xs = np.where(data >= 50)
        free_ys, free_xs = np.where(data == 0)

        if len(occupied_xs) > 0:
            wall_min_x = ox + np.min(occupied_xs) * res
            wall_max_x = ox + np.max(occupied_xs) * res
            wall_min_y = oy + np.min(occupied_ys) * res
            wall_max_y = oy + np.max(occupied_ys) * res
            est_width_m = wall_max_x - wall_min_x
            est_height_m = wall_max_y - wall_min_y
        else:
            wall_min_x = wall_max_x = wall_min_y = wall_max_y = 0.0
            est_width_m = est_height_m = 0.0

        if len(free_xs) > 0:
            free_min_x = ox + np.min(free_xs) * res
            free_max_x = ox + np.max(free_xs) * res
            free_min_y = oy + np.min(free_ys) * res
            free_max_y = oy + np.max(free_ys) * res
        else:
            free_min_x = free_max_x = free_min_y = free_max_y = 0.0

        coverage_pct = (free_area_m2 / self.target_area) * 100.0 if self.target_area > 0 else 0.0

        report = f"""==================================================
      OOMWOO SLAM MAP GENERATION & ANALYSIS REPORT
==================================================
Output Files:
  - PGM Image: {pgm_path}
  - YAML Config: {yaml_path}

Grid Specifications:
  - Resolution: {res:.4f} m/cell
  - Grid Size: {w} x {h} cells
  - Total Map Extent: {w * res:.2f} m x {h * res:.2f} m
  - Origin: ({ox:.3f}, {oy:.3f})

Occupancy Breakdown:
  - Free Cells (Floor): {free_cells:,} ({free_cells / total_cells * 100:.1f}%)
  - Occupied Cells (Walls): {occupied_cells:,} ({occupied_cells / total_cells * 100:.1f}%)
  - Unknown Cells: {unknown_cells:,} ({unknown_cells / total_cells * 100:.1f}%)

Geometric Reconstruction:
  - Mapped Free Area: {free_area_m2:.2f} m^2 (Target: {self.target_area:.2f} m^2)
  - Estimated Room Width (X): {est_width_m:.2f} m (Expected: 6.00 m)
  - Estimated Room Length (Y): {est_height_m:.2f} m (Expected: 6.00 m)
  - Wall X Bounds: [{wall_min_x:.2f} m, {wall_max_x:.2f} m]
  - Wall Y Bounds: [{wall_min_y:.2f} m, {wall_max_y:.2f} m]
  - Interior Free Bounds: X in [{free_min_x:.2f}, {free_max_x:.2f}], Y in [{free_min_y:.2f}, {free_max_y:.2f}]

Quality Assessment:
  - Coverage Completeness: {coverage_pct:.1f}% of 6x6m floor area mapped
  - Aspect Ratio: {est_width_m / est_height_m:.2f} (Expected: 1.00 for square room)
==================================================
"""
        with open(report_path, 'w') as f:
            f.write(report)

        print(report)
        return report


def main():
    parser = argparse.ArgumentParser(description='Save and analyze SLAM map')
    parser.add_argument('output_dir', nargs='?', default='/home/deepak-r/Projects/oomwoo-clean-and-map-arkz/maps', help='Directory to save map files')
    parser.add_argument('--duration', type=float, default=20.0, help='Seconds to observe/update map after first map received')
    parser.add_argument('--wait-completion', action='store_true', help='Wait until cleaning completes')
    parser.add_argument('--timeout', type=float, default=300.0, help='Maximum wait timeout in seconds')
    args = parser.parse_args()

    rclpy.init()
    node = MapSaverAndAnalyzer(args.output_dir)

    start_time = time.time()
    first_map_time = None
    started_cleaning = False

    while rclpy.ok() and (time.time() - start_time) < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.map_msg is not None and first_map_time is None:
            first_map_time = time.time()
            node.get_logger().info('First /map received. Observing updates...')

        if node.cleaning_active is True:
            started_cleaning = True

        if args.wait_completion:
            if started_cleaning and node.cleaning_active is False:
                node.get_logger().info('Coverage cleaning reported complete!')
                time.sleep(2.0)
                rclpy.spin_once(node, timeout_sec=1.0)
                break
        elif first_map_time is not None and (time.time() - first_map_time) >= args.duration:
            break

    node.save_and_analyze()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
