#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
from scipy import ndimage
import time

from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.duration import Duration

class GroundDetectionNode(Node):
    def __init__(self):
        super().__init__('ground_detection_raytrace')

        # --- Parameters ---
        self.declare_parameter('lidar_topic', '/livox/lidar')
        self.declare_parameter('imu_topic', '/livox/imu')
        self.declare_parameter('target_frame', 'base_link')
        
        # --- FIXED GRID PARAMETERS (10m x 10m) ---
        self.declare_parameter('grid_size_x', 10.0)
        self.declare_parameter('grid_size_y', 10.0)
        self.declare_parameter('grid_res', 0.05)

        # --- Raytracing ---
        self.declare_parameter('min_ray_range', 0.5)
        self.declare_parameter('max_ray_range', 15.0) 
        self.declare_parameter('morphological_smooth_size', 3)

        # --- Ground Logic ---
        self.declare_parameter('expected_ground_z', -0.15) 
        self.declare_parameter('z_search_tolerance', 0.25) 
        self.declare_parameter('max_robot_tilt_deg', 10.0)
        self.declare_parameter('ransac_dist_thresh', 0.1)
        self.declare_parameter('max_obs_height', 1.6)
        
        self.voxel_size = 0.05           

        # --- TF & State ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.current_roll = 0.0
        self.current_pitch = 0.0

        # --- Pub/Sub ---
        self.lidar_sub = self.create_subscription(
            PointCloud2, self.get_parameter('lidar_topic').value, self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, self.get_parameter('imu_topic').value, self.imu_callback, 10)

        self.grid_pub = self.create_publisher(OccupancyGrid, '/map_local', 10)
        self.ground_pub = self.create_publisher(PointCloud2, '/points_ground', 10)
        self.nonground_pub = self.create_publisher(PointCloud2, '/points_nonground', 10)

        self.get_logger().info("Fixed-Grid Ground Detection Started.")

    def imu_callback(self, msg):
        try:
            q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
            rot = R.from_quat(q)
            yaw, pitch, roll = rot.as_euler('zyx', degrees=True)
            self.current_pitch = pitch
            self.current_roll = roll
        except Exception:
            pass

    def lidar_callback(self, msg):
        # 1. Safety Check
        max_tilt = self.get_parameter('max_robot_tilt_deg').value
        if abs(self.current_roll) > max_tilt or abs(self.current_pitch) > max_tilt:
            return

        # 2. Transform
        target_frame = self.get_parameter('target_frame').value
        try:
            t = self.tf_buffer.lookup_transform(
                target_frame, 
                msg.header.frame_id, 
                msg.header.stamp, 
                Duration(seconds=0.2)
            )
        except TransformException:
            try:
                t = self.tf_buffer.lookup_transform(target_frame, msg.header.frame_id, rclpy.time.Time())
            except TransformException:
                return

        # 3. Convert ROS -> Open3D
        pcd = self.convert_ros_to_o3d(msg)
        if pcd is None or len(pcd.points) < 10: return

        # Apply TF
        transform_matrix = self.ros_transform_to_matrix(t)
        pcd.transform(transform_matrix)

        # 4. Downsample
        pcd_down = pcd.voxel_down_sample(voxel_size=self.voxel_size)
        
        # 5. Range Filter
        points_temp = np.asarray(pcd_down.points)
        dists = np.linalg.norm(points_temp[:, :2], axis=1)
        min_r = self.get_parameter('min_ray_range').value
        max_r = self.get_parameter('max_ray_range').value
        range_mask = (dists >= min_r) & (dists <= max_r)
        
        if np.sum(range_mask) < 10: return
        pcd_down = pcd_down.select_by_index(np.where(range_mask)[0])

        # 6. Run Logic
        new_header = Header()
        new_header.frame_id = target_frame
        # new_header.stamp = msg.header.stamp
        new_header.stamp = self.get_clock().now().to_msg()
        
        nonground_pts, gnd_pts = self.process_ransac_3d(pcd_down)
        # occ_grid, nonground_pts, gnd_pts = self.process_ransac_3d(pcd_down)

        # 7. Publish
        # if occ_grid is not None:
            # Note: origin/shape are now derived from parameters inside publish_occupancy_grid
            # self.publish_occupancy_grid(occ_grid, new_header)
        self.publish_pcd(gnd_pts, self.ground_pub, new_header)
        self.publish_pcd(nonground_pts, self.nonground_pub, new_header)

    def process_ransac_3d(self, pcd):
        points = np.asarray(pcd.points)
        if len(points) == 0: return None, None, None
        
        exp_z = self.get_parameter('expected_ground_z').value
        tol = self.get_parameter('z_search_tolerance').value
        dist_thresh = self.get_parameter('ransac_dist_thresh').value
        
        z_min = exp_z - tol
        z_max = exp_z + tol
        
        candidate_mask = (points[:, 2] > z_min) & (points[:, 2] < z_max)
        candidate_indices = np.where(candidate_mask)[0]
        
        ground_mask = np.zeros(len(points), dtype=bool)
        plane_model = None

        if len(candidate_indices) > 50:
            candidate_pcd = pcd.select_by_index(candidate_indices)
            plane_model, inliers = candidate_pcd.segment_plane(
                distance_threshold=dist_thresh, ransac_n=3, num_iterations=50
            )
            [a, b, c, d] = plane_model
            distances = np.abs(a * points[:, 0] + b * points[:, 1] + c * points[:, 2] + d)
            ground_mask = distances < dist_thresh
            
            normal = np.array([a, b, c])
            if np.degrees(np.arccos(np.abs(np.dot(normal, [0, 0, 1])))) > 25.0:
                 ground_mask = np.zeros(len(points), dtype=bool)
                 plane_model = None

        return self.generate_fixed_grid(points, ground_mask, plane_model, exp_z)

    def generate_fixed_grid(self, points, ground_mask, plane_model, fallback_z):
        nonground_pts = points[~ground_mask]
        gnd_pts = points[ground_mask]
        
        # # --- FIXED MAP CONFIGURATION ---
        # res = self.get_parameter('grid_res').value
        # size_x = self.get_parameter('grid_size_x').value
        # size_y = self.get_parameter('grid_size_y').value
        
        # cols = int(size_x / res)
        # rows = int(size_y / res)
        
        # # Origin (Bottom-Left) relative to Robot (0,0)
        # x_min = -size_x / 2.0
        # y_min = -size_y / 2.0
        
        # # Init Grid (-1 = Unknown)
        # occ_grid = np.full((rows, cols), -1, dtype=np.int8)
        
        # # Robot is at center index
        # r_robot = int(rows / 2)
        # c_robot = int(cols / 2)
        
        # max_obs_height = self.get_parameter('max_obs_height').value

        # # --- 1. IDENTIFY OBSTACLES ---
        # obs_cells = set()
        
        # if len(nonground_pts) > 0:
        #     if plane_model is not None:
        #         [a, b, c, d] = plane_model
        #         if abs(c) < 1e-6: c = 1e-6
        #         z_ground = -(a * nonground_pts[:, 0] + b * nonground_pts[:, 1] + d) / c
        #         z_diff = nonground_pts[:, 2] - z_ground
        #     else:
        #         z_diff = nonground_pts[:, 2] - fallback_z
            
        #     # Logic: >8cm is Obstacle, <-20cm is Drop-off
        #     valid_obs_mask = ((z_diff > 0.08) & (z_diff < max_obs_height)) | (z_diff < -0.20)
        #     valid_obs_pts = nonground_pts[valid_obs_mask]

        #     if len(valid_obs_pts) > 0:
        #         # Vectorized coordinate calculation
        #         cs = ((valid_obs_pts[:, 0] - x_min) / res).astype(int)
        #         rs = ((valid_obs_pts[:, 1] - y_min) / res).astype(int)
                
        #         valid = (cs >= 0) & (cs < cols) & (rs >= 0) & (rs < rows)
                
        #         # Mark on Grid
        #         occ_grid[rs[valid], cs[valid]] = 100
                
        #         for r, c in zip(rs[valid], cs[valid]):
        #             obs_cells.add((r, c))

        # # --- 2. RAYTRACE GROUND ---
        # if len(gnd_pts) > 0:
        #     cs = ((gnd_pts[:, 0] - x_min) / res).astype(int)
        #     rs = ((gnd_pts[:, 1] - y_min) / res).astype(int)
        #     valid = (cs >= 0) & (cs < cols) & (rs >= 0) & (rs < rows)
            
        #     gnd_cells = set(zip(rs[valid], cs[valid]))
            
        #     for (r_end, c_end) in gnd_cells:
        #         line = self.bresenham_line(r_robot, c_robot, r_end, c_end)
        #         for r, c in line:
        #             if 0 <= r < rows and 0 <= c < cols:
        #                 if occ_grid[r, c] == 100: break # Collision
        #                 occ_grid[r, c] = 0

        # # --- 3. RAYTRACE OBSTACLES ---
        # for (r_end, c_end) in obs_cells:
        #     line = self.bresenham_line(r_robot, c_robot, r_end, c_end)
        #     for i in range(len(line) - 1): # Skip last point (obstacle)
        #         r, c = line[i]
        #         if 0 <= r < rows and 0 <= c < cols:
        #             if occ_grid[r, c] == 100: break
        #             occ_grid[r, c] = 0

        # # Ensure Robot cell is free
        # if 0 <= r_robot < rows and 0 <= c_robot < cols:
        #     occ_grid[r_robot, c_robot] = 0

        # # --- Morphological Smoothing ---
        # k_size = self.get_parameter('morphological_smooth_size').value
        # if k_size > 0:
        #     free_mask = (occ_grid == 0)
        #     structure = np.ones((k_size, k_size))
        #     closed_mask = ndimage.binary_closing(free_mask, structure=structure)
        #     safe_fill = closed_mask & (occ_grid != 100)
        #     occ_grid[safe_fill] = 0

        # return occ_grid, nonground_pts, gnd_pts
        return nonground_pts, gnd_pts

    def bresenham_line(self, x0, y0, x1, y1):
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        points.append((x1, y1))
        return points

    # --- TF & ROS Helpers ---
    def ros_transform_to_matrix(self, t):
        tx, ty, tz = t.transform.translation.x, t.transform.translation.y, t.transform.translation.z
        qx, qy, qz, qw = t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w
        r = R.from_quat([qx, qy, qz, qw])
        mat = np.eye(4)
        mat[:3, :3] = r.as_matrix()
        mat[:3, 3] = [tx, ty, tz]
        return mat

    def convert_ros_to_o3d(self, ros_msg):
        gen = pc2.read_points(ros_msg, field_names=("x", "y", "z"), skip_nans=True)
        points_list = [[p[0], p[1], p[2]] for p in gen]
        if not points_list: return None
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points_list, dtype=np.float32))
        return pcd

    def publish_pcd(self, points_vec, publisher, header):
        if len(points_vec) == 0: return
        cloud_msg = pc2.create_cloud_xyz32(header, np.asarray(points_vec))
        publisher.publish(cloud_msg)

    def publish_occupancy_grid(self, grid, header):
        msg = OccupancyGrid()
        msg.header = header
        
        # Retrieve Fixed Params
        res = self.get_parameter('grid_res').value
        size_x = self.get_parameter('grid_size_x').value
        size_y = self.get_parameter('grid_size_y').value
        
        msg.info.resolution = res
        msg.info.width = int(size_x / res)
        msg.info.height = int(size_y / res)
        
        # Force Origin to Center
        msg.info.origin.position.x = -size_x / 2.0
        msg.info.origin.position.y = -size_y / 2.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        flat_grid = grid.flatten()
        msg.data = flat_grid.astype(np.int8).tolist()
        self.grid_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GroundDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()