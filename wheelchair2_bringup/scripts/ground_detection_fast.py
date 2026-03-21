#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import cv2
from tf2_ros import Buffer, TransformListener, TransformException
from scipy.spatial.transform import Rotation as R
from scipy.stats import binned_statistic_2d
import scipy.ndimage as ndimage

class GroundDetectionRobust(Node):
    def __init__(self):
        super().__init__('ground_detection_robust')

        self.declare_parameter('lidar_topic', '/livox/lidar')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('publish_grid', True)

        # ROI
        self.declare_parameter('min_range', 0.5)      
        self.declare_parameter('max_range', 15.0)     
        self.declare_parameter('min_z_limit', -2.0)

        # --- TUNED PARAMETERS FOR PLATFORMS ---
        # 1. Smaller Cells: Reduces the "ramp error" so ramps don't look like walls.
        self.declare_parameter('cell_size', 0.25)      
        
        # 2. Thickness: Must be > (CellSize * tan(Slope)) + Noise
        # 0.25m * tan(15deg) = 0.06m. Plus noise -> 0.12m is safe.
        self.declare_parameter('ground_thickness', 0.05)
        
        # 3. Step Limit: Heights > this are WALLS. Heights < this are RAMPS.
        # A 12cm platform will be detected as a wall.
        self.declare_parameter('max_step_height', 0.05)

        # Ray Tracing
        self.declare_parameter('out_grid_res', 0.05)
        self.declare_parameter('out_size_xy', 10.0)
        self.declare_parameter('ray_step', 3)
        self.declare_parameter('angular_res_deg', 0.5)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        topic = self.get_parameter('lidar_topic').value
        self.lidar_sub = self.create_subscription(
            PointCloud2, topic, self.lidar_callback, 10)
        
        self.grid_pub = self.create_publisher(OccupancyGrid, '/map_local', 10)
        self.ground_pub = self.create_publisher(PointCloud2, '/points_ground', 10)
        self.nonground_pub = self.create_publisher(PointCloud2, '/points_nonground', 10)

        self.get_logger().info("Robust Small-Cell Ground Detection Started.")

    def lidar_callback(self, msg):
        target_frame = self.get_parameter('target_frame').value
        try:
            t = self.tf_buffer.lookup_transform(
                target_frame, msg.header.frame_id, rclpy.time.Time())
        except TransformException:
            return

        points = self.get_points_numpy(msg)
        if points is None or len(points) < 10: return
        points = self.apply_transform(points, t)

        # Filter
        min_r = self.get_parameter('min_range').value
        max_r = self.get_parameter('max_range').value
        min_z = self.get_parameter('min_z_limit').value

        dists_sq = points[:, 0]**2 + points[:, 1]**2
        mask_valid = (dists_sq >= min_r**2) & \
                     (dists_sq <= max_r**2) & \
                     (points[:, 2] > min_z)
        points = points[mask_valid]
        if len(points) < 10: return

        # Segment
        gnd_mask = self.segment_ground_grid(points)
        gnd_pts = points[gnd_mask]
        obs_pts = points[~gnd_mask]

        # Occlusion Filter
        gnd_pts_visible = self.filter_occluded_ground(gnd_pts, obs_pts)

        header = Header()
        header.frame_id = target_frame
        header.stamp = self.get_clock().now().to_msg()

        if self.get_parameter('publish_grid').value:
            self.generate_raytraced_grid(obs_pts, gnd_pts_visible, header)

        self.publish_pcd(gnd_pts, self.ground_pub, header)
        self.publish_pcd(obs_pts, self.nonground_pub, header)

    def segment_ground_grid(self, points):
        cell_size = self.get_parameter('cell_size').value
        thickness = self.get_parameter('ground_thickness').value
        max_step = self.get_parameter('max_step_height').value

        min_x, max_x = np.min(points[:,0]), np.max(points[:,0])
        min_y, max_y = np.min(points[:,1]), np.max(points[:,1])
        
        width_x = int(np.ceil((max_x - min_x) / cell_size)) + 1
        width_y = int(np.ceil((max_y - min_y) / cell_size)) + 1
        
        x_bins = min_x + np.arange(width_x + 1) * cell_size
        y_bins = min_y + np.arange(width_y + 1) * cell_size

        # 1. Raw Grid Minima
        stat_res = binned_statistic_2d(
            points[:, 0], points[:, 1], points[:, 2], 
            statistic='min', bins=[x_bins, y_bins]
        )
        min_z_grid = stat_res.statistic

        # 2. INTERPOLATE HOLES (Fix for Small Cells)
        # Identify empty cells (NaNs) and valid cells
        mask_nan = np.isnan(min_z_grid)
        if np.all(mask_nan): return np.zeros(len(points), dtype=bool)

        # Use Nearest Neighbor to fill NaNs
        # (ndimage.distance_transform_edt provides indices of nearest valid pixel)
        indices = ndimage.distance_transform_edt(mask_nan, return_distances=False, return_indices=True)
        filled_grid = min_z_grid[tuple(indices)]

        # 3. SMOOTH SURFACE (Median Filter)
        # This handles the noise stability
        smooth_surface = ndimage.median_filter(filled_grid, size=3, mode='reflect')
        
        # 4. STEP DETECTION
        step_diff = filled_grid - smooth_surface
        
        # A cell is ground if it matches the smooth surface
        cell_is_flat = (step_diff < max_step)

        # 5. MORPHOLOGICAL CLOSING (The Anti-Flicker Fix)
        # If a "Wall" cell is surrounded by "Ground" cells, it's probably just noise. Make it ground.
        # We perform binary closing on the "Ground Mask"
        struct = ndimage.generate_binary_structure(2, 2) # 8-connected
        cell_is_flat_clean = ndimage.binary_closing(cell_is_flat, structure=struct, iterations=1)

        # 6. Map Points
        xi = ((points[:, 0] - min_x) / cell_size).astype(np.int32)
        yi = ((points[:, 1] - min_y) / cell_size).astype(np.int32)
        xi = np.clip(xi, 0, min_z_grid.shape[0] - 1)
        yi = np.clip(yi, 0, min_z_grid.shape[1] - 1)

        # 7. Final Check
        # Point is ground if:
        # A) Its cell is flat (from the clean grid)
        # B) Point height matches the INTERPOLATED floor height
        local_floor = filled_grid[xi, yi]
        is_ground = cell_is_flat_clean[xi, yi] & ((points[:, 2] - local_floor) < thickness)

        return is_ground

    def filter_occluded_ground(self, gnd_pts, obs_pts):
        if len(obs_pts) == 0 or len(gnd_pts) == 0: return gnd_pts
        ang_res = np.radians(self.get_parameter('angular_res_deg').value)
        
        obs_ang = np.arctan2(obs_pts[:, 1], obs_pts[:, 0])
        obs_dist = np.sqrt(obs_pts[:, 0]**2 + obs_pts[:, 1]**2)
        gnd_ang = np.arctan2(gnd_pts[:, 1], gnd_pts[:, 0])
        gnd_dist = np.sqrt(gnd_pts[:, 0]**2 + gnd_pts[:, 1]**2)

        obs_bins = ((obs_ang + np.pi) / ang_res).astype(np.int32)
        gnd_bins = ((gnd_ang + np.pi) / ang_res).astype(np.int32)
        
        max_bin = int((2 * np.pi) / ang_res) + 1
        min_obs_dist = np.full(max_bin, 999.0, dtype=np.float32)
        np.minimum.at(min_obs_dist, obs_bins, obs_dist)
        
        gnd_bins = np.clip(gnd_bins, 0, max_bin - 1)
        limit_dist = min_obs_dist[gnd_bins]
        
        visible_mask = gnd_dist < (limit_dist + 0.2)
        return gnd_pts[visible_mask]

    def generate_raytraced_grid(self, obs_pts, gnd_pts, header):
        res = self.get_parameter('out_grid_res').value
        size = self.get_parameter('out_size_xy').value
        ray_step = self.get_parameter('ray_step').value

        width = int(size / res)
        height = int(size / res)
        cx, cy = width // 2, height // 2
        
        grid_img = np.full((height, width), 128, dtype=np.uint8)

        def to_grid(pts):
            if len(pts) == 0: return None
            cs = ((pts[:, 0] / res) + cx).astype(np.int32)
            rs = ((pts[:, 1] / res) + cy).astype(np.int32)
            return np.column_stack((cs, rs))

        if len(gnd_pts) > 0 and len(obs_pts) > 0:
            all_endpoints = np.vstack((gnd_pts[::ray_step], obs_pts[::ray_step]))
        elif len(gnd_pts) > 0:
            all_endpoints = gnd_pts[::ray_step]
        else:
            all_endpoints = obs_pts[::ray_step]

        grid_endpoints = to_grid(all_endpoints)
        robot_pt = (int(cx), int(cy))
        
        if grid_endpoints is not None:
            grid_endpoints[:, 0] = np.clip(grid_endpoints[:, 0], 0, width-1)
            grid_endpoints[:, 1] = np.clip(grid_endpoints[:, 1], 0, height-1)
            for pt in grid_endpoints:
                cv2.line(grid_img, robot_pt, (pt[0], pt[1]), 0, 1)

        obs_grid = to_grid(obs_pts)
        if obs_grid is not None:
            valid_mask = (obs_grid[:, 0] >= 0) & (obs_grid[:, 0] < width) & \
                         (obs_grid[:, 1] >= 0) & (obs_grid[:, 1] < height)
            valid_obs = obs_grid[valid_mask]
            grid_img[valid_obs[:, 1], valid_obs[:, 0]] = 255

        msg = OccupancyGrid()
        msg.header = header
        msg.info.resolution = res
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = -size / 2.0
        msg.info.origin.position.y = -size / 2.0
        msg.info.origin.position.z = 0.0
        
        flat_img = grid_img.flatten()
        ros_data = np.full_like(flat_img, -1, dtype=np.int8)
        ros_data[flat_img == 0] = 0
        ros_data[flat_img == 255] = 100
        msg.data = ros_data.tolist()
        self.grid_pub.publish(msg)

    def get_points_numpy(self, msg):
        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        raw_list = list(gen)
        if not raw_list: return None
        raw_arr = np.array(raw_list)
        try:
            return np.column_stack((raw_arr['x'], raw_arr['y'], raw_arr['z'])).astype(np.float32)
        except:
            return np.column_stack((raw_arr['X'], raw_arr['Y'], raw_arr['Z'])).astype(np.float32)

    def apply_transform(self, points, t):
        trans = t.transform.translation
        rot = t.transform.rotation
        r = R.from_quat([rot.x, rot.y, rot.z, rot.w])
        return points @ r.as_matrix().T + np.array([trans.x, trans.y, trans.z])

    def publish_pcd(self, points, publisher, header):
        if len(points) == 0: return
        msg = pc2.create_cloud_xyz32(header, points)
        publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GroundDetectionRobust()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()