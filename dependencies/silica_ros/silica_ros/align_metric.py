#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from cv_bridge import CvBridge
import message_filters

import numpy as np
import cv2

# ==========================================
# MATH CORE: SIMPLIFIED LEAST SQUARES ALIGNMENT
# ==========================================

def depth2disparity(depth, return_mask=False):
    if isinstance(depth, np.ndarray):
        disparity = np.zeros_like(depth)
    non_negative_mask = depth > 0
    disparity[non_negative_mask] = 1.0 / depth[non_negative_mask]
    if return_mask:
        return disparity, non_negative_mask
    else:
        return disparity

def disparity2depth(disparity, **kwargs):
    return depth2disparity(disparity, **kwargs)

def align_depth_least_square(
    gt_arr: np.ndarray,
    pred_arr: np.ndarray,
    valid_mask_arr: np.ndarray,
    return_scale_shift=False,
):
    ori_shape = pred_arr.shape  

    gt = gt_arr.squeeze()  
    pred = pred_arr.squeeze()
    valid_mask = valid_mask_arr.squeeze()

    gt_masked = gt[valid_mask].reshape((-1, 1))
    pred_masked = pred[valid_mask].reshape((-1, 1))

    # Guard against completely empty masks
    if len(pred_masked) < 10:
        if return_scale_shift:
            return pred_arr, 1.0, 0.0
        return pred_arr

    _ones = np.ones_like(pred_masked)
    A = np.concatenate([pred_masked, _ones], axis=-1)
    
    X = np.linalg.lstsq(A, gt_masked, rcond=None)[0]
    scale, shift = X
    aligned_pred = pred_arr * scale + shift
    aligned_pred = aligned_pred.reshape(ori_shape)

    if return_scale_shift:
        return aligned_pred, scale, shift
    else:
        return aligned_pred

# ==========================================
# ROS 2 NODE INFRASTRUCTURE
# ==========================================

class AlignMetricNode(Node):
    def __init__(self):
        super().__init__('align_metric')
        
        # 1. Declare Parameters
        self.declare_parameter('sub_topic_raw_depth', '/camera/depth/image_rect_raw')
        self.declare_parameter('sub_topic_raw_rgb', '/camera/color/image_raw')
        self.declare_parameter('sub_topic_camera_info', '/camera/depth/camera_info')
        self.declare_parameter('sub_topic_silica_depth', '/silica/aligned_depth')
        self.declare_parameter('sub_topic_silica_mask', '/silica/segmentation_mask')
        
        self.declare_parameter('pub_topic_glass_depth', '/glass_aware/depth/image_rect_raw')
        self.declare_parameter('pub_topic_glass_rgb', '/glass_aware/color/image_overlay')
        self.declare_parameter('pub_topic_glass_pc', '/glass_aware/depth/color/points')
        
        self.declare_parameter('mask_threshold', 127)
        self.declare_parameter('pc_decimation', 4)

        self.threshold = self.get_parameter('mask_threshold').value
        self.pc_step = self.get_parameter('pc_decimation').value
        
        self.bridge = CvBridge()
        self.intrinsics = None
        
        # 2. Publishers
        self.pub_depth = self.create_publisher(Image, self.get_parameter('pub_topic_glass_depth').value, 10)
        self.pub_rgb = self.create_publisher(Image, self.get_parameter('pub_topic_glass_rgb').value, 10)
        self.pub_pc = self.create_publisher(PointCloud2, self.get_parameter('pub_topic_glass_pc').value, 10)
        
        # 3. Subscribers
        self.sub_info = self.create_subscription(
            CameraInfo,
            self.get_parameter('sub_topic_camera_info').value,
            self.info_callback,
            10
        )
        
        # Synchronized Subscribers
        self.sub_raw_depth = message_filters.Subscriber(self, Image, self.get_parameter('sub_topic_raw_depth').value)
        self.sub_raw_rgb = message_filters.Subscriber(self, Image, self.get_parameter('sub_topic_raw_rgb').value)
        self.sub_silica_depth = message_filters.Subscriber(self, Image, self.get_parameter('sub_topic_silica_depth').value)
        self.sub_silica_mask = message_filters.Subscriber(self, Image, self.get_parameter('sub_topic_silica_mask').value)
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_raw_depth, self.sub_raw_rgb, self.sub_silica_depth, self.sub_silica_mask],
            queue_size=5,
            slop=0.1
        )
        self.ts.registerCallback(self.sync_callback)

        self.get_logger().info("Simple Glass Reconstructor Initialized. Waiting for CameraInfo...")

    def info_callback(self, msg: CameraInfo):
        if self.intrinsics is None:
            self.intrinsics = {
                'fx': msg.k[0],
                'cx': msg.k[2],
                'fy': msg.k[4],
                'cy': msg.k[5]
            }
            self.get_logger().info("Camera intrinsics locked.")

    def sync_callback(self, msg_raw_depth, msg_raw_rgb, msg_silica_depth, msg_silica_mask):
        if self.intrinsics is None:
            return
            
        try:
            # --- 1. Load Images & Handle Data Types ---
            raw_depth_mm = self.bridge.imgmsg_to_cv2(msg_raw_depth, desired_encoding="16UC1")
            raw_rgb = self.bridge.imgmsg_to_cv2(msg_raw_rgb, desired_encoding="rgb8")
            
            silica_depth_raw = self.bridge.imgmsg_to_cv2(msg_silica_depth, desired_encoding="passthrough")
            pred_z = silica_depth_raw.astype(np.float32) if silica_depth_raw.dtype != np.uint16 else silica_depth_raw.astype(np.float32) / 1000.0
                
            silica_mask_raw = self.bridge.imgmsg_to_cv2(msg_silica_mask, desired_encoding="passthrough")
            
            if silica_mask_raw.dtype in [np.float32, np.float64]:
                if np.nanmax(silica_mask_raw) <= 1.01:
                    silica_mask = (silica_mask_raw * 255.0).astype(np.uint8)
                else:
                    silica_mask = silica_mask_raw.astype(np.uint8)
            else:
                silica_mask = silica_mask_raw.astype(np.uint8)
            
            sensor_z = raw_depth_mm.astype(np.float32) / 1000.0
            
            # --- 2. Resize Model Outputs to Match Camera ---
            target_H, target_W = sensor_z.shape[:2]
            
            if silica_mask.shape[:2] != (target_H, target_W):
                silica_mask = cv2.resize(silica_mask, (target_W, target_H), interpolation=cv2.INTER_NEAREST)
                
            if pred_z.shape[:2] != (target_H, target_W):
                pred_z = cv2.resize(pred_z, (target_W, target_H), interpolation=cv2.INTER_NEAREST)
            
            # --- 3. Segmentation Logic ---
            pred_mask = silica_mask > self.threshold
            background_mask = ~pred_mask
            glass_mask = pred_mask
            
            # --- 4. Simple Disparity Least Squares Alignment ---
            mixed_depth = sensor_z.copy()
            valid_mask = (sensor_z > 1e-3) & (pred_z > 1e-3) & background_mask

            gt_disparity, gt_non_neg_mask = depth2disparity(depth=sensor_z, return_mask=True)
            pred_non_neg_mask = pred_z > 0
            valid_nonnegative_mask = valid_mask & gt_non_neg_mask & pred_non_neg_mask

            disparity_pred = align_depth_least_square(
                gt_arr=gt_disparity,
                pred_arr=pred_z,
                valid_mask_arr=valid_nonnegative_mask,
                return_scale_shift=False
            )
            
            disparity_pred = np.clip(disparity_pred, a_min=1e-3, a_max=None)
            pred_depth_aligned = disparity2depth(disparity_pred)

            if pred_depth_aligned.ndim == 3:
                pred_depth_aligned = pred_depth_aligned.squeeze()

            # Splice directly
            mixed_depth[glass_mask] = pred_depth_aligned[glass_mask]

            # --- 5. Overlays ---
            overlay_rgb = raw_rgb.copy()
            red_color = np.array([255, 0, 0], dtype=np.uint8)
            alpha = 0.4
            
            overlay_rgb[glass_mask] = (overlay_rgb[glass_mask] * (1 - alpha) + red_color * alpha).astype(np.uint8)
            
            edge_kernel = np.ones((9, 9), np.uint8)
            border_mask = (cv2.dilate(glass_mask.astype(np.uint8), edge_kernel, iterations=1) > 0) & ~glass_mask
            overlay_rgb[border_mask] = [0, 255, 0]
            
            # --- 6. Generate PointCloud2 ---
            step = self.pc_step
            H, W = mixed_depth.shape
            v_grid, u_grid = np.indices((H, W))
            
            u_dec = u_grid[::step, ::step]
            v_dec = v_grid[::step, ::step]
            z_dec = mixed_depth[::step, ::step]
            rgb_dec = overlay_rgb[::step, ::step]
            
            valid_pc = z_dec > 0.001
            u_pc = u_dec[valid_pc]
            v_pc = v_dec[valid_pc]
            z_pc = z_dec[valid_pc]
            
            # Deproject
            X = (u_pc - self.intrinsics['cx']) * z_pc / self.intrinsics['fx']
            Y = (v_pc - self.intrinsics['cy']) * z_pc / self.intrinsics['fy']
            
            # Pack RGB into FLOAT32
            colors = rgb_dec[valid_pc]
            r = colors[:, 0].astype(np.uint32)
            g = colors[:, 1].astype(np.uint32)
            b = colors[:, 2].astype(np.uint32)
            rgb_packed = (r << 16) | (g << 8) | b
            rgb_float = rgb_packed.view(np.float32)
            
            points = np.column_stack((X, Y, z_pc, rgb_float))
            
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            
            # --- 7. Publish with TF Intact ---
            target_header = msg_raw_depth.header
            
            final_depth_mm = np.clip(mixed_depth * 1000.0, 0, 65535).astype(np.uint16)
            
            out_depth_msg = self.bridge.cv2_to_imgmsg(final_depth_mm, encoding="16UC1")
            out_depth_msg.header = target_header
            
            out_rgb_msg = self.bridge.cv2_to_imgmsg(overlay_rgb, encoding="rgb8")
            out_rgb_msg.header = target_header
            
            out_pc_msg = pc2.create_cloud(target_header, fields, points)
            
            self.pub_depth.publish(out_depth_msg)
            self.pub_rgb.publish(out_rgb_msg)
            self.pub_pc.publish(out_pc_msg)

        except Exception as e:
            self.get_logger().error(f"Failed to process frames: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AlignMetricNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()