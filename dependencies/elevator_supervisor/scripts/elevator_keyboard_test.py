#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Empty, Trigger
from nav2_msgs.srv import LoadMap
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType
from geometry_msgs.msg import PoseWithCovarianceStamped

import sys, os, tty, termios, threading, time, math

# =============================================================================
# CONFIGURE FLOOR MAP PATHS + ANCHOR POSES
#
# anchor_pose: approximate robot pose at the elevator EXIT on each floor.
# Used as a fallback /initialpose when BBS fails (elevator interior is
# feature-poor and BBS often scores below threshold there).
#
# Fill in x/y/yaw from your map — these are the P1/P4 coordinates.
# yaw is in radians (0 = facing +X, π/2 = facing +Y).
# =============================================================================
FLOOR_CONFIGS = {
    1: {
        "label":     "Ground Floor (Floor 1)",
        "pcd_path":  "/home/container_user/wheelchair2/src/wheelchair2_navigation/maps/ground_floor/may_28/lidarnewmap1.pcd",
        "nav2_yaml": "/home/container_user/wheelchair2/src/wheelchair2_navigation/maps/ground_floor/may_28/lidarnewmap2d.yaml",
        "anchor_pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},  # ← set to your P1
    },
    4: {
        "label":     "Floor 4",
        "pcd_path":  "/home/container_user/wheelchair2/src/wheelchair2_navigation/maps/4th_floor/may_28/lidarnewmap.pcd",
        "nav2_yaml": "/home/container_user/wheelchair2/src/wheelchair2_navigation/maps/4th_floor/may_28/lidarnewmap2d.yaml",
        "anchor_pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},  # ← set to your P4
    },
}

ICP_NODE_NAME = "/icp_localization"
BBS_NODE_NAME = "/gpu_ros2_test_rviz2"


class ElevatorTestNode(Node):
    def __init__(self):
        super().__init__("elevator_keyboard_test")

        # ── ICP ──────────────────────────────────────────────────────────────
        self.icp_pause     = self.create_client(SetBool, "/pause_localization")
        self.icp_reset     = self.create_client(Empty,   "/reset_localization")
        self.icp_load      = self.create_client(Trigger, "/load_map")
        self.icp_set_param = self.create_client(SetParameters,
                                f"{ICP_NODE_NAME}/set_parameters")

        # ── BBS ──────────────────────────────────────────────────────────────
        self.bbs_pause     = self.create_client(SetBool, "/bbs/pause")
        self.bbs_reload    = self.create_client(Trigger, "/bbs/reload_map")
        self.bbs_localize  = self.create_client(Trigger, "/bbs/localize")
        self.bbs_set_param = self.create_client(SetParameters,
                                f"{BBS_NODE_NAME}/set_parameters")

        # ── Nav2 ─────────────────────────────────────────────────────────────
        self.map_load      = self.create_client(LoadMap, "/map_server/load_map")
        self.global_clear  = self.create_client(Empty,
                                "/global_costmap/clear_entirely_global_costmap")
        self.local_clear   = self.create_client(Empty,
                                "/local_costmap/clear_entirely_local_costmap")

        # ── /initialpose publisher (fallback anchor pose) ─────────────────────
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)

        self.current_floor  = 1
        self.busy           = False
        self._maps_loaded   = False
        self._pending_floor = None
        self.log("Node ready.")

    # ── Logging ───────────────────────────────────────────────────────────────
    def log(self, msg):
        print(f"\n  [TEST] {msg}", flush=True)

    def log_step(self, step, total, msg):
        print(f"\n  [{step}/{total}] {msg}", flush=True)

    # ── Generic callers ───────────────────────────────────────────────────────
    def call_set_bool(self, client, value, label, timeout=5.0):
        if not client.service_is_ready():
            self.log(f"WARN: '{label}' not ready"); return False
        req = SetBool.Request(); req.data = value
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is not None:
            r = future.result()
            self.log(f"{label} [{'OK' if r.success else 'FAIL'}] → {r.message}")
            return r.success
        self.log(f"ERROR: '{label}' timed out after {timeout}s"); return False

    def call_empty(self, client, label, timeout=5.0):
        if not client.service_is_ready():
            self.log(f"WARN: '{label}' not ready"); return False
        future = client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        self.log(f"{label} → done"); return True

    def call_trigger(self, client, label, timeout=60.0):
        if not client.service_is_ready():
            self.log(f"WARN: '{label}' not ready"); return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is not None:
            r = future.result()
            self.log(f"{label} [{'OK' if r.success else 'FAIL'}] → {r.message}")
            return r.success
        self.log(f"ERROR: '{label}' timed out after {timeout}s"); return False

    def call_load_map(self, yaml_path, label, timeout=10.0):
        if not self.map_load.service_is_ready():
            self.log(f"WARN: '{label}' /map_server/load_map not ready"); return False
        req = LoadMap.Request(); req.map_url = yaml_path
        future = self.map_load.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is not None:
            ok = (future.result().result == 0)
            self.log(f"{label} [{'OK' if ok else 'FAIL'}] code={future.result().result}")
            return ok
        self.log(f"ERROR: '{label}' timed out"); return False

    def set_param_string(self, client, param_name, param_value, label, timeout=5.0):
        if not client.service_is_ready():
            self.log(f"WARN: '{label}' set_parameters not ready"); return False
        req = SetParameters.Request()
        p = Parameter(); p.name = param_name
        p.value.type = ParameterType.PARAMETER_STRING
        p.value.string_value = param_value
        req.parameters = [p]
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is not None:
            self.log(f"{label}: {param_name} = {param_value}"); return True
        self.log(f"ERROR: '{label}' set_parameters timed out"); return False

    # ── Anchor pose fallback ──────────────────────────────────────────────────
    def publish_anchor_pose(self, floor_id):
        """
        Publishes a known anchor pose directly as /initialpose.
        Called when BBS fails to localize (e.g., inside featureless elevator).
        ICP will pick this up via set2DPoseCallback and snap to the new floor.
        """
        ap = FLOOR_CONFIGS[floor_id].get("anchor_pose")
        if ap is None:
            self.log("WARN: No anchor_pose configured for this floor — skipping fallback.")
            return False

        yaw = ap["yaw"]
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id    = "map"
        msg.header.stamp       = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = ap["x"]
        msg.pose.pose.position.y = ap["y"]
        msg.pose.pose.position.z = ap["z"]

        # yaw → quaternion (rotation around Z)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # Generous covariance — ICP will refine quickly
        msg.pose.covariance[0]  = 1.0   # x
        msg.pose.covariance[7]  = 1.0   # y
        msg.pose.covariance[14] = 0.25  # z
        msg.pose.covariance[21] = 0.25  # roll
        msg.pose.covariance[28] = 0.25  # pitch
        msg.pose.covariance[35] = 0.5   # yaw

        self.initialpose_pub.publish(msg)
        self.log(f"Anchor pose published → x={ap['x']:.2f}  y={ap['y']:.2f}  "
                 f"yaw={math.degrees(yaw):.1f}°")
        return True

    # ── Individual ops ────────────────────────────────────────────────────────
    def pause_icp(self):   self.call_set_bool(self.icp_pause, True,  "ICP PAUSE")
    def resume_icp(self):  self.call_set_bool(self.icp_pause, False, "ICP RESUME")
    def reset_icp(self):   self.call_empty(self.icp_reset, "ICP RESET")
    def pause_bbs(self):   self.call_set_bool(self.bbs_pause, True,  "BBS PAUSE")
    def resume_bbs(self):  self.call_set_bool(self.bbs_pause, False, "BBS RESUME")

    def clear_costmaps(self):
        self.log("Clearing Nav2 costmaps...")
        self.call_empty(self.global_clear, "GLOBAL COSTMAP CLEAR")
        self.call_empty(self.local_clear,  "LOCAL COSTMAP CLEAR")

    def reload_nav2_map(self, floor_id):
        yaml_path = FLOOR_CONFIGS[floor_id]["nav2_yaml"]
        if not os.path.exists(yaml_path):
            self.log(f"WARN: nav2_yaml not found: {yaml_path} — skipping"); return False
        self.log(f"Loading Nav2 map: {yaml_path}")
        return self.call_load_map(yaml_path, "MAP SERVER LOAD")

    def reload_bbs_map(self, floor_id):
        """Only rebuilds GPU voxel map. Does NOT localize."""
        pcd_path = FLOOR_CONFIGS[floor_id]["pcd_path"]
        dir_path = os.path.dirname(pcd_path) if pcd_path.endswith(".pcd") else pcd_path
        self.log(f"BBS map directory: {dir_path}")
        self.set_param_string(self.bbs_set_param, "map_path", dir_path, "BBS SET PARAM")
        return self.call_trigger(self.bbs_reload, "BBS RELOAD MAP", timeout=60.0)

    def reload_icp_map(self, floor_id):
        pcd_path = FLOOR_CONFIGS[floor_id]["pcd_path"]
        self.set_param_string(self.icp_set_param, "map_path", pcd_path, "ICP SET PARAM")
        return self.call_trigger(self.icp_load, "ICP LOAD MAP")

    # =========================================================================
    # STAGE 1 — load maps, nodes stay paused
    # =========================================================================
    def switch_to_floor(self, floor_id):
        if self.busy:
            self.log("Busy — wait."); return
        if floor_id not in FLOOR_CONFIGS:
            self.log(f"ERROR: Floor {floor_id} not configured."); return

        self.busy = True
        label = FLOOR_CONFIGS[floor_id]["label"]
        total = 7

        print(f"\n  ╔══════════════════════════════════════╗"
              f"\n  ║  SWITCHING TO: {label:<23}║"
              f"\n  ╚══════════════════════════════════════╝")

        self.log_step(1, total, "Pausing ICP...")
        self.call_set_bool(self.icp_pause, True, "ICP PAUSE")

        self.log_step(2, total, "Pausing BBS...")
        self.call_set_bool(self.bbs_pause, True, "BBS PAUSE")

        self.log_step(3, total, "Reloading BBS GPU voxel map...")
        self.reload_bbs_map(floor_id)

        self.log_step(4, total, "Reloading ICP PCD map...")
        self.reload_icp_map(floor_id)

        self.log_step(5, total, "Resetting ICP state...")
        self.reset_icp()

        self.log_step(6, total, "Switching Nav2 map server...")
        self.reload_nav2_map(floor_id)

        self.log_step(7, total, "Clearing Nav2 costmaps...")
        self.clear_costmaps()

        self._pending_floor = floor_id
        self._maps_loaded   = True
        self.busy           = False

        print(f"""
  ╔══════════════════════════════════════════════════════╗
  ║  MAPS LOADED — ICP + BBS STILL PAUSED               ║
  ║  Check /tar_points in RViz → should show {label[:12]:<12}  ║
  ║                                                      ║
  ║  Press SPACE / d when robot arrives on new floor.   ║
  ║  Sequence:                                           ║
  ║    8. Resume ICP          (listens for /initialpose) ║
  ║    9. /bbs/localize       → tries to publish pose    ║
  ║       fallback anchor     → if BBS fails, Python     ║
  ║                             publishes known pose     ║
  ║   10. Resume BBS                                     ║
  ╚══════════════════════════════════════════════════════╝""", flush=True)

    # =========================================================================
    # STAGE 2 — camera confirmed, exit elevator
    # =========================================================================
    def confirm_floor_and_resume(self):
        """
        Steps 8-10 with anchor pose fallback.

        Critical ordering:
          8. Resume ICP first  → ICP is now listening on /initialpose
          9. /bbs/localize     → if OK, ICP snaps to BBS pose
             anchor fallback   → if BBS failed, Python publishes known pose,
                                 ICP snaps to that instead
         10. Resume BBS
        """
        if self.busy:
            self.log("Busy — wait."); return
        if not self._maps_loaded:
            self.log("No pending floor switch. Run a floor switch first."); return

        self.busy     = True
        floor_id      = self._pending_floor
        label         = FLOOR_CONFIGS[floor_id]["label"]

        self.log(f"Camera confirmed → exiting to {label}...")

        # Step 8: Resume ICP — now listening for /initialpose
        self.log("Step 8/10: Resuming ICP (listening for /initialpose)...")
        self.call_set_bool(self.icp_pause, False, "ICP RESUME")

        # Brief delay — ensures ICP's set2DPoseCallback subscription is active
        time.sleep(0.5)

        # Step 9: Try BBS localization first
        self.log("Step 9/10: Attempting BBS localization...")
        bbs_ok = self.call_trigger(self.bbs_localize, "BBS LOCALIZE", timeout=30.0)

        if bbs_ok:
            self.log("BBS localization succeeded → ICP will snap to /initialpose.")
        else:
            # ── Fallback: publish known anchor pose ───────────────────────
            self.log("BBS localization FAILED (elevator is feature-poor — expected).")
            self.log("Publishing anchor pose fallback → ICP will snap to known exit pose.")
            anchor_ok = self.publish_anchor_pose(floor_id)
            if not anchor_ok:
                self.log("WARN: No anchor_pose set for this floor.")
                self.log("      Edit FLOOR_CONFIGS and fill in x/y/yaw for each floor.")
                self.log("      ICP will try to match from last known pose — may drift.")

            # Give ICP time to process the /initialpose message
            time.sleep(0.5)

        # Step 10: Resume BBS
        self.log("Step 10/10: Resuming BBS (continuous localization)...")
        self.call_set_bool(self.bbs_pause, False, "BBS RESUME")

        self.current_floor  = floor_id
        self._maps_loaded   = False
        self._pending_floor = None
        self.busy           = False

        self.log(f"Floor switch complete → {label}")
        self.log("Watch RViz: map->odom TF + /registered_cloud should align.")

    def run_bbs_localize(self):
        """Manual /bbs/localize retry (key 'L')."""
        self.log("Manually triggering /bbs/localize...")
        ok = self.call_trigger(self.bbs_localize, "BBS LOCALIZE", timeout=30.0)
        if not ok:
            self.log("BBS failed — try 'A' to publish anchor pose instead.")

    def publish_anchor_now(self):
        """Manual anchor pose publish (key 'A') — useful when BBS can't localize."""
        if self._pending_floor is not None:
            floor_id = self._pending_floor
        else:
            floor_id = self.current_floor
        self.log(f"Publishing anchor pose for floor {floor_id}...")
        self.publish_anchor_pose(floor_id)


# =============================================================================
# Menu
# =============================================================================
def print_menu(current_floor, maps_loaded):
    floor_keys = "\n".join(
        f"  ║    {fid}  →  Switch to {cfg['label']:<36}║"
        for fid, cfg in FLOOR_CONFIGS.items()
    )
    state = " [MAPS LOADED — awaiting SPACE]" if maps_loaded else ""
    print(f"""
  ╔══════════════════════════════════════════════════════╗
  ║       ELEVATOR LOCALIZATION TEST CONSOLE             ║
  ║       Floor: {str(current_floor) + state:<39}║
  ╠══════════════════════════════════════════════════════╣
  ║  FLOOR SWITCH  (steps 1-7, then pause)               ║
{floor_keys}
  ╠══════════════════════════════════════════════════════╣
  ║  CAMERA / EXIT SIMULATION                            ║
  ║  SPACE / d  →  Confirm arrival → resume + localize   ║
  ╠══════════════════════════════════════════════════════╣
  ║  INDIVIDUAL OPERATIONS                               ║
  ║    p  →  Pause  ICP            P  →  Pause  BBS      ║
  ║    r  →  Resume ICP            R  →  Resume BBS      ║
  ║    e  →  Reset  ICP state      L  →  /bbs/localize   ║
  ║    A  →  Publish anchor pose (fallback /initialpose) ║
  ║    n  →  Reload Nav2 map server (current floor)      ║
  ║    c  →  Clear  Nav2 costmaps                        ║
  ╠══════════════════════════════════════════════════════╣
  ║    m  →  Show this menu        q  →  Quit            ║
  ╚══════════════════════════════════════════════════════╝
  Press a key (no Enter needed): """, end="", flush=True)


def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rclpy.init()
    node = ElevatorTestNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    time.sleep(1.0)
    print_menu(node.current_floor, node._maps_loaded)

    try:
        while True:
            key = get_key()

            if key.isdigit() and int(key) in FLOOR_CONFIGS:
                floor_id = int(key)
                if floor_id == node.current_floor:
                    print(f"\n  Already on floor {floor_id}.")
                elif node.busy:
                    print("\n  Busy — wait.")
                else:
                    threading.Thread(target=node.switch_to_floor,
                                     args=(floor_id,), daemon=True).start()
            elif key in (' ', 'd'):
                threading.Thread(target=node.confirm_floor_and_resume,
                                 daemon=True).start()
            elif key == 'p': threading.Thread(target=node.pause_icp,  daemon=True).start()
            elif key == 'r': threading.Thread(target=node.resume_icp, daemon=True).start()
            elif key == 'e': threading.Thread(target=node.reset_icp,  daemon=True).start()
            elif key == 'P': threading.Thread(target=node.pause_bbs,  daemon=True).start()
            elif key == 'R': threading.Thread(target=node.resume_bbs, daemon=True).start()
            elif key == 'L': threading.Thread(target=node.run_bbs_localize, daemon=True).start()
            elif key == 'A': threading.Thread(target=node.publish_anchor_now, daemon=True).start()
            elif key == 'n':
                threading.Thread(target=node.reload_nav2_map,
                                 args=(node.current_floor,), daemon=True).start()
            elif key == 'c': threading.Thread(target=node.clear_costmaps, daemon=True).start()
            elif key == 'm': print_menu(node.current_floor, node._maps_loaded)
            elif key in ('q', '\x03'): print("\n\n  Quitting.\n"); break
            else: print(f"\n  Unknown key '{repr(key)}'. Press 'm' for menu.")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()