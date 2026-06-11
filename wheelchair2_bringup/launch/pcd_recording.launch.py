#!/usr/bin/env python3
"""
PCD Map Recording Launch File
==============================
Plays a rosbag through a crop-lidar filter into FastLIO, then saves the PCD map.

Required:
  bag_dir        - recording folder name OR full path to the rosbag directory
  map_save_path  - full path (with filename) for the output PCD (must end with .pcd)

Usage:
  # Using recording folder name (resolved under RECORDS_ROOT):
  ros2 launch wheelchair2_bringup pcd_recording.launch.py \\
      bag_dir:=2026-06-02_11-35-40 \\
      map_save_path:=/path/to/output/map.pcd

  # Using full path:
  ros2 launch wheelchair2_bringup pcd_recording.launch.py \\
      bag_dir:=/full/path/to/rosbag \\
      map_save_path:=/path/to/output/map.pcd

  # With time limit:
  ros2 launch wheelchair2_bringup pcd_recording.launch.py \\
      bag_dir:=2026-06-02_11-35-40 \\
      map_save_path:=/path/to/output/map.pcd \\
      max_time:=120

Manual stop and save (from a second terminal):
  pkill -INT -f 'ros2 bag play'
"""

import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


# =============================================================================
# REQUIRED  –  set these or pass via CLI (CLI overrides these)
# =============================================================================

# Recording folder name (e.g. '2026-06-02_11-35-40') OR full path to rosbag dir
BAG_DIR = ''

# Full path (including filename) for the output PCD map
MAP_SAVE_PATH = ''

# =============================================================================
# OPTIONAL
# =============================================================================

# Root directory containing recording folders (used when bag_dir is just a name)
RECORDS_ROOT = '/home/container_user/wheelchair2/src/records'

# Max playback time in seconds. 0 = play entire bag.
MAX_TIME = 0

# FastLIO config file name (in fast_lio/config/)
FASTLIO_CONFIG_FILE = 'mid360.yaml'

# Service to trigger map saving
SAVE_SERVICE_NAME = '/map_save'
SAVE_SERVICE_TYPE = 'std_srvs/srv/Trigger'

# Stagger delays (seconds)
STARTUP_DELAY = 2.0
FASTLIO_DELAY = 3.0


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def resolve_bag_dir(bag_dir_arg: str, records_root: str):
    """
    Resolve bag directory from a full path or a recording folder name.
    Returns the resolved path string, or None if not found.
    """
    bag_path = Path(bag_dir_arg)

    # Full path provided
    if bag_path.is_absolute() and bag_path.is_dir():
        return str(bag_path)

    # Try as a recording name: RECORDS_ROOT/<name>/rosbag
    candidate = Path(records_root) / bag_dir_arg / 'rosbag'
    if candidate.is_dir():
        return str(candidate)

    # Try as a recording name without /rosbag suffix
    candidate = Path(records_root) / bag_dir_arg
    if candidate.is_dir():
        return str(candidate)

    return None


# ---------------------------------------------------------------------------
# Launch setup
# ---------------------------------------------------------------------------
def launch_setup(context, *args, **kwargs):
    # Flag to track if the shutdown was manually triggered via Ctrl+C keyboard listener
    ctrl_c_pressed = False

    # -- Resolve arguments --------------------------------------------------
    bag_dir_arg    = LaunchConfiguration('bag_dir').perform(context)
    map_save_path  = LaunchConfiguration('map_save_path').perform(context)
    max_time       = float(LaunchConfiguration('max_time').perform(context))
    use_sim_time_s = LaunchConfiguration('use_sim_time').perform(context)
    config_file    = LaunchConfiguration('config_file').perform(context)
    rviz_arg       = LaunchConfiguration('rviz').perform(context)
    save_svc_name  = LaunchConfiguration('save_service_name').perform(context)
    save_svc_type  = LaunchConfiguration('save_service_type').perform(context)
    startup_delay  = float(LaunchConfiguration('startup_delay').perform(context))
    fastlio_delay  = float(LaunchConfiguration('fastlio_delay').perform(context))
    records_root   = LaunchConfiguration('records_root').perform(context)

    # -- Validate required arguments ----------------------------------------
    errors = []

    if not bag_dir_arg:
        errors.append('  bag_dir        : not set')
    if not map_save_path:
        errors.append('  map_save_path  : not set')
    elif not map_save_path.lower().endswith('.pcd'):
        errors.append('  map_save_path  : path does not end with ".pcd" extension!')

    if errors:
        print('')
        print('[pcd_recording] ERROR: required arguments missing or invalid:')
        for e in errors:
            print(f'[pcd_recording] {e}')
        print('')
        print('[pcd_recording] Set them in the launch file or pass via CLI:')
        print('[pcd_recording]   ros2 launch wheelchair2_bringup pcd_recording.launch.py \\')
        print('[pcd_recording]       bag_dir:=2026-06-02_11-35-40 \\')
        print('[pcd_recording]       map_save_path:=/path/to/output/map.pcd')
        print('')
        sys.exit(1)

    # -- Resolve bag directory ----------------------------------------------
    selected_bag_dir = resolve_bag_dir(bag_dir_arg, records_root)

    if not selected_bag_dir:
        print('')
        print(f'[pcd_recording] ERROR: bag directory not found: {bag_dir_arg}')
        print(f'[pcd_recording]  Checked as full path : {bag_dir_arg}')
        print(f'[pcd_recording]  Checked under root   : {records_root}/{bag_dir_arg}/rosbag')
        print('')
        sys.exit(1)

    # -- Package paths ------------------------------------------------------
    fast_lio_share      = Path(get_package_share_directory('fast_lio'))
    mapping_launch_path = fast_lio_share / 'launch' / 'mapping.launch.py'

    # -- Configuration summary ----------------------------------------------
    print('')
    print('[pcd_recording] ---- Configuration ----')
    print(f'[pcd_recording]  bag_dir        : {selected_bag_dir}')
    print(f'[pcd_recording]  map_save_path  : {map_save_path}')
    print(f'[pcd_recording]  max_time       : {"full bag" if max_time <= 0 else f"{int(max_time)}s"}')
    print(f'[pcd_recording]  config_file    : {config_file}')
    print(f'[pcd_recording]  use_sim_time   : {use_sim_time_s}')
    print(f'[pcd_recording]  save_service   : {save_svc_name} [{save_svc_type}]')
    print(f'[pcd_recording]  startup_delay  : {startup_delay}s')
    print(f'[pcd_recording]  fastlio_delay  : {fastlio_delay}s')
    print('[pcd_recording] -------------------------')
    print('')

    # ======================================================================
    # 1. LIDAR CROP NODE  (starts immediately)
    # ======================================================================
    crop_lidar_node = Node(
        package='wheelchair2_bringup',
        executable='crop_lidar.py',
        name='lidar_crop_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time_s.lower() == 'true'}],
    )

    # ======================================================================
    # 2. FAST-LIO  (via mapping.launch.py with parameter overrides)
    # ======================================================================
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(mapping_launch_path)),
        launch_arguments={
            'use_sim_time': use_sim_time_s,
            'rviz': rviz_arg,
            'config_file': config_file,
            'lid_topic': '/livox/lidar_cropped',
            'dense_publish_en': 'true',
            'map_file_path': map_save_path,
        }.items(),
    )

    # ======================================================================
    # 3. ROSBAG PLAY
    # ======================================================================
    rosbag_cmd = ['ros2', 'bag', 'play', selected_bag_dir]
    if use_sim_time_s.lower() == 'true':
        rosbag_cmd.append('--clock')
    if max_time > 0:
        rosbag_cmd = ['timeout', '--signal=INT', str(int(max_time))] + rosbag_cmd

    rosbag_play = ExecuteProcess(
        cmd=rosbag_cmd,
        output='screen',
        name='rosbag_play',
    )

    # ======================================================================
    # 4. ELAPSED TIMER
    # ======================================================================
    max_info = f'/{int(max_time)}s' if max_time > 0 else ''
    timer_script = (
        'elapsed_timer_process_marker=1; '
        'trap "exit 0" INT TERM; '
        'echo "[pcd_recording] Playback started. Note the time if you see something unwanted."; '
        'echo "[pcd_recording] Re-run with max_time:=<seconds> to trim there."; '
        'start=$SECONDS; '
        'while true; do '
        '  elapsed=$((SECONDS - start)); '
        '  mins=$((elapsed / 60)); '
        '  secs=$((elapsed % 60)); '
        '  printf "[pcd_recording] [elapsed] %02d:%02d (%ds' + max_info + ')\\n" "$mins" "$secs" "$elapsed"; '
        '  sleep 1; '
        'done'
    )
    elapsed_timer = ExecuteProcess(
        cmd=['bash', '-c', timer_script],
        output='screen',
        name='elapsed_timer',
    )

    # ======================================================================
    # 5. On rosbag exit -> call save service (if not manually stopped via Ctrl+C)
    # ======================================================================
    save_map_action = ExecuteProcess(
        cmd=['ros2', 'service', 'call', save_svc_name, save_svc_type, '{}'],
        output='screen',
        name='save_map_service',
    )

    def on_rosbag_exit_cb(event, context):
        nonlocal ctrl_c_pressed
        if ctrl_c_pressed:
            # Bypassed because keyboard listener handles the manual saving/exit
            return []
        return [
            LogInfo(msg='[pcd_recording] Rosbag finished naturally. Calling ' + save_svc_name + ' ...'),
            save_map_action,
        ]

    on_rosbag_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=rosbag_play,
            on_exit=on_rosbag_exit_cb,
        )
    )

    # ======================================================================
    # 6. On save complete -> shutdown
    # ======================================================================
    on_save_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=save_map_action,
            on_exit=[
                LogInfo(msg='[pcd_recording] Map saved to: ' + map_save_path),
                LogInfo(msg='[pcd_recording] Shutting down.'),
                Shutdown(reason='PCD map saved successfully.'),
            ],
        )
    )

    # -- Staggered launch --------------------------------------------------
    total_rosbag_delay = startup_delay + fastlio_delay

    delayed_fastlio = TimerAction(
        period=startup_delay,
        actions=[
            LogInfo(msg=f'[pcd_recording] Starting FastLIO (after {startup_delay}s delay)...'),
            fast_lio_launch,
        ],
    )

    delayed_rosbag = TimerAction(
        period=total_rosbag_delay,
        actions=[
            LogInfo(msg=f'[pcd_recording] Starting rosbag playback (after {total_rosbag_delay}s total delay)...'),
            rosbag_play,
            elapsed_timer,
        ],
    )

    # -- Start keyboard listener --------------------------------------------
    def start_keyboard_listener():
        import threading
        import termios
        import tty
        import select
        import subprocess
        import time
        import atexit
        import os
        import signal

        fd = sys.stdin.fileno()
        try:
            old_settings = termios.tcgetattr(fd)
        except Exception:
            # stdin is not a TTY (e.g. running in a non-interactive environment)
            return

        # Register atexit handler to ensure terminal is restored when python exits
        atexit.register(termios.tcsetattr, fd, termios.TCSADRAIN, old_settings)

        def run_listener():
            nonlocal ctrl_c_pressed
            try:
                # Set terminal to cbreak mode
                tty.setcbreak(fd)
                # Disable flow control (IXON) and signal handling (ISIG) so Ctrl+C is passed to Python
                current_settings = termios.tcgetattr(fd)
                current_settings[0] &= ~termios.IXON
                current_settings[3] &= ~termios.ISIG
                termios.tcsetattr(fd, termios.TCSADRAIN, current_settings)

                print("\n[pcd_recording] ==========================================================")
                print("[pcd_recording] KEYBOARD LISTENER ACTIVE:")
                print("[pcd_recording] Press [Ctrl+C] at any time to stop playback.")
                print("[pcd_recording] ==========================================================")

                while True:
                    rlist, _, _ = select.select([sys.stdin], [], [], 0.2)
                    if rlist:
                        key = sys.stdin.read(1)
                        # \x03 is Ctrl+C
                        if key == '\x03':
                            # Mark that we stopped via Ctrl+C to bypass automatic exit handler
                            ctrl_c_pressed = True
                            
                            # Immediately terminate the rosbag playback process and the elapsed timer
                            subprocess.run(['pkill', '-INT', '-f', 'ros2 bag play'])
                            subprocess.run(['pkill', '-INT', '-f', 'elapsed_timer_process_marker'])

                            sys.stdout.write(f"\n[pcd_recording] Do you want to save the map to {map_save_path}? [y/n]: ")
                            sys.stdout.flush()

                            ans = ''
                            while True:
                                rlist2, _, _ = select.select([sys.stdin], [], [], 0.1)
                                if rlist2:
                                    ans = sys.stdin.read(1).lower()
                                    if ans in ['y', 'n'] or ans == '\x03':
                                        break
                                time.sleep(0.05)

                            if ans == 'y':
                                sys.stdout.write("y\n[pcd_recording] Saving map...\n")
                                sys.stdout.flush()
                                # Call the save service manually while FAST-LIO remains active
                                subprocess.run(['ros2', 'service', 'call', save_svc_name, save_svc_type, '{}'])
                                sys.stdout.write("[pcd_recording] Map saved. Exiting...\n")
                                sys.stdout.flush()
                                # Restore terminal and send SIGINT to self to trigger shutdown
                                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                                os.kill(os.getpid(), signal.SIGINT)
                                break
                            else:
                                if ans == '\x03':
                                    sys.stdout.write("Ctrl+C\n")
                                else:
                                    sys.stdout.write("n\n")
                                sys.stdout.write("[pcd_recording] Exiting immediately without saving...\n")
                                sys.stdout.flush()
                                # Restore terminal and send SIGINT to self to trigger immediate shutdown
                                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                                os.kill(os.getpid(), signal.SIGINT)
                                break
                    time.sleep(0.05)
            except Exception as e:
                print(f"\n[pcd_recording] Keyboard listener error: {e}")
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        t = threading.Thread(target=run_listener, daemon=True)
        t.start()

    start_keyboard_listener()

    return [
        crop_lidar_node,
        delayed_fastlio,
        delayed_rosbag,
        on_rosbag_exit,
        on_save_exit,
    ]


# ---------------------------------------------------------------------------
# Launch description
# ---------------------------------------------------------------------------
def generate_launch_description():
    return LaunchDescription([
        # -- Required -------------------------------------------------------
        DeclareLaunchArgument(
            'bag_dir', default_value=BAG_DIR,
            description='[REQUIRED] Recording folder name or full path to rosbag dir',
        ),
        DeclareLaunchArgument(
            'map_save_path', default_value=MAP_SAVE_PATH,
            description='[REQUIRED] Full path (with filename) for the output PCD map',
        ),
        # -- Optional -------------------------------------------------------
        DeclareLaunchArgument(
            'records_root', default_value=RECORDS_ROOT,
            description='Root dir for recordings (used when bag_dir is a folder name)',
        ),
        DeclareLaunchArgument(
            'max_time', default_value=str(MAX_TIME),
            description='Max playback duration in seconds (0 = play entire bag)',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Publish /clock from bag and use sim time',
        ),
        DeclareLaunchArgument(
            'config_file', default_value=FASTLIO_CONFIG_FILE,
            description='FastLIO YAML config file name',
        ),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz2',
        ),
        DeclareLaunchArgument(
            'save_service_name', default_value=SAVE_SERVICE_NAME,
            description='Service name to save the PCD map',
        ),
        DeclareLaunchArgument(
            'save_service_type', default_value=SAVE_SERVICE_TYPE,
            description='Service type for map save',
        ),
        DeclareLaunchArgument(
            'startup_delay', default_value=str(STARTUP_DELAY),
            description='Seconds before starting FastLIO',
        ),
        DeclareLaunchArgument(
            'fastlio_delay', default_value=str(FASTLIO_DELAY),
            description='Seconds after FastLIO before rosbag playback',
        ),
        # -------------------------------------------------------------------
        OpaqueFunction(function=launch_setup),
    ])
