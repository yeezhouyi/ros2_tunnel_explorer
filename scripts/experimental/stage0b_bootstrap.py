#!/usr/bin/env python3
"""
Stage 0B bootstrap monitor.

Waits for /slam_toolbox, activates it, then starts a Nav2 lifecycle
manager to bring up the navigation stack.  Designed to compensate for
unreliable SLAM lifecycle events in nested WSL2 launches.

Usage:
    python3 stage0b_bootstrap.py

Environment:
    Sources ROS2 Jazzy + workspace setup.bash before running.
"""

import subprocess
import sys
import time


def run(cmd, timeout=15):
    """Run a shell command and return (returncode, stdout+stderr)."""
    try:
        r = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=timeout
        )
        return r.returncode, r.stdout + r.stderr
    except subprocess.TimeoutExpired as e:
        return -1, str(e)


def wait_for_slam(max_wait=90):
    """Poll for /slam_toolbox to appear. Returns state string or None."""
    for i in range(max_wait // 3):
        rc, out = run('ros2 lifecycle get /slam_toolbox', timeout=5)
        if 'active' in out:
            return 'active'
        if 'unconfigured' in out or 'inactive' in out:
            return 'inactive'  # actually unconfigured or inactive
        sys.stdout.write(f'  [{i * 3}s] waiting for slam_toolbox ...\n')
        sys.stdout.flush()
        time.sleep(3)
    return None


def activate_slam():
    """Configure and activate /slam_toolbox."""
    print('  Configuring slam_toolbox...')
    rc, out = run('ros2 lifecycle set /slam_toolbox configure', timeout=15)
    print(f'    configure -> rc={rc}: {out[:80]}')
    time.sleep(2)

    print('  Activating slam_toolbox...')
    rc, out = run('ros2 lifecycle set /slam_toolbox activate', timeout=15)
    print(f'    activate -> rc={rc}: {out[:80]}')
    time.sleep(2)

    rc, out = run('ros2 lifecycle get /slam_toolbox', timeout=5)
    print(f'  State: {out.strip()}')
    return 'active' in out


def start_nav2_lifecycle():
    """Start Nav2 lifecycle manager to activate the navigation stack."""
    print('  Starting lifecycle_manager_navigation...')
    run('pkill -f "lifecycle_manager.*navigation" 2>/dev/null', timeout=3)
    time.sleep(1)
    cmd = (
        'ros2 run nav2_lifecycle_manager lifecycle_manager '
        '--ros-args -r __node:=lifecycle_manager_navigation '
        '-p autostart:=true '
        '-p "node_names:=[\'controller_server\',\'smoother_server\',\'planner_server\','
        '\'route_server\',\'behavior_server\',\'bt_navigator\',\'waypoint_follower\','
        '\'velocity_smoother\',\'collision_monitor\',\'docking_server\']" '
        '-p use_sim_time:=true'
    )
    proc = subprocess.Popen(cmd, shell=True)
    print(f'  Started lifecycle manager (PID {proc.pid})')
    time.sleep(10)
    return proc


def check_nav2():
    """Check Nav2 action server availability."""
    rc, out = run('ros2 action info /navigate_to_pose', timeout=8)
    servers = out.count('Action servers:')
    print(f'  /navigate_to_pose: {out.strip()[:100]}')
    return 'Action servers:' in out and '0' not in out.split('Action servers:')[1][:2]


def main():
    print('=== Stage 0B bootstrap ===')

    # Step 1: wait for SLAM
    print('[1/4] Waiting for slam_toolbox node...')
    state = wait_for_slam()
    if state is None:
        print('ERROR: slam_toolbox did not appear within 90s')
        sys.exit(1)
    print(f'  Found slam_toolbox (state={state})')

    # Step 2: activate SLAM if needed
    print('[2/4] Activating SLAM...')
    if state == 'active':
        print('  Already active')
    else:
        ok = activate_slam()
        if not ok:
            print('WARNING: SLAM activation may have failed, continuing...')

    # Step 3: start Nav2 lifecycle manager
    print('[3/4] Starting Nav2 lifecycle manager...')
    proc = start_nav2_lifecycle()

    # Step 4: verify
    print('[4/4] Verifying Nav2 action server...')
    time.sleep(5)
    nav2_ok = check_nav2()
    if nav2_ok:
        print('SUCCESS: /navigate_to_pose is available')
    else:
        print('WARNING: /navigate_to_pose may not be ready')

    print('=== Bootstrap complete ===')
    print('Leave this process running to keep the lifecycle manager alive.')

    try:
        while True:
            time.sleep(60)
            rc, out = run('ros2 action info /navigate_to_pose', timeout=5)
            if 'Action servers: 0' in out or 'Action servers: 1' not in out:
                print('WARNING: /navigate_to_pose lost, re-running lifecycle manager...')
                proc = start_nav2_lifecycle()
    except KeyboardInterrupt:
        print('Shutting down...')
        proc.terminate()


if __name__ == '__main__':
    main()
