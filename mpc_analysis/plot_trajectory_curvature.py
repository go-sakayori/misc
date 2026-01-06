#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import os
from matplotlib.collections import LineCollection
from scipy import interpolate
from scipy.interpolate import CubicSpline

def resample_trajectory_by_distance(x, y, yaw, resample_interval=0.1, ego_x=None, ego_y=None):
    """Resample trajectory by distance using cubic spline interpolation (same as MPC)

    MPC uses autoware::interpolation::spline which implements natural cubic spline
    (boundary condition: second derivative = 0 at endpoints)

    MPC resamples from ego position in both forward and backward directions to ensure
    ego point is accurately sampled.

    For n == 2: uses linear interpolation (y = c*dx + d)
    For n >= 3: uses natural cubic spline

    Args:
        x, y, yaw: trajectory points
        resample_interval: resampling interval in meters
        ego_x, ego_y: ego vehicle position (optional, for MPC-like resampling from ego position)

    Returns:
        tuple: (x_resampled, y_resampled, yaw_resampled, arc_length_resampled)

    Raises:
        ValueError: if len(x) < 2 (same as Autoware's validateKeysAndValues)
    """
    # Validate input (same as Autoware)
    if len(x) < 2:
        raise ValueError(f"Points size is less than 2: size = {len(x)}")

    if len(x) != len(y) or len(x) != len(yaw):
        raise ValueError(f"Points size mismatch: x={len(x)}, y={len(y)}, yaw={len(yaw)}")

    # Calculate arc length
    arc_length = [0.0]
    for i in range(1, len(x)):
        dist = np.sqrt((x[i] - x[i-1])**2 + (y[i] - y[i-1])**2)
        arc_length.append(arc_length[-1] + dist)

    # Create output arc length array (same as MPC)
    # MPC resamples from ego position in forward and backward directions
    if ego_x is not None and ego_y is not None:
        # Find nearest segment to ego position
        min_dist = float('inf')
        nearest_seg_idx = 0
        for i in range(len(x) - 1):
            # Calculate distance from ego to segment
            dist = np.sqrt((x[i] - ego_x)**2 + (y[i] - ego_y)**2)
            if dist < min_dist:
                min_dist = dist
                nearest_seg_idx = i

        # Calculate ego offset to segment
        # Project ego position onto the segment
        if nearest_seg_idx < len(x) - 1:
            # Vector from segment start to ego
            dx_ego = ego_x - x[nearest_seg_idx]
            dy_ego = ego_y - y[nearest_seg_idx]
            # Vector along segment
            dx_seg = x[nearest_seg_idx + 1] - x[nearest_seg_idx]
            dy_seg = y[nearest_seg_idx + 1] - y[nearest_seg_idx]
            seg_length = np.sqrt(dx_seg**2 + dy_seg**2)

            if seg_length > 1e-6:
                # Project ego onto segment
                projection = (dx_ego * dx_seg + dy_ego * dy_seg) / seg_length
                ego_offset_to_segment = np.clip(projection, 0, seg_length)
            else:
                ego_offset_to_segment = 0.0
        else:
            ego_offset_to_segment = 0.0

        # Ego position in arc length coordinate
        ego_arc_length = arc_length[nearest_seg_idx] + ego_offset_to_segment
        ego_arc_length = np.clip(ego_arc_length, 0.0, arc_length[-1] - 1e-6)

        # Resample backward from ego position
        output_arc_length = []
        s = ego_arc_length
        while s >= 0:
            output_arc_length.append(s)
            s -= resample_interval
        output_arc_length.reverse()

        # Resample forward from ego position
        s = ego_arc_length + resample_interval
        while s < arc_length[-1]:
            output_arc_length.append(s)
            s += resample_interval

        output_arc_length = np.array(output_arc_length)
    else:
        # Simple resampling from start to end (no ego position)
        output_arc_length = np.arange(0, arc_length[-1], resample_interval)

    if len(output_arc_length) == 0:
        # If trajectory is too short, return original
        return x, y, yaw, arc_length

    # Autoware's spline implementation:
    # - n == 2: linear interpolation (a=0, b=0, c=(y1-y0)/(x1-x0), d=y0)
    # - n >= 3: natural cubic spline (second derivative = 0 at boundaries)
    if len(x) == 2:
        # Linear interpolation (same as Autoware for n==2)
        x_resampled = np.interp(output_arc_length, arc_length, x)
        y_resampled = np.interp(output_arc_length, arc_length, y)
        yaw_resampled = np.interp(output_arc_length, arc_length, yaw)
    else:
        # Natural cubic spline (n >= 3)
        # bc_type='natural' means second derivative = 0 at boundaries
        x_spline = CubicSpline(arc_length, x, bc_type='natural', extrapolate=True)
        y_spline = CubicSpline(arc_length, y, bc_type='natural', extrapolate=True)
        yaw_spline = CubicSpline(arc_length, yaw, bc_type='natural', extrapolate=True)

        x_resampled = x_spline(output_arc_length)
        y_resampled = y_spline(output_arc_length)
        yaw_resampled = yaw_spline(output_arc_length)

    return x_resampled, y_resampled, yaw_resampled, output_arc_length

def normalize_radian(angle):
    """Normalize angle to [-pi, pi] range (same as MPC)"""
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle

def convert_euler_angle_to_monotonic(angle_vector):
    """Convert Euler angle vector to monotonic (same as MPC)

    This ensures angle continuity across -pi/pi boundary by unwrapping angles.
    """
    if len(angle_vector) < 2:
        return angle_vector

    result = np.copy(angle_vector)
    for i in range(1, len(angle_vector)):
        da = result[i] - result[i-1]
        result[i] = result[i-1] + normalize_radian(da)

    return result

def calc_trajectory_yaw_from_xy(x, y, is_forward_shift=True):
    """Calculate trajectory yaw from XY coordinates (same as MPC)

    Args:
        x, y: trajectory coordinates (can be numpy arrays or lists)
        is_forward_shift: True for forward driving, False for backward

    Returns:
        yaw array (modifies in-place if x, y are numpy arrays)
    """
    x_arr = np.asarray(x)
    y_arr = np.asarray(y)

    if len(x_arr) < 3:
        return np.zeros(len(x_arr))

    yaw = np.zeros(len(x_arr))

    # Calculate yaw for middle points using central difference
    for i in range(1, len(x_arr) - 1):
        dx = x_arr[i+1] - x_arr[i-1]
        dy = y_arr[i+1] - y_arr[i-1]
        yaw[i] = np.arctan2(dy, dx) if is_forward_shift else np.arctan2(dy, dx) + np.pi

    # Copy boundary values
    if len(x_arr) > 1:
        yaw[0] = yaw[1]
        yaw[-1] = yaw[-2]

    return yaw

def calc_mpc_trajectory_time(x, y, vx):
    """Calculate relative time for trajectory (same as MPC)

    Args:
        x, y: trajectory coordinates
        vx: velocity at each point

    Returns:
        relative_time array
    """
    min_dt = 1.0e-4  # must be positive value to avoid duplication in time
    t = 0.0
    relative_time = [t]

    for i in range(len(x) - 1):
        dist = np.sqrt((x[i+1] - x[i])**2 + (y[i+1] - y[i])**2)
        v = max(abs(vx[i]), 0.1)
        t += max(dist / v, min_dt)
        relative_time.append(t)

    return np.array(relative_time)

def extend_trajectory_in_yaw_direction(x, y, yaw, vx, k, smooth_k, relative_time,
                                       terminal_yaw, interval, is_forward_shift=True):
    """Extend trajectory in yaw direction (same as MPC)

    This extends the trajectory by adding points in the direction of terminal_yaw.
    Used to improve terminal yaw control performance.

    Args:
        x, y, yaw, vx, k, smooth_k, relative_time: trajectory data (will be modified in-place)
        terminal_yaw: desired yaw angle at trajectory end
        interval: distance interval for extension
        is_forward_shift: True for forward driving, False for backward

    Returns:
        None (modifies arrays in-place)
    """
    if len(x) == 0:
        return

    # Set terminal yaw
    yaw[-1] = terminal_yaw

    # Extension parameters (same as MPC)
    extend_dist = 10.0  # extend 10 meters
    extend_vel = vx[-1]
    x_offset = interval if is_forward_shift else -interval
    min_vel_threshold = 0.1
    dt = 1.0e-4 if abs(extend_vel) < min_vel_threshold else interval / abs(extend_vel)
    num_extended_point = int(extend_dist / interval)

    # Current terminal position and yaw
    current_x = x[-1]
    current_y = y[-1]
    current_yaw = yaw[-1]

    # Extend trajectory by adding points along the terminal yaw direction
    for i in range(num_extended_point):
        # Calculate next position along yaw direction
        # This mimics autoware_utils::calc_offset_pose(pose, x_offset, 0.0, 0.0)
        current_x += x_offset * np.cos(current_yaw)
        current_y += x_offset * np.sin(current_yaw)

        # Append new point (keeping terminal yaw, velocity, and curvature)
        x = np.append(x, current_x)
        y = np.append(y, current_y)
        yaw = np.append(yaw, current_yaw)
        vx = np.append(vx, extend_vel)
        k = np.append(k, k[-1])
        smooth_k = np.append(smooth_k, smooth_k[-1])
        relative_time = np.append(relative_time, relative_time[-1] + dt)

    return x, y, yaw, vx, k, smooth_k, relative_time

def moving_average_filter(data, window_size):
    """Apply moving average filter (same as MPC path smoothing)

    Args:
        data: input array
        window_size: window size for moving average

    Returns:
        filtered array
    """
    if len(data) < 2 * window_size:
        return data

    filtered = np.copy(data)
    for i in range(window_size, len(data) - window_size):
        filtered[i] = np.mean(data[i - window_size : i + window_size + 1])

    return filtered

def calculate_curvature_from_points(x, y, smoothing_num=5):
    """
    Calculate curvature using 3-point circle fitting (same as MPC)

    Args:
        x, y: trajectory points
        smoothing_num: number of points to skip for smoothing (L parameter)

    Returns:
        curvature array
    """
    curvature = np.zeros(len(x))
    # MPC uses: floor(0.5 * (size - 1)), not size // 2
    max_smoothing_num = int(np.floor(0.5 * (len(x) - 1)))
    L = min(smoothing_num, max_smoothing_num)

    for i in range(L, len(x) - L):
        # Use 3 points: i-L, i, i+L
        p1_x, p1_y = x[i-L], y[i-L]
        p2_x, p2_y = x[i], y[i]
        p3_x, p3_y = x[i+L], y[i+L]

        # Calculate curvature using circle fitting formula
        a = np.sqrt((p2_x - p1_x)**2 + (p2_y - p1_y)**2)
        b = np.sqrt((p3_x - p2_x)**2 + (p3_y - p2_y)**2)
        c = np.sqrt((p1_x - p3_x)**2 + (p1_y - p3_y)**2)

        # Area using cross product
        area = 0.5 * abs((p2_x - p1_x) * (p3_y - p1_y) - (p3_x - p1_x) * (p2_y - p1_y))

        if a * b * c > 1e-10:
            curvature[i] = 4.0 * area / (a * b * c)

            # Determine sign based on cross product
            cross = (p2_x - p1_x) * (p3_y - p2_y) - (p2_y - p1_y) * (p3_x - p2_x)
            if cross < 0:
                curvature[i] = -curvature[i]
        else:
            curvature[i] = 0.0

    # Fill first and last L points (same as MPC)
    for i in range(min(L, len(x))):
        curvature[i] = curvature[min(L, len(x) - 1)]
        curvature[len(x) - i - 1] = curvature[max(len(x) - L - 1, 0)]

    return curvature

def plot_trajectory_curvature(extracted_data):
    """Plot trajectory shape and curvature relationship for all trajectories in time window

    Args:
        extracted_data: dictionary containing pre-extracted data:
            - ego_positions: list of ego positions
            - diag_curvature_data: list of diagnostic curvature data
            - resampled_trajectories: list of resampled trajectories
            - planning_trajectories: list of planning trajectories
    """

    ego_positions = extracted_data['ego_positions']
    diag_curvature_data = extracted_data['diag_curvature_data']
    all_trajectories = extracted_data['resampled_trajectories']
    all_planning_trajectories = extracted_data['planning_trajectories']

    if len(all_trajectories) == 0:
        print("Warning: No trajectory data found for visualization")
        return

    print(f"\n{'Trajectory Visualization':-^80}")
    print(f"  Total trajectories in time window: {len(all_trajectories)}")

    # Create output directory if it doesn't exist
    output_dir = '/home/npc2301030/misc/mpc_analysis/trajectory_plots'
    os.makedirs(output_dir, exist_ok=True)

    # Create plots for each trajectory
    for idx, traj in enumerate(all_trajectories):
        traj_x = traj['x']
        traj_y = traj['y']
        traj_yaw = traj['yaw']
        traj_time = traj['time']

        traj_x_np = np.array(traj_x)
        traj_y_np = np.array(traj_y)
        traj_yaw_np = np.array(traj_yaw)

        # Calculate curvature for resampled trajectory (using MPC parameters)
        curvature_smoothing_num_traj = 15  # curvature_smoothing_num_traj (for k)
        curvature_smoothing_num_ref_steer = 15  # curvature_smoothing_num_ref_steer (for smooth_k, used in FF)
        traj_curvature_raw = calculate_curvature_from_points(traj_x_np, traj_y_np, smoothing_num=curvature_smoothing_num_traj)
        traj_curvature_smooth = calculate_curvature_from_points(traj_x_np, traj_y_np, smoothing_num=curvature_smoothing_num_ref_steer)

        # Get velocity at this time (from ego or planning trajectory)
        velocity_kmh = 0.0
        if len(all_planning_trajectories) > 0:
            planning_time_diffs = [abs(planning_traj['time'] - traj_time) for planning_traj in all_planning_trajectories]
            closest_planning_idx = np.argmin(planning_time_diffs)
            planning_traj_for_vel = all_planning_trajectories[closest_planning_idx]
            if len(planning_traj_for_vel['vx']) > 0:
                velocity_kmh = planning_traj_for_vel['vx'][0] * 3.6  # m/s to km/h

        # Create figure with 2 subplots
        fig, axes = plt.subplots(2, 1, figsize=(14, 10))
        fig.suptitle(f'Trajectory and Curvature Analysis (t={traj_time:.2f}s, v={velocity_kmh:.1f} km/h, #{idx+1}/{len(all_trajectories)})', fontsize=14)

        # Plot 1: Trajectory with arrows showing yaw angle, colored by curvature
        ax1 = axes[0]

        # Determine arrow spacing (show every N points to avoid clutter)
        num_points = len(traj_x)
        if num_points > 50:
            arrow_skip = max(1, num_points // 50)  # Show ~50 arrows
        else:
            arrow_skip = 1

        # Calculate arrow length based on trajectory scale
        x_range = np.max(traj_x) - np.min(traj_x)
        y_range = np.max(traj_y) - np.min(traj_y)
        arrow_length = max(x_range, y_range) * 0.04  # 4% of the range (increased for visibility)

        # Initialize colormap (will be set based on planning trajectory curvature range)
        curvature_norm = None
        curvature_cmap = plt.cm.RdYlBu_r

        # Initialize planning trajectory variables (will be used in both plots)
        planning_x_display = None
        planning_y_display = None
        planning_yaw_display = None
        planning_curvature_display = None

        # Add planning trajectory to the plot with arrows (after MPC processing)
        if len(all_planning_trajectories) > 0:
            # Find planning trajectory closest to this time
            planning_time_diffs = [abs(planning_traj['time'] - traj_time) for planning_traj in all_planning_trajectories]
            closest_planning_idx = np.argmin(planning_time_diffs)
            planning_traj = all_planning_trajectories[closest_planning_idx]

            planning_x_np = np.array(planning_traj['x'])
            planning_y_np = np.array(planning_traj['y'])
            planning_yaw_np = np.array(planning_traj['yaw'])
            planning_vx_np = np.array(planning_traj['vx'])

            # Draw original planning trajectory (before MPC processing) with black arrows
            num_planning_raw = len(planning_x_np)
            if num_planning_raw > 0:
                if num_planning_raw > 50:
                    planning_raw_arrow_skip = max(1, num_planning_raw // 50)
                else:
                    planning_raw_arrow_skip = 1

                for i in range(0, num_planning_raw, planning_raw_arrow_skip):
                    dx = arrow_length * 0.9 * np.cos(planning_yaw_np[i])  # Slightly shorter to show underneath
                    dy = arrow_length * 0.9 * np.sin(planning_yaw_np[i])
                    ax1.arrow(planning_x_np[i], planning_y_np[i], dx, dy,
                             head_width=arrow_length*0.7, head_length=arrow_length*0.4,
                             fc='none', ec='black', alpha=0.3, linewidth=2.0, zorder=5)

            # Apply MPC-like processing to planning trajectory
            # MPC default params from lateral_controller_defaults.param.yaml
            resample_interval = 0.1  # traj_resample_dist [m]
            enable_path_smoothing = False  # default: false
            path_smoothing_num = 25  # path_filter_moving_ave_num (only used if enable_path_smoothing=true)
            extend_trajectory_for_end_yaw_control = True
            is_forward_shift = True  # assume forward driving

            try:
                # Step 0: Convert to MPC trajectory format (calculate relative_time)
                # This mimics convertToMPCTrajectory() and calcMPCTrajectoryTime()
                planning_relative_time = calc_mpc_trajectory_time(planning_x_np, planning_y_np, planning_vx_np)

                # Get ego position at this trajectory time
                ego_x_current = None
                ego_y_current = None
                if len(ego_positions) > 0:
                    time_diffs_ego = [abs(ego['time'] - traj_time) for ego in ego_positions]
                    closest_ego_idx = np.argmin(time_diffs_ego)
                    ego_pos_current = ego_positions[closest_ego_idx]
                    ego_x_current = ego_pos_current['x']
                    ego_y_current = ego_pos_current['y']

                # Step 1: Resample by distance (spline interpolation, from ego position like MPC)
                planning_x_resampled, planning_y_resampled, planning_yaw_resampled, _ = \
                    resample_trajectory_by_distance(planning_x_np, planning_y_np, planning_yaw_np,
                                                   resample_interval, ego_x_current, ego_y_current)

                planning_x_display = planning_x_resampled
                planning_y_display = planning_y_resampled
                planning_yaw_display = planning_yaw_resampled
                planning_vx_display = np.full(len(planning_x_resampled), planning_vx_np[-1])  # Use terminal velocity
                planning_k_display = np.zeros(len(planning_x_resampled))  # Will be calculated later
                planning_smooth_k_display = np.zeros(len(planning_x_resampled))
                planning_relative_time_display = calc_mpc_trajectory_time(planning_x_resampled, planning_y_resampled, planning_vx_display)

                # Step 2: Apply path smoothing (moving average filter) if enabled
                if enable_path_smoothing and len(planning_x_resampled) > 2 * path_smoothing_num:
                    planning_x_display = moving_average_filter(planning_x_resampled, path_smoothing_num)
                    planning_y_display = moving_average_filter(planning_y_resampled, path_smoothing_num)
                    planning_yaw_display = moving_average_filter(planning_yaw_resampled, path_smoothing_num)

                # Step 2.5: Extend trajectory in yaw direction (if enabled)
                # Note: uses original terminal yaw (before smoothing), same as MPC
                if extend_trajectory_for_end_yaw_control:
                    terminal_yaw_raw = planning_yaw_np[-1]  # Original terminal yaw
                    planning_x_display, planning_y_display, planning_yaw_display, \
                    planning_vx_display, planning_k_display, planning_smooth_k_display, \
                    planning_relative_time_display = extend_trajectory_in_yaw_direction(
                        planning_x_display, planning_y_display, planning_yaw_display,
                        planning_vx_display, planning_k_display, planning_smooth_k_display,
                        planning_relative_time_display, terminal_yaw_raw, resample_interval, is_forward_shift)

                # Step 3: Recalculate yaw from XY coordinates (after path smoothing and extension)
                planning_yaw_display = calc_trajectory_yaw_from_xy(planning_x_display, planning_y_display, is_forward_shift)

                # Step 4: Convert Euler angles to monotonic
                planning_yaw_display = convert_euler_angle_to_monotonic(planning_yaw_display)

                # Step 5: Calculate curvature for color mapping
                curvature_smoothing_num = 15
                planning_curvature_display = calculate_curvature_from_points(
                    planning_x_display, planning_y_display, smoothing_num=curvature_smoothing_num)

                # Normalize curvature for color mapping
                if len(planning_curvature_display) > 0:
                    curvature_norm = plt.Normalize(vmin=np.min(planning_curvature_display),
                                                   vmax=np.max(planning_curvature_display))
                    curvature_cmap = plt.cm.RdYlBu_r

                # Draw arrows for processed planning trajectory
                num_planning_points = len(planning_x_display)
                if num_planning_points > 50:
                    planning_arrow_skip = max(1, num_planning_points // 50)
                else:
                    planning_arrow_skip = 1

                for i in range(0, num_planning_points, planning_arrow_skip):
                    # Get color based on curvature
                    if i < len(planning_curvature_display):
                        arrow_color = curvature_cmap(curvature_norm(planning_curvature_display[i]))
                    else:
                        arrow_color = 'darkgreen'

                    dx = arrow_length * np.cos(planning_yaw_display[i])
                    dy = arrow_length * np.sin(planning_yaw_display[i])
                    ax1.arrow(planning_x_display[i], planning_y_display[i], dx, dy,
                             head_width=arrow_length*0.6, head_length=arrow_length*0.4,
                             fc=arrow_color, ec=arrow_color, alpha=0.8, linewidth=1.5, zorder=4)

                # Draw start and end arrows
                # Start arrow (green)
                if len(planning_x_display) > 1:
                    dx_start = arrow_length * np.cos(planning_yaw_display[0])
                    dy_start = arrow_length * np.sin(planning_yaw_display[0])
                    ax1.arrow(planning_x_display[0], planning_y_display[0], dx_start, dy_start,
                             head_width=arrow_length*0.8, head_length=arrow_length*0.6,
                             fc='green', ec='green', alpha=1.0, linewidth=2.5, zorder=10,
                             label='Planning Traj MPC-like')
                    # End arrow (red)
                    dx_end = arrow_length * np.cos(planning_yaw_display[-1])
                    dy_end = arrow_length * np.sin(planning_yaw_display[-1])
                    ax1.arrow(planning_x_display[-1], planning_y_display[-1], dx_end, dy_end,
                             head_width=arrow_length*0.8, head_length=arrow_length*0.6,
                             fc='red', ec='red', alpha=1.0, linewidth=2.5, zorder=10,
                             label='Planning Traj End')
                # Add a black arrow in legend
                if len(planning_x_np) > 0:
                    ax1.arrow(0, 0, 0, 0, fc='none', ec='black', alpha=0.7, linewidth=2.0, label='Planning Traj (raw)')
            except ValueError as e:
                # If planning trajectory has less than 2 points, skip it
                print(f"Warning: Skipping planning trajectory visualization: {e}")

        # Find and plot ego vehicle position at this trajectory time
        if len(ego_positions) > 0:
            # Find closest ego position to this trajectory time
            time_diffs = [abs(ego['time'] - traj_time) for ego in ego_positions]
            closest_idx = np.argmin(time_diffs)
            ego_pos = ego_positions[closest_idx]
            ax1.plot(ego_pos['x'], ego_pos['y'], 'b^', markersize=14,
                    label=f"Ego Position (t={ego_pos['time']:.2f}s)", zorder=11,
                    markeredgecolor='white', markeredgewidth=2)

        # Add colorbar (based on planning trajectory curvature range)
        if curvature_norm is not None:
            sm = plt.cm.ScalarMappable(cmap=curvature_cmap, norm=curvature_norm)
            sm.set_array([])
            cbar = fig.colorbar(sm, ax=ax1)
            cbar.set_label('Curvature [1/m]')

        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Y [m]')
        ax1.set_title(f'Planning Trajectory (MPC-like processed) with Yaw Angles')
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc='best', fontsize=9)
        ax1.axis('equal')

        # Set axis limits based on planning trajectory range (if available)
        if planning_x_display is not None and len(planning_x_display) > 0:
            x_margin = (np.max(planning_x_display) - np.min(planning_x_display)) * 0.1
            y_margin = (np.max(planning_y_display) - np.min(planning_y_display)) * 0.1
            ax1.set_xlim(np.min(planning_x_display) - x_margin, np.max(planning_x_display) + x_margin)
            ax1.set_ylim(np.min(planning_y_display) - y_margin, np.max(planning_y_display) + y_margin)

        # Plot 2: Curvature along trajectory index
        ax2 = axes[1]

        # Add planning trajectory curvature (reuse calculated values from plot 1)
        if planning_curvature_display is not None:
            planning_indices_display = np.arange(len(planning_x_display))
            ax2.plot(planning_indices_display, planning_curvature_display, 'green', linewidth=2.0, alpha=0.8,
                    label=f'Planning Traj MPC-like ({len(planning_indices_display)} pts)', linestyle='-')

        # Add diagnostic curvature at this trajectory time
        if len(diag_curvature_data) > 0 and len(ego_positions) > 0:
            # Find diagnostic curvature closest to this trajectory time
            time_diffs = [abs(diag['time'] - traj_time) for diag in diag_curvature_data]
            closest_idx = np.argmin(time_diffs)
            diag = diag_curvature_data[closest_idx]

            # Find ego position at this time
            ego_time_diffs = [abs(ego['time'] - traj_time) for ego in ego_positions]
            ego_idx = np.argmin(ego_time_diffs)
            ego_pos = ego_positions[ego_idx]

            # Find nearest trajectory point to ego position
            distances = np.sqrt((traj_x_np - ego_pos['x'])**2 + (traj_y_np - ego_pos['y'])**2)
            nearest_traj_idx = np.argmin(distances)

            # Draw horizontal lines to show diagnostic curvature values
            ax2.axhline(y=diag['curvature_raw'], color='cyan', linestyle='--', linewidth=2,
                       label=f"Diag Raw k at Ego: {diag['curvature_raw']:.4f}", alpha=0.8)
            ax2.axhline(y=diag['curvature_smooth'], color='magenta', linestyle='--', linewidth=2,
                       label=f"Diag Smooth k at Ego: {diag['curvature_smooth']:.4f}", alpha=0.8)

            # Show the trajectory curvature at nearest point
            if nearest_traj_idx < len(traj_curvature_smooth):
                ax2.scatter([nearest_traj_idx], [traj_curvature_smooth[nearest_traj_idx]],
                           s=100, c='red', marker='x', zorder=10,
                           label=f"Traj smooth_k[{nearest_traj_idx}]={traj_curvature_smooth[nearest_traj_idx]:.4f}")

        ax2.set_xlabel('Trajectory Point Index')
        ax2.set_ylabel('Curvature [1/m]')
        ax2.set_title('Curvature Comparison: Trajectory vs Diagnostic (at Ego)')
        ax2.grid(True, alpha=0.3)
        ax2.legend(fontsize=8)

        plt.tight_layout()

        # Save figure
        output_file = os.path.join(output_dir, f'trajectory_curvature_{idx+1:04d}.png')
        plt.savefig(output_file, dpi=150)
        plt.close(fig)

    print(f"  Saved {len(all_trajectories)} trajectory plots to: {output_dir}/trajectory_curvature_*.png")
