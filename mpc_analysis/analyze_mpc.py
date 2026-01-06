#!/usr/bin/env python3

import sys
import math
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict
from plot_trajectory_curvature import plot_trajectory_curvature

def analyze_mcap(mcap_path):
    """Read MCAP file and extract raw ROS messages

    Args:
        mcap_path: Path to MCAP file

    Returns:
        dict: Dictionary of topic name -> list of {time, msg} dicts
    """
    topics_data = defaultdict(list)
    decoder_factory = DecoderFactory()

    print("Reading MCAP file...")
    with open(mcap_path, "rb") as f:
        reader = make_reader(f, decoder_factories=[decoder_factory])

        target_topics = [
            "/control/trajectory_follower/control_cmd",
            "/control/trajectory_follower/lateral/predicted_trajectory",
            "/control/trajectory_follower/controller_node_exe/debug/resampled_reference_trajectory",
            "/control/trajectory_follower/lateral/diagnostic",
            "/localization/kinematic_state",
            "/vehicle/status/steering_status",
            "/planning/trajectory",
        ]

        for _, channel, message, decoded_msg in reader.iter_decoded_messages(topics=target_topics):
            timestamp = message.log_time / 1e9  # Convert to seconds
            topics_data[channel.topic].append({
                'time': timestamp,
                'msg': decoded_msg
            })

    return topics_data

def calculate_derivative(times, values, method='central'):
    """Calculate time derivative with noise consideration"""
    derivatives = []
    for i in range(len(times)):
        if method == 'central' and i > 0 and i < len(times) - 1:
            dt = times[i+1] - times[i-1]
            dv = values[i+1] - values[i-1]
            derivatives.append(dv / dt if dt > 0 else 0.0)
        elif i > 0:
            dt = times[i] - times[i-1]
            dv = values[i] - values[i-1]
            derivatives.append(dv / dt if dt > 0 else 0.0)
        else:
            derivatives.append(0.0)
    return derivatives

def detect_stop_start_time(velocity_times, velocities, stop_threshold=0.1, min_stop_duration=0.5):
    """
    データの最後の停止区間の開始時刻を検出

    Args:
        velocity_times: 時刻配列 [s]
        velocities: 速度配列 [m/s]
        stop_threshold: 停止判定閾値 [m/s]
        min_stop_duration: 停止と判定する最小継続時間 [s]

    Returns:
        停止開始時刻 [s] (停止が見つからない場合はNone)
    """
    if len(velocity_times) == 0 or len(velocities) == 0:
        return None

    # 最後から逆順にスキャンして停止区間を探す
    stop_start_idx = None
    in_stop_region = False

    for i in range(len(velocities) - 1, -1, -1):
        if velocities[i] < stop_threshold:
            if not in_stop_region:
                # 停止区間に入った
                in_stop_region = True
                stop_end_idx = i
            stop_start_idx = i
        else:
            if in_stop_region:
                # 停止区間から出た
                # 停止継続時間をチェック
                stop_duration = velocity_times[stop_end_idx] - velocity_times[stop_start_idx]
                if stop_duration >= min_stop_duration:
                    # 十分な長さの停止区間が見つかった
                    return velocity_times[stop_start_idx]
                else:
                    # 短すぎる停止区間なので無視して続行
                    in_stop_region = False
                    stop_start_idx = None

    # データの最初まで停止区間が続いていた場合
    if in_stop_region and stop_start_idx is not None:
        stop_duration = velocity_times[-1] - velocity_times[stop_start_idx]
        if stop_duration >= min_stop_duration:
            return velocity_times[stop_start_idx]

    return None

def extract_all_data(data):
    """Extract and normalize all data from MCAP data dictionary

    Args:
        data: MCAP data dictionary from analyze_mcap()

    Returns:
        dict: Extracted data with absolute timestamps
    """
    # Extract control commands (steering)
    control_times = []
    control_steers = []
    if "/control/trajectory_follower/control_cmd" in data:
        for msg_data in data["/control/trajectory_follower/control_cmd"]:
            control_times.append(msg_data['time'])
            control_steers.append(msg_data['msg'].lateral.steering_tire_angle)

    # Extract actual steering
    actual_times = []
    actual_steers = []
    if "/vehicle/status/steering_status" in data:
        for msg_data in data["/vehicle/status/steering_status"]:
            actual_times.append(msg_data['time'])
            actual_steers.append(msg_data['msg'].steering_tire_angle)

    # Extract diagnostic data (full MPC internal states)
    diag_times = []
    diag_final_steer = []
    diag_mpc_raw = []
    diag_ff_steer = []
    diag_ff_steer_raw = []
    diag_current_steer = []
    lateral_errors = []
    diag_current_yaw = []
    diag_reference_yaw = []
    heading_errors = []
    diag_ref_velocity = []
    diag_measured_velocity = []
    diag_curvature_smooth = []
    diag_curvature_raw = []
    diag_predicted_steer = []
    diag_iteration_num = []
    diag_runtime = []
    diag_obj_value = []

    if "/control/trajectory_follower/lateral/diagnostic" in data:
        for msg_data in data["/control/trajectory_follower/lateral/diagnostic"]:
            diag_times.append(msg_data['time'])
            diag_data = msg_data['msg'].data

            if len(diag_data) >= 21:
                diag_final_steer.append(diag_data[0])
                diag_mpc_raw.append(diag_data[1])
                diag_ff_steer.append(diag_data[2])
                diag_ff_steer_raw.append(diag_data[3])
                diag_current_steer.append(diag_data[4])
                lateral_errors.append(diag_data[5])
                diag_current_yaw.append(diag_data[6])
                diag_reference_yaw.append(diag_data[7])
                heading_errors.append(diag_data[8])
                diag_ref_velocity.append(diag_data[9])
                diag_measured_velocity.append(diag_data[10])
                diag_curvature_smooth.append(diag_data[14])
                diag_curvature_raw.append(diag_data[15])
                diag_predicted_steer.append(diag_data[16])
                diag_iteration_num.append(diag_data[18])
                diag_runtime.append(diag_data[19])
                diag_obj_value.append(diag_data[20])

    # Extract velocity
    velocity_times = []
    velocities = []
    if "/localization/kinematic_state" in data:
        for msg_data in data["/localization/kinematic_state"]:
            velocity_times.append(msg_data['time'])
            vel = msg_data['msg'].twist.twist.linear
            velocities.append(np.sqrt(vel.x**2 + vel.y**2))

    # Normalize time to start from 0
    all_times = []
    if control_times:
        all_times.extend(control_times)
    if actual_times:
        all_times.extend(actual_times)
    if diag_times:
        all_times.extend(diag_times)
    if velocity_times:
        all_times.extend(velocity_times)

    t0 = min(all_times) if all_times else 0
    control_times = [t - t0 for t in control_times]
    actual_times = [t - t0 for t in actual_times]
    diag_times = [t - t0 for t in diag_times]
    velocity_times = [t - t0 for t in velocity_times]

    return {
        'control_times': control_times,
        'control_steers': control_steers,
        'actual_times': actual_times,
        'actual_steers': actual_steers,
        'diag_times': diag_times,
        'diag_final_steer': diag_final_steer,
        'diag_mpc_raw': diag_mpc_raw,
        'diag_ff_steer': diag_ff_steer,
        'diag_ff_steer_raw': diag_ff_steer_raw,
        'diag_current_steer': diag_current_steer,
        'lateral_errors': lateral_errors,
        'diag_current_yaw': diag_current_yaw,
        'diag_reference_yaw': diag_reference_yaw,
        'heading_errors': heading_errors,
        'diag_ref_velocity': diag_ref_velocity,
        'diag_measured_velocity': diag_measured_velocity,
        'diag_curvature_smooth': diag_curvature_smooth,
        'diag_curvature_raw': diag_curvature_raw,
        'diag_predicted_steer': diag_predicted_steer,
        'diag_iteration_num': diag_iteration_num,
        'diag_runtime': diag_runtime,
        'diag_obj_value': diag_obj_value,
        'velocity_times': velocity_times,
        'velocities': velocities,
        't0': t0
    }

def analyze_data(extracted_data):
    """Analyze data: detect stop, determine time window, calculate derivatives, print statistics

    Args:
        extracted_data: Dictionary from extract_all_data()

    Returns:
        dict: Analysis results including time_window_start, time_window_end, derivatives, etc.
    """
    velocity_times = extracted_data['velocity_times']
    velocities = extracted_data['velocities']
    diag_times = extracted_data['diag_times']
    diag_final_steer = extracted_data['diag_final_steer']
    diag_mpc_raw = extracted_data['diag_mpc_raw']
    diag_ff_steer = extracted_data['diag_ff_steer']

    # Detect stop start time
    stop_start_time = detect_stop_start_time(velocity_times, velocities, stop_threshold=0.1, min_stop_duration=0.5)

    # Determine time window for plotting
    time_window_start = None
    time_window_end = None

    if stop_start_time is not None:
        time_window_start = stop_start_time - 5.0
        time_window_end = stop_start_time

        print(f"  Time window for plotting: [{time_window_start:.2f}, {time_window_end:.2f}] s")
    else:
        print("\nWarning: No stop detected in data. Plotting full range.")

    # Calculate steering rate (time derivatives)
    diag_final_steer_rate = []
    diag_mpc_raw_rate = []
    if diag_times and len(diag_times) > 1:
        diag_final_steer_rate = calculate_derivative(diag_times, diag_final_steer)
        diag_mpc_raw_rate = calculate_derivative(diag_times, diag_mpc_raw)

    # Calculate MPC raw vs LPF difference
    diag_lpf_effect = []
    if len(diag_final_steer) > 0 and len(diag_mpc_raw) > 0:
        diag_lpf_effect = [np.rad2deg(final - raw) for final, raw in zip(diag_final_steer, diag_mpc_raw)]

    # Calculate feedback component
    diag_feedback = []
    if len(diag_mpc_raw) > 0 and len(diag_ff_steer) > 0:
        diag_feedback = [mpc - ff for mpc, ff in zip(diag_mpc_raw, diag_ff_steer)]

    # Interpolate velocity to diagnostic times
    diag_velocities = []
    if len(diag_times) > 0 and len(velocity_times) > 0 and len(velocities) > 0:
        diag_velocities = np.interp(diag_times, velocity_times, velocities)

    return {
        'time_window_start': time_window_start,
        'time_window_end': time_window_end,
        'diag_final_steer_rate': diag_final_steer_rate,
        'diag_mpc_raw_rate': diag_mpc_raw_rate,
        'diag_lpf_effect': diag_lpf_effect,
        'diag_feedback': diag_feedback,
        'diag_velocities': diag_velocities
    }

def extract_trajectory_data(data, extracted_data, analysis_results):
    """Extract trajectory data for visualization within time window

    Args:
        data: Original MCAP data dictionary
        extracted_data: Extracted data from extract_all_data()
        analysis_results: Analysis results from analyze_data()

    Returns:
        dict: Trajectory data for visualization
    """
    t0 = extracted_data['t0']
    time_window_start = analysis_results['time_window_start']
    time_window_end = analysis_results['time_window_end']

    if time_window_start is None or time_window_end is None:
        return {
            'ego_positions': [],
            'diag_curvature_data': [],
            'resampled_trajectories': [],
            'planning_trajectories': []
        }

    trajectory_data = {
        'ego_positions': [],
        'diag_curvature_data': [],
        'resampled_trajectories': [],
        'planning_trajectories': []
    }

    # Extract ego vehicle positions
    if "/localization/kinematic_state" in data:
        for msg_data in data["/localization/kinematic_state"]:
            ego_time = msg_data['time']
            if time_window_start <= (ego_time - t0) <= time_window_end:
                trajectory_data['ego_positions'].append({
                    'time': ego_time,
                    'x': msg_data['msg'].pose.pose.position.x,
                    'y': msg_data['msg'].pose.pose.position.y
                })

    # Extract diagnostic curvature data
    if "/control/trajectory_follower/lateral/diagnostic" in data:
        for msg_data in data["/control/trajectory_follower/lateral/diagnostic"]:
            diag_time = msg_data['time']
            if time_window_start <= (diag_time - t0) <= time_window_end:
                diag_data = msg_data['msg'].data
                if len(diag_data) >= 21:
                    trajectory_data['diag_curvature_data'].append({
                        'time': diag_time,
                        'curvature_raw': diag_data[15],
                        'curvature_smooth': diag_data[14]
                    })

    # Extract resampled trajectories
    if "/control/trajectory_follower/controller_node_exe/debug/resampled_reference_trajectory" in data:
        for traj_data in data["/control/trajectory_follower/controller_node_exe/debug/resampled_reference_trajectory"]:
            traj_time = traj_data['time']
            if time_window_start <= (traj_time - t0) <= time_window_end:
                traj_x = []
                traj_y = []
                traj_yaw = []
                msg = traj_data['msg']

                for point in msg.points:
                    traj_x.append(point.pose.position.x)
                    traj_y.append(point.pose.position.y)

                    qx = point.pose.orientation.x
                    qy = point.pose.orientation.y
                    qz = point.pose.orientation.z
                    qw = point.pose.orientation.w
                    siny_cosp = 2 * (qw * qz + qx * qy)
                    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
                    yaw = math.atan2(siny_cosp, cosy_cosp)
                    traj_yaw.append(yaw)

                if len(traj_x) > 0:
                    trajectory_data['resampled_trajectories'].append({
                        'time': traj_time,
                        'x': traj_x,
                        'y': traj_y,
                        'yaw': traj_yaw
                    })

    # Extract planning trajectories
    if "/planning/trajectory" in data:
        for traj_data in data["/planning/trajectory"]:
            traj_time = traj_data['time']
            if time_window_start <= (traj_time - t0) <= time_window_end:
                traj_x = []
                traj_y = []
                traj_yaw = []
                traj_vx = []
                msg = traj_data['msg']

                for point in msg.points:
                    traj_x.append(point.pose.position.x)
                    traj_y.append(point.pose.position.y)

                    qx = point.pose.orientation.x
                    qy = point.pose.orientation.y
                    qz = point.pose.orientation.z
                    qw = point.pose.orientation.w
                    siny_cosp = 2 * (qw * qz + qx * qy)
                    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
                    yaw = math.atan2(siny_cosp, cosy_cosp)
                    traj_yaw.append(yaw)

                    traj_vx.append(point.longitudinal_velocity_mps)

                if len(traj_x) > 0:
                    trajectory_data['planning_trajectories'].append({
                        'time': traj_time,
                        'x': traj_x,
                        'y': traj_y,
                        'yaw': traj_yaw,
                        'vx': traj_vx
                    })

    return trajectory_data

def plot_mpc_analysis(extracted_data, analysis_results):
    """Plot MPC performance metrics

    Args:
        extracted_data: Extracted data from extract_all_data()
        analysis_results: Analysis results from analyze_data()
    """
    control_times = extracted_data['control_times']
    control_steers = extracted_data['control_steers']
    actual_times = extracted_data['actual_times']
    actual_steers = extracted_data['actual_steers']
    diag_times = extracted_data['diag_times']
    diag_final_steer = extracted_data['diag_final_steer']
    diag_mpc_raw = extracted_data['diag_mpc_raw']
    diag_ff_steer = extracted_data['diag_ff_steer']
    diag_ff_steer_raw = extracted_data['diag_ff_steer_raw']
    diag_curvature_smooth = extracted_data['diag_curvature_smooth']
    diag_curvature_raw = extracted_data['diag_curvature_raw']
    diag_current_yaw = extracted_data['diag_current_yaw']
    diag_reference_yaw = extracted_data['diag_reference_yaw']
    lateral_errors = extracted_data['lateral_errors']
    heading_errors = extracted_data['heading_errors']
    velocity_times = extracted_data['velocity_times']
    velocities = extracted_data['velocities']

    time_window_start = analysis_results['time_window_start']
    time_window_end = analysis_results['time_window_end']
    diag_feedback = analysis_results['diag_feedback']
    diag_velocities = analysis_results['diag_velocities']

    # Define low-speed thresholds
    STOP_THRESHOLD = 0.2
    LOW_SPEED_1 = 1.0
    LOW_SPEED_2 = 2.0
    LOW_SPEED_3 = 3.0

    # Helper function to add low-speed region highlighting
    def highlight_low_speed(ax, times, velocities):
        if len(times) == 0 or len(velocities) == 0:
            return
        for i in range(len(times)):
            if i == 0:
                continue
            v = velocities[i] if i < len(velocities) else velocities[-1]
            if v < STOP_THRESHOLD:
                ax.axvspan(times[i-1], times[i], alpha=0.3, color='red', linewidth=0)
            elif v < LOW_SPEED_1:
                ax.axvspan(times[i-1], times[i], alpha=0.2, color='orange', linewidth=0)
            elif v < LOW_SPEED_2:
                ax.axvspan(times[i-1], times[i], alpha=0.15, color='yellow', linewidth=0)

    # Create plots (6 subplots)
    fig = plt.figure(figsize=(16, 18))
    gs = fig.add_gridspec(6, 1, hspace=0.3)
    axes = [fig.add_subplot(gs[i, 0]) for i in range(6)]
    fig.suptitle('MPC Performance Analysis - Low Speed Steering Behavior', fontsize=16, y=0.995)

    # Plot 1: Steering angle with Velocity
    ax1 = axes[0]
    ax1_vel = ax1.twinx()

    if len(diag_times) > 0 and len(diag_velocities) > 0:
        highlight_low_speed(ax1, diag_times, diag_velocities)
    if len(diag_times) > 0 and len(diag_mpc_raw) > 0:
        ax1.plot(diag_times, np.rad2deg(diag_mpc_raw), 'b-', label='MPC Raw', linewidth=1.2, alpha=0.7)
    if len(diag_times) > 0 and len(diag_final_steer) > 0:
        ax1.plot(diag_times, np.rad2deg(diag_final_steer), 'g-', label='MPC Final (LPF)', linewidth=1.5)
    if len(actual_times) > 0:
        ax1.plot(actual_times, np.rad2deg(actual_steers), 'r--', label='Actual', linewidth=1.2, alpha=0.6)

    if len(velocity_times) > 0:
        ax1_vel.plot(velocity_times, [v * 3.6 for v in velocities], 'orange',
                     linewidth=1.5, alpha=0.8, label='Velocity')
        ax1_vel.set_ylabel('Velocity [km/h]', color='orange')
        ax1_vel.tick_params(axis='y', labelcolor='orange')

    ax1.set_ylabel('Steering Angle [deg]')
    ax1.grid(True, alpha=0.3)
    ax1.set_title('Steering Angle')

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax1_vel.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right')

    # Plot 2: Feedforward and Feedback components
    ax2 = axes[1]
    if len(diag_times) > 0 and len(diag_velocities) > 0:
        highlight_low_speed(ax2, diag_times, diag_velocities)

    if len(diag_times) > 0 and len(diag_ff_steer) > 0:
        ax2.plot(diag_times, np.rad2deg(diag_ff_steer), 'orange', label='Feedforward', linewidth=1.5, alpha=0.8)
    if len(diag_times) > 0 and len(diag_feedback) > 0:
        ax2.plot(diag_times, np.rad2deg(diag_feedback), 'cyan', label='Feedback (MPC)', linewidth=1.5, alpha=0.8)
    if len(diag_times) > 0 and len(diag_mpc_raw) > 0:
        ax2.plot(diag_times, np.rad2deg(diag_mpc_raw), 'b-', label='Total (FF+FB)', linewidth=2.0)
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax2.set_ylabel('Steering Angle [deg]')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    ax2.set_title('Steering Angle Decomposition: Feedforward + Feedback')

    # Plot 3: Path curvature
    ax3 = axes[2]
    if len(diag_times) > 0 and len(diag_velocities) > 0:
        highlight_low_speed(ax3, diag_times, diag_velocities)
    if len(diag_times) > 0 and len(diag_curvature_raw) > 0:
        ax3.plot(diag_times, diag_curvature_raw, 'gray', linewidth=1.0, alpha=0.5, label='Raw Curvature')
    if len(diag_times) > 0 and len(diag_curvature_smooth) > 0:
        ax3.plot(diag_times, diag_curvature_smooth, 'brown', linewidth=2.0, label='Smoothed Curvature (used for FF)')
        ax3.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax3.set_ylabel('Curvature [1/m]')
    ax3.legend(loc='upper right')
    ax3.grid(True, alpha=0.3)
    ax3.set_title('Reference Path Curvature: Raw vs Smoothed')

    # Plot 4: Reference path yaw and vehicle yaw
    ax4 = axes[3]
    if len(diag_times) > 0 and len(diag_velocities) > 0:
        highlight_low_speed(ax4, diag_times, diag_velocities)
    if len(diag_times) > 0 and len(diag_reference_yaw) > 0:
        ax4.plot(diag_times, np.rad2deg(diag_reference_yaw), 'brown', linewidth=1.5, label='Reference Path Yaw')
    if len(diag_times) > 0 and len(diag_current_yaw) > 0:
        ax4.plot(diag_times, np.rad2deg(diag_current_yaw), 'blue', linewidth=1.5, alpha=0.7, label='Vehicle Yaw')
    ax4.set_ylabel('Yaw Angle [deg]')
    ax4.legend(loc='upper right')
    ax4.grid(True, alpha=0.3)
    ax4.set_title('Reference Path Yaw vs Vehicle Yaw')

    # Plot 5: Lateral and Heading errors
    ax5 = axes[4]
    ax5_yaw = ax5.twinx()

    if len(diag_times) > 0 and len(diag_velocities) > 0:
        highlight_low_speed(ax5, diag_times, diag_velocities)

    if len(diag_times) > 0 and len(lateral_errors) > 0:
        ax5.plot(diag_times, [e * 1000 for e in lateral_errors], 'g-', linewidth=1.5, label='Lateral Error')
        ax5.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax5.set_ylabel('Lateral Error [mm]', color='g')
    ax5.tick_params(axis='y', labelcolor='g')

    if len(diag_times) > 0 and len(heading_errors) > 0:
        ax5_yaw.plot(diag_times, np.rad2deg(heading_errors), 'purple', linewidth=1.5, label='Heading Error')
        ax5_yaw.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax5_yaw.set_ylabel('Heading Error [deg]', color='purple')
    ax5_yaw.tick_params(axis='y', labelcolor='purple')

    ax5.grid(True, alpha=0.3)
    ax5.set_title('MPC Feedback Errors: Lateral (mm) & Heading (deg)')

    lines1, labels1 = ax5.get_legend_handles_labels()
    lines2, labels2 = ax5_yaw.get_legend_handles_labels()
    ax5.legend(lines1 + lines2, labels1 + labels2, loc='upper right')

    # Plot 6: Feedforward steering: raw vs LPF applied
    ax6 = axes[5]
    if len(diag_times) > 0 and len(diag_velocities) > 0:
        highlight_low_speed(ax6, diag_times, diag_velocities)

    if len(diag_times) > 0 and len(diag_ff_steer_raw) > 0:
        ax6.plot(diag_times, np.rad2deg(diag_ff_steer_raw), 'orange', linewidth=1.5, alpha=0.8,
                label='FF Raw (no LPF)')
    if len(diag_times) > 0 and len(diag_ff_steer) > 0:
        ax6.plot(diag_times, np.rad2deg(diag_ff_steer), 'red', linewidth=2.0,
                label='FF Final (LPF applied)')
    ax6.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax6.set_ylabel('FF Steering Angle [deg]')
    ax6.set_xlabel('Time [s]')
    ax6.legend(loc='upper right')
    ax6.grid(True, alpha=0.3)
    ax6.set_title('Feedforward Steering: Raw vs LPF Applied')

    # Synchronize x-axis limits across all plots
    if time_window_start is not None and time_window_end is not None:
        for ax in axes:
            ax.set_xlim(time_window_start, time_window_end)
    elif len(control_times) > 0 or len(diag_times) > 0 or len(velocity_times) > 0:
        all_times = []
        if len(control_times) > 0:
            all_times.extend(control_times)
        if len(diag_times) > 0:
            all_times.extend(diag_times)
        if len(velocity_times) > 0:
            all_times.extend(velocity_times)

        if all_times:
            x_min, x_max = min(all_times), max(all_times)
            for ax in axes:
                ax.set_xlim(x_min, x_max)

    plt.tight_layout()

    # Save figure
    output_file = '/home/npc2301030/mpc_analysis.png'
    plt.savefig(output_file, dpi=150)
    print(f"\nPlot saved to: {output_file}")

def plot_feedback_dominance(extracted_data, analysis_results):
    """Plot feedback dominance analysis (lateral error vs yaw error)

    Args:
        extracted_data: Extracted data from extract_all_data()
        analysis_results: Analysis results from analyze_data()
    """
    # Extract relevant data
    diag_times = extracted_data['diag_times']
    lateral_errors = extracted_data['lateral_errors']
    heading_errors = extracted_data['heading_errors']
    diag_velocities = analysis_results['diag_velocities']

    time_window_start = analysis_results['time_window_start']
    time_window_end = analysis_results['time_window_end']

    if time_window_start is None or time_window_end is None:
        print("\nWarning: No time window available for feedback dominance analysis")
        return

    # Filter data within time window
    mask = (np.array(diag_times) >= time_window_start) & (np.array(diag_times) <= time_window_end)
    times = np.array(diag_times)[mask]
    lat_errors = np.array(lateral_errors)[mask]
    yaw_errors = np.array(heading_errors)[mask]
    velocities = np.array(diag_velocities)[mask]

    if len(times) == 0:
        print("\nWarning: No data in time window for feedback dominance analysis")
        return

    # MPC weight parameters (from lateral_controller_defaults.param.yaml)
    weight_lat_error = 1.0
    weight_heading_error = 0.0
    weight_heading_error_squared_vel = 0.3

    print(f"\n{'Feedback Dominance Analysis':-^80}")
    print(f"  MPC Q Matrix Weights:")
    print(f"    Lateral error:     {weight_lat_error}")
    print(f"    Heading error:     {weight_heading_error} + {weight_heading_error_squared_vel} * v²")

    # Calculate effective weights at each time
    effective_yaw_weight = weight_heading_error + weight_heading_error_squared_vel * velocities**2

    # Calculate normalized error contributions
    lat_error_contribution = weight_lat_error * lat_errors**2
    yaw_error_contribution = effective_yaw_weight * yaw_errors**2
    total_contribution = lat_error_contribution + yaw_error_contribution

    # Calculate percentages
    lat_error_percentage = np.zeros_like(lat_error_contribution)
    yaw_error_percentage = np.zeros_like(yaw_error_contribution)
    mask_nonzero = total_contribution > 1e-10
    lat_error_percentage[mask_nonzero] = 100 * lat_error_contribution[mask_nonzero] / total_contribution[mask_nonzero]
    yaw_error_percentage[mask_nonzero] = 100 * yaw_error_contribution[mask_nonzero] / total_contribution[mask_nonzero]

    print(f"  Overall dominance:")
    print(f"    Lateral error: {np.mean(lat_error_percentage):.1f}%")
    print(f"    Yaw error:     {np.mean(yaw_error_percentage):.1f}%")

    # Velocity dependency
    low_speed_mask = velocities < 1.0
    high_speed_mask = velocities >= 1.0
    if np.sum(low_speed_mask) > 0:
        print(f"  At low speed (< 1.0 m/s):")
        print(f"    Lateral error: {np.mean(lat_error_percentage[low_speed_mask]):.1f}%")
        print(f"    Yaw error:     {np.mean(yaw_error_percentage[low_speed_mask]):.1f}%")
    if np.sum(high_speed_mask) > 0:
        print(f"  At high speed (>= 1.0 m/s):")
        print(f"    Lateral error: {np.mean(lat_error_percentage[high_speed_mask]):.1f}%")
        print(f"    Yaw error:     {np.mean(yaw_error_percentage[high_speed_mask]):.1f}%")

    # Create visualization
    fig, axes = plt.subplots(4, 1, figsize=(14, 12))
    fig.suptitle('MPC Feedback Dominance Analysis', fontsize=14)

    # Plot 1: Errors
    ax1 = axes[0]
    ax1.plot(times, lat_errors * 1000, 'g-', label='Lateral Error', linewidth=1.5)
    ax1_yaw = ax1.twinx()
    ax1_yaw.plot(times, np.rad2deg(yaw_errors), 'b-', label='Yaw Error', linewidth=1.5)
    ax1.set_ylabel('Lateral Error [mm]', color='g')
    ax1_yaw.set_ylabel('Yaw Error [deg]', color='b')
    ax1.tick_params(axis='y', labelcolor='g')
    ax1_yaw.tick_params(axis='y', labelcolor='b')
    ax1.grid(True, alpha=0.3)
    ax1.set_title('Tracking Errors')
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax1_yaw.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right')

    # Plot 2: Effective weights
    ax2 = axes[1]
    ax2.axhline(y=weight_lat_error, color='g', linestyle='--', linewidth=2,
                label=f'Lateral Error Weight (const={weight_lat_error})')
    ax2.plot(times, effective_yaw_weight, 'b-', linewidth=1.5,
             label='Yaw Error Weight (0.0 + 0.3*v²)')
    ax2_vel = ax2.twinx()
    ax2_vel.plot(times, velocities * 3.6, 'orange', linewidth=1.0, alpha=0.6, label='Velocity')
    ax2.set_ylabel('MPC Weight', color='k')
    ax2_vel.set_ylabel('Velocity [km/h]', color='orange')
    ax2_vel.tick_params(axis='y', labelcolor='orange')
    ax2.grid(True, alpha=0.3)
    ax2.set_title('MPC Q Matrix Weights (velocity-dependent)')
    lines1, labels1 = ax2.get_legend_handles_labels()
    lines2, labels2 = ax2_vel.get_legend_handles_labels()
    ax2.legend(lines1 + lines2, labels1 + labels2, loc='upper right')

    # Plot 3: Error contributions
    ax3 = axes[2]
    ax3.plot(times, lat_error_contribution, 'g-', label='Lateral Error Contribution', linewidth=1.5)
    ax3.plot(times, yaw_error_contribution, 'b-', label='Yaw Error Contribution', linewidth=1.5)
    ax3.set_ylabel('Error Contribution (weight * error²)')
    ax3.grid(True, alpha=0.3)
    ax3.set_title('Weighted Error Contributions to MPC Cost')
    ax3.legend(loc='upper right')
    ax3.set_yscale('log')

    # Plot 4: Dominance percentage
    ax4 = axes[3]
    ax4.fill_between(times, 0, lat_error_percentage, color='g', alpha=0.5, label='Lateral Error Dominance')
    ax4.fill_between(times, lat_error_percentage, 100, color='b', alpha=0.5, label='Yaw Error Dominance')
    ax4.axhline(y=50, color='k', linestyle='--', linewidth=1, alpha=0.5)
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Dominance [%]')
    ax4.set_ylim([0, 100])
    ax4.grid(True, alpha=0.3)
    ax4.set_title('Error Dominance Over Time')
    ax4.legend(loc='upper right')

    plt.tight_layout()
    output_path = '/home/npc2301030/misc/mpc_analysis/feedback_dominance_analysis.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"  Plot saved to: {output_path}")
    plt.close()

def main():
    if len(sys.argv) < 2:
        print("Usage: python analyze_mpc.py <mcap_file>")
        sys.exit(1)

    mcap_path = sys.argv[1]

    try:
        data = analyze_mcap(mcap_path)

        if not data:
            print("No data collected from target topics!")
            sys.exit(1)

        extracted_data = extract_all_data(data)

        # Step 3: Analyze data (detect stop, calculate derivatives, print statistics)
        analysis_results = analyze_data(extracted_data)

        # Step 4: Plot MPC analysis
        plot_mpc_analysis(extracted_data, analysis_results)

        # Step 5: Plot feedback dominance analysis
        plot_feedback_dominance(extracted_data, analysis_results)

        # Step 6: Extract trajectory data for visualization
        trajectory_data = extract_trajectory_data(data, extracted_data, analysis_results)

        # Step 7: Plot trajectory curvature
        plot_trajectory_curvature(trajectory_data)

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
