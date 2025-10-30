#!/usr/bin/env python3

import sys
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict
import json

def analyze_mcap(mcap_path):
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
            "/vehicle/status/actuation_status",
        ]

        for schema, channel, message, decoded_msg in reader.iter_decoded_messages(topics=target_topics):
            timestamp = message.log_time / 1e9  # Convert to seconds
            topics_data[channel.topic].append({
                'time': timestamp,
                'msg': decoded_msg
            })

    print(f"\nCollected data:")
    for topic, data in topics_data.items():
        print(f"  {topic}: {len(data)} messages")

    return topics_data

def plot_mpc_analysis(data):
    """Plot MPC performance metrics"""

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

    # Extract control data (lateral error, heading error) from diagnostic topic
    lateral_error_times = []
    lateral_errors = []
    heading_errors = []
    if "/control/trajectory_follower/lateral/diagnostic" in data:
        for msg_data in data["/control/trajectory_follower/lateral/diagnostic"]:
            lateral_error_times.append(msg_data['time'])
            # diagnostic msg is Float32MultiArrayStamped with data array
            # Typical order: [lateral_error, heading_error, ...]
            if len(msg_data['msg'].data) >= 2:
                lateral_errors.append(msg_data['msg'].data[0])
                heading_errors.append(msg_data['msg'].data[1])

    # Extract velocity
    velocity_times = []
    velocities = []
    if "/localization/kinematic_state" in data:
        for msg_data in data["/localization/kinematic_state"]:
            velocity_times.append(msg_data['time'])
            vel = msg_data['msg'].twist.twist.linear
            velocities.append(np.sqrt(vel.x**2 + vel.y**2))

    # Normalize time to start from 0
    if control_times:
        t0 = min(control_times)
        control_times = [t - t0 for t in control_times]
        actual_times = [t - t0 for t in actual_times]
        lateral_error_times = [t - t0 for t in lateral_error_times]
        velocity_times = [t - t0 for t in velocity_times]

    # Create plots
    fig, axes = plt.subplots(4, 1, figsize=(12, 10))
    fig.suptitle('MPC Performance Analysis', fontsize=16)

    # Plot 1: Steering command vs actual
    ax1 = axes[0]
    if control_times:
        ax1.plot(control_times, np.rad2deg(control_steers), 'b-', label='Command', linewidth=1.5)
    if actual_times:
        ax1.plot(actual_times, np.rad2deg(actual_steers), 'r--', label='Actual', linewidth=1.5, alpha=0.7)
    ax1.set_ylabel('Steering Angle [deg]')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.set_title('Steering Command vs Actual')

    # Plot 2: Lateral error
    ax2 = axes[1]
    if lateral_error_times:
        ax2.plot(lateral_error_times, lateral_errors, 'g-', linewidth=1.5)
        ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax2.set_ylabel('Lateral Error [m]')
    ax2.grid(True, alpha=0.3)
    ax2.set_title('Lateral Error')

    # Plot 3: Heading error
    ax3 = axes[2]
    if lateral_error_times:
        ax3.plot(lateral_error_times, np.rad2deg(heading_errors), 'purple', linewidth=1.5)
        ax3.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax3.set_ylabel('Heading Error [deg]')
    ax3.grid(True, alpha=0.3)
    ax3.set_title('Heading Error')

    # Plot 4: Velocity
    ax4 = axes[3]
    if velocity_times:
        ax4.plot(velocity_times, velocities, 'orange', linewidth=1.5)
    ax4.set_ylabel('Velocity [m/s]')
    ax4.set_xlabel('Time [s]')
    ax4.grid(True, alpha=0.3)
    ax4.set_title('Vehicle Velocity')

    plt.tight_layout()

    # Save figure
    output_file = '/home/npc2301030/mpc_analysis.png'
    plt.savefig(output_file, dpi=150)
    print(f"\nPlot saved to: {output_file}")

    # Calculate statistics
    print("\n" + "="*60)
    print("Statistics")
    print("="*60)

    if lateral_errors:
        print(f"\nLateral Error:")
        print(f"  Mean: {np.mean(np.abs(lateral_errors)):.3f} m")
        print(f"  Max: {np.max(np.abs(lateral_errors)):.3f} m")
        print(f"  Std: {np.std(lateral_errors):.3f} m")

    if heading_errors:
        print(f"\nHeading Error:")
        print(f"  Mean: {np.mean(np.abs(np.rad2deg(heading_errors))):.3f} deg")
        print(f"  Max: {np.max(np.abs(np.rad2deg(heading_errors))):.3f} deg")
        print(f"  Std: {np.std(np.rad2deg(heading_errors)):.3f} deg")

    if control_steers and actual_steers:
        # Interpolate to compare at same time points
        if len(control_times) > 0 and len(actual_times) > 0:
            steering_error = []
            for i, t in enumerate(control_times):
                # Find closest actual steering time
                idx = np.argmin(np.abs(np.array(actual_times) - t))
                if abs(actual_times[idx] - t) < 0.1:  # Within 100ms
                    steering_error.append(control_steers[i] - actual_steers[idx])

            if steering_error:
                print(f"\nSteering Tracking Error:")
                print(f"  Mean: {np.mean(np.abs(np.rad2deg(steering_error))):.3f} deg")
                print(f"  Max: {np.max(np.abs(np.rad2deg(steering_error))):.3f} deg")

    if velocities:
        print(f"\nVelocity:")
        print(f"  Mean: {np.mean(velocities):.2f} m/s")
        print(f"  Max: {np.max(velocities):.2f} m/s")
        print(f"  Min: {np.min(velocities):.2f} m/s")

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

        plot_mpc_analysis(data)

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
