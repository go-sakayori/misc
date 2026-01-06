#!/usr/bin/env python3
"""Analyze which error (lateral vs yaw) is dominant in MPC feedback"""

import sys
import numpy as np
import matplotlib.pyplot as plt
sys.path.insert(0, '/home/npc2301030/misc/mpc_analysis')

from analyze_mpc import analyze_mcap, extract_all_data, analyze_data

def analyze_feedback_dominance(mcap_path):
    """Analyze the dominance of lateral error vs yaw error in MPC feedback

    MPC uses Q matrix weights:
    - Q(0,0) = mpc_weight_lat_error = 1.0
    - Q(1,1) = mpc_weight_heading_error = 0.0
    - Q(1,1) += velocity^2 * mpc_weight_heading_error_squared_vel = velocity^2 * 0.3

    So the effective cost contribution is:
    - Lateral error cost: lat_error^2 * 1.0
    - Yaw error cost: yaw_error^2 * (0.0 + velocity^2 * 0.3)
    """

    # Load data
    data = analyze_mcap(mcap_path)
    extracted_data = extract_all_data(data)
    analysis_results = analyze_data(extracted_data)

    # Extract relevant data
    diag_times = extracted_data['diag_times']
    lateral_errors = extracted_data['lateral_errors']
    heading_errors = extracted_data['heading_errors']
    diag_velocities = analysis_results['diag_velocities']
    diag_feedback = analysis_results['diag_feedback']

    time_window_start = analysis_results['time_window_start']
    time_window_end = analysis_results['time_window_end']

    if time_window_start is None or time_window_end is None:
        print("No stop detected, cannot analyze")
        return

    # Filter data within time window
    mask = (np.array(diag_times) >= time_window_start) & (np.array(diag_times) <= time_window_end)
    times = np.array(diag_times)[mask]
    lat_errors = np.array(lateral_errors)[mask]
    yaw_errors = np.array(heading_errors)[mask]
    velocities = np.array(diag_velocities)[mask]
    feedbacks = np.array(diag_feedback)[mask]

    # MPC weight parameters (from lateral_controller_defaults.param.yaml)
    weight_lat_error = 1.0
    weight_heading_error = 0.0
    weight_heading_error_squared_vel = 0.3

    # Calculate effective weights at each time
    effective_yaw_weight = weight_heading_error + weight_heading_error_squared_vel * velocities**2

    # Calculate normalized error contributions (not actual cost, but proportional)
    # These represent the relative influence on the optimization
    lat_error_contribution = weight_lat_error * lat_errors**2
    yaw_error_contribution = effective_yaw_weight * yaw_errors**2

    total_contribution = lat_error_contribution + yaw_error_contribution

    # Calculate percentages (avoid division by zero)
    lat_error_percentage = np.zeros_like(lat_error_contribution)
    yaw_error_percentage = np.zeros_like(yaw_error_contribution)

    mask_nonzero = total_contribution > 1e-10
    lat_error_percentage[mask_nonzero] = 100 * lat_error_contribution[mask_nonzero] / total_contribution[mask_nonzero]
    yaw_error_percentage[mask_nonzero] = 100 * yaw_error_contribution[mask_nonzero] / total_contribution[mask_nonzero]

    # Find regions where each error dominates
    lat_dominant_mask = lat_error_percentage > 50
    yaw_dominant_mask = yaw_error_percentage > 50

    # Find correlation with velocity
    low_speed_mask = velocities < 1.0  # < 1 m/s
    high_speed_mask = velocities >= 1.0

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
    ax2.axhline(y=weight_lat_error, color='g', linestyle='--', linewidth=2, label=f'Lateral Error Weight (const={weight_lat_error})')
    ax2.plot(times, effective_yaw_weight, 'b-', linewidth=1.5, label='Yaw Error Weight (0.0 + 0.3*v²)')
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
    print(f"\nPlot saved to: {output_path}")
    plt.close()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_feedback_dominance.py <mcap_path>")
        sys.exit(1)

    mcap_path = sys.argv[1]
    analyze_feedback_dominance(mcap_path)
