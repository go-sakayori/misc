#!/usr/bin/env python3
"""
Lightweight MCAP batch analyzer - only reads diagnostic topics without full decoding
"""

import sys
import os
from pathlib import Path
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
import numpy as np
from collections import defaultdict
import json
from datetime import datetime
from multiprocessing import Pool, cpu_count
import struct

def quick_analyze_mcap(mcap_path):
    """Quick analysis - sample only 20% of messages for speed"""
    decoder_factory = DecoderFactory()

    try:
        stats = {
            'filename': os.path.basename(mcap_path),
            'filepath': str(mcap_path),
            'success': True,
            'error': None,
        }

        lateral_errors = []
        heading_errors = []
        velocities = []

        with open(mcap_path, "rb") as f:
            reader = make_reader(f, decoder_factories=[decoder_factory])

            # Only read diagnostic and kinematic state
            target_topics = [
                "/control/trajectory_follower/lateral/diagnostic",
                "/localization/kinematic_state",
            ]

            sample_counter = 0
            for schema, channel, message, decoded_msg in reader.iter_decoded_messages(topics=target_topics):
                sample_counter += 1
                # Sample only every 5th message to speed up (20% sampling)
                if sample_counter % 5 != 0:
                    continue

                if channel.topic == "/control/trajectory_follower/lateral/diagnostic":
                    if len(decoded_msg.data) >= 2:
                        lateral_errors.append(decoded_msg.data[0])
                        heading_errors.append(decoded_msg.data[1])

                elif channel.topic == "/localization/kinematic_state":
                    vel = decoded_msg.twist.twist.linear
                    velocities.append(np.sqrt(vel.x**2 + vel.y**2))

        # Calculate statistics
        if lateral_errors:
            stats['lateral_error_mean'] = float(np.mean(np.abs(lateral_errors)))
            stats['lateral_error_max'] = float(np.max(np.abs(lateral_errors)))
            stats['lateral_error_std'] = float(np.std(lateral_errors))
            stats['lateral_error_95percentile'] = float(np.percentile(np.abs(lateral_errors), 95))
            stats['sample_count'] = len(lateral_errors)
        else:
            stats['lateral_error_mean'] = None
            stats['lateral_error_max'] = None
            stats['lateral_error_std'] = None
            stats['lateral_error_95percentile'] = None
            stats['sample_count'] = 0

        if heading_errors:
            heading_errors_deg = np.rad2deg(heading_errors)
            stats['heading_error_mean'] = float(np.mean(np.abs(heading_errors_deg)))
            stats['heading_error_max'] = float(np.max(np.abs(heading_errors_deg)))
            stats['heading_error_std'] = float(np.std(heading_errors_deg))
            stats['heading_error_95percentile'] = float(np.percentile(np.abs(heading_errors_deg), 95))
        else:
            stats['heading_error_mean'] = None
            stats['heading_error_max'] = None
            stats['heading_error_std'] = None
            stats['heading_error_95percentile'] = None

        if velocities:
            stats['velocity_mean'] = float(np.mean(velocities))
            stats['velocity_max'] = float(np.max(velocities))
            stats['velocity_min'] = float(np.min(velocities))
        else:
            stats['velocity_mean'] = None
            stats['velocity_max'] = None
            stats['velocity_min'] = None

        return stats

    except Exception as e:
        return {
            'filename': os.path.basename(mcap_path),
            'filepath': str(mcap_path),
            'success': False,
            'error': str(e),
            'sample_count': 0,
        }

def process_wrapper(args):
    mcap_path, index, total = args
    stats = quick_analyze_mcap(mcap_path)
    return (index, total, stats)

def generate_report(all_stats, output_dir):
    """Generate report"""
    successful_stats = [s for s in all_stats if s['success']]
    failed_stats = [s for s in all_stats if not s['success']]

    report = []
    report.append("=" * 80)
    report.append("MPC Batch Analysis Report (Quick Mode - 20% Sampling)")
    report.append("=" * 80)
    report.append(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    report.append(f"Total files: {len(all_stats)}")
    report.append(f"Successfully analyzed: {len(successful_stats)}")
    report.append(f"Failed: {len(failed_stats)}")
    report.append("=" * 80)
    report.append("")

    if successful_stats:
        lateral_means = [s['lateral_error_mean'] for s in successful_stats if s['lateral_error_mean'] is not None]
        lateral_maxs = [s['lateral_error_max'] for s in successful_stats if s['lateral_error_max'] is not None]
        lateral_95p = [s['lateral_error_95percentile'] for s in successful_stats if s['lateral_error_95percentile'] is not None]

        heading_means = [s['heading_error_mean'] for s in successful_stats if s['heading_error_mean'] is not None]
        heading_maxs = [s['heading_error_max'] for s in successful_stats if s['heading_error_max'] is not None]
        heading_95p = [s['heading_error_95percentile'] for s in successful_stats if s['heading_error_95percentile'] is not None]

        velocity_means = [s['velocity_mean'] for s in successful_stats if s['velocity_mean'] is not None]
        velocity_maxs = [s['velocity_max'] for s in successful_stats if s['velocity_max'] is not None]

        report.append("## 全体統計 (Overall Statistics)")
        report.append("=" * 80)
        report.append("")

        if lateral_means:
            report.append("### 横偏差 (Lateral Error) [m]")
            report.append(f"  全ファイルの平均横偏差の平均:     {np.mean(lateral_means):.4f} m")
            report.append(f"  全ファイルの平均横偏差の中央値:   {np.median(lateral_means):.4f} m")
            report.append(f"  全ファイルの平均横偏差の標準偏差: {np.std(lateral_means):.4f} m")
            report.append(f"  全ファイル中の最大横偏差:         {np.max(lateral_maxs):.4f} m")
            report.append(f"  全ファイル中の最小横偏差:         {np.min(lateral_maxs):.4f} m")
            report.append(f"  95パーセンタイル平均:             {np.mean(lateral_95p):.4f} m")
            report.append("")

        if heading_means:
            report.append("### ヘディング誤差 (Heading Error) [deg]")
            report.append(f"  全ファイルの平均ヘディング誤差の平均:     {np.mean(heading_means):.4f} deg")
            report.append(f"  全ファイルの平均ヘディング誤差の中央値:   {np.median(heading_means):.4f} deg")
            report.append(f"  全ファイルの平均ヘディング誤差の標準偏差: {np.std(heading_means):.4f} deg")
            report.append(f"  全ファイル中の最大ヘディング誤差:         {np.max(heading_maxs):.4f} deg")
            report.append(f"  全ファイル中の最小ヘディング誤差:         {np.min(heading_maxs):.4f} deg")
            report.append(f"  95パーセンタイル平均:                     {np.mean(heading_95p):.4f} deg")
            report.append("")

        if velocity_means:
            report.append("### 速度 (Velocity) [m/s]")
            report.append(f"  全ファイルの平均速度の平均:   {np.mean(velocity_means):.2f} m/s")
            report.append(f"  全ファイルの平均速度の中央値: {np.median(velocity_means):.2f} m/s")
            report.append(f"  全ファイル中の最大速度:       {np.max(velocity_maxs):.2f} m/s")
            report.append("")

        # Top 10 worst by lateral error
        report.append("=" * 80)
        report.append("## 横偏差が大きいファイル Top 10")
        report.append("=" * 80)
        report.append("")
        sorted_by_lateral = sorted(
            [s for s in successful_stats if s['lateral_error_max'] is not None],
            key=lambda x: x['lateral_error_max'],
            reverse=True
        )[:10]

        for i, s in enumerate(sorted_by_lateral, 1):
            report.append(f"{i:2d}. {s['filename']}")
            report.append(f"    最大横偏差: {s['lateral_error_max']:.4f} m")
            report.append(f"    平均横偏差: {s['lateral_error_mean']:.4f} m")
            if s['velocity_mean']:
                report.append(f"    平均速度:   {s['velocity_mean']:.2f} m/s")
            report.append("")

        # Top 10 worst by heading error
        report.append("=" * 80)
        report.append("## ヘディング誤差が大きいファイル Top 10")
        report.append("=" * 80)
        report.append("")
        sorted_by_heading = sorted(
            [s for s in successful_stats if s['heading_error_max'] is not None],
            key=lambda x: x['heading_error_max'],
            reverse=True
        )[:10]

        for i, s in enumerate(sorted_by_heading, 1):
            report.append(f"{i:2d}. {s['filename']}")
            report.append(f"    最大ヘディング誤差: {s['heading_error_max']:.4f} deg")
            report.append(f"    平均ヘディング誤差: {s['heading_error_mean']:.4f} deg")
            if s['velocity_mean']:
                report.append(f"    平均速度:           {s['velocity_mean']:.2f} m/s")
            report.append("")

    # Metric explanations
    report.append("=" * 80)
    report.append("## 指標の説明 (Metric Definitions)")
    report.append("=" * 80)
    report.append("")
    report.append("### 横偏差 (Lateral Error)")
    report.append("  - 目標軌道からの横方向のずれ [m]")
    report.append("  - 値が小さいほど軌道追従性能が良い")
    report.append("  - 平均値: サンプルポイントでの横偏差の絶対値の平均")
    report.append("  - 最大値: サンプルポイントでの横偏差の絶対値の最大値")
    report.append("  - 95パーセンタイル: 横偏差の絶対値の95%がこの値以下")
    report.append("")
    report.append("### ヘディング誤差 (Heading Error)")
    report.append("  - 目標軌道との車両姿勢角のずれ [deg]")
    report.append("  - 値が小さいほど方向制御性能が良い")
    report.append("  - 平均値: サンプルポイントでのヘディング誤差の絶対値の平均")
    report.append("  - 最大値: サンプルポイントでのヘディング誤差の絶対値の最大値")
    report.append("  - 95パーセンタイル: ヘディング誤差の絶対値の95%がこの値以下")
    report.append("")
    report.append("### 全体統計の意味")
    report.append("  - 「全ファイルの平均〇〇の平均」: 各ファイルごとの平均値を全ファイルで平均")
    report.append("    → 全走行データにおける典型的な性能")
    report.append("  - 「全ファイルの平均〇〇の中央値」: 各ファイルごとの平均値の中央値")
    report.append("    → 外れ値の影響を受けにくい代表値")
    report.append("  - 「全ファイル中の最大〇〇」: 全ファイル・全時刻で最も悪かった瞬間の値")
    report.append("    → 最悪ケースの性能（パラメータチューニングで改善すべき目標）")
    report.append("  - 「95パーセンタイル」: 値の95%がこの値以下")
    report.append("    → 異常値を除いた実用的な最大値")
    report.append("")
    report.append("注: このレポートは20%サンプリングによる高速分析です。")
    report.append("    実際のメッセージの5つに1つをサンプリングして統計を算出しています。")
    report.append("")

    if failed_stats:
        report.append("=" * 80)
        report.append("## 分析失敗ファイル")
        report.append("=" * 80)
        report.append("")
        for s in failed_stats:
            report.append(f"  - {s['filename']}: {s['error']}")
        report.append("")

    # Save files
    json_path = os.path.join(output_dir, 'mpc_batch_analysis.json')
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(all_stats, f, indent=2, ensure_ascii=False)

    txt_path = os.path.join(output_dir, 'mpc_batch_analysis_report.txt')
    with open(txt_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(report))

    print('\n'.join(report))

    return txt_path, json_path

def main():
    if len(sys.argv) < 2:
        print("Usage: python quick_batch_analyze.py <directory>")
        sys.exit(1)

    input_dir = sys.argv[1]
    output_dir = "/home/npc2301030/misc/mpc_analysis"
    os.makedirs(output_dir, exist_ok=True)

    mcap_files = list(Path(input_dir).rglob("*.mcap"))

    if not mcap_files:
        print(f"No MCAP files found in {input_dir}")
        sys.exit(1)

    print(f"Found {len(mcap_files)} MCAP files")
    print(f"Using {cpu_count()} CPU cores")
    print("Starting quick batch analysis (20% sampling)...\n")

    args = [(f, i+1, len(mcap_files)) for i, f in enumerate(mcap_files)]

    all_stats = []
    with Pool(processes=cpu_count()) as pool:
        for idx, total, stats in pool.imap_unordered(process_wrapper, args):
            all_stats.append(stats)
            status = "✓" if stats['success'] else "✗"
            samples = f" ({stats.get('sample_count', 0)} samples)" if stats['success'] else ""
            print(f"[{idx}/{total}] {stats['filename']}: {status}{samples}")

    txt_path, json_path = generate_report(all_stats, output_dir)
    print(f"\nレポート保存:")
    print(f"  テキスト: {txt_path}")
    print(f"  JSON:     {json_path}")

if __name__ == "__main__":
    main()
