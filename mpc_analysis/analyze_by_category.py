#!/usr/bin/env python3
"""
MPC Analysis categorized by velocity and scene (time-based directory)
"""

import sys
import os
import json
import numpy as np
from pathlib import Path
from collections import defaultdict
from datetime import datetime

def load_analysis_results(json_path):
    """Load previously analyzed results"""
    with open(json_path, 'r', encoding='utf-8') as f:
        return json.load(f)

def categorize_by_velocity(stats):
    """Categorize data by velocity ranges"""
    categories = {
        'stopped': {'range': '0-1 m/s', 'min': 0.0, 'max': 1.0, 'data': []},
        'low_speed': {'range': '1-3 m/s', 'min': 1.0, 'max': 3.0, 'data': []},
        'medium_speed': {'range': '3-6 m/s', 'min': 3.0, 'max': 6.0, 'data': []},
        'high_speed': {'range': '6-10 m/s', 'min': 6.0, 'max': 10.0, 'data': []},
        'very_high_speed': {'range': '10+ m/s', 'min': 10.0, 'max': 999.0, 'data': []},
    }

    for stat in stats:
        if not stat['success'] or stat['velocity_mean'] is None:
            continue

        vel = stat['velocity_mean']
        for cat_name, cat_info in categories.items():
            if cat_info['min'] <= vel < cat_info['max']:
                cat_info['data'].append(stat)
                break

    return categories

def categorize_by_scene(stats):
    """Categorize data by time-based scene (directory name)"""
    scenes = defaultdict(list)

    for stat in stats:
        if not stat['success']:
            continue

        # Extract time from filepath (e.g., "17-57" from path)
        filepath = stat['filepath']
        parts = filepath.split('/')
        for part in parts:
            if '-' in part and len(part) == 5:  # Format: "HH-MM"
                scene_time = part
                scenes[scene_time].append(stat)
                break

    return dict(sorted(scenes.items()))

def calculate_category_stats(data):
    """Calculate statistics for a category"""
    if not data:
        return None

    stats = {
        'count': len(data),
        'lateral_error': {},
        'heading_error': {},
        'velocity': {},
    }

    # Lateral error stats
    lat_means = [d['lateral_error_mean'] for d in data if d['lateral_error_mean'] is not None]
    lat_maxs = [d['lateral_error_max'] for d in data if d['lateral_error_max'] is not None]
    lat_95p = [d['lateral_error_95percentile'] for d in data if d['lateral_error_95percentile'] is not None]

    if lat_means:
        stats['lateral_error'] = {
            'mean_avg': float(np.mean(lat_means)),
            'mean_median': float(np.median(lat_means)),
            'mean_std': float(np.std(lat_means)),
            'max_worst': float(np.max(lat_maxs)),
            'max_best': float(np.min(lat_maxs)),
            'percentile_95_avg': float(np.mean(lat_95p)),
        }

    # Heading error stats
    head_means = [d['heading_error_mean'] for d in data if d['heading_error_mean'] is not None]
    head_maxs = [d['heading_error_max'] for d in data if d['heading_error_max'] is not None]
    head_95p = [d['heading_error_95percentile'] for d in data if d['heading_error_95percentile'] is not None]

    if head_means:
        stats['heading_error'] = {
            'mean_avg': float(np.mean(head_means)),
            'mean_median': float(np.median(head_means)),
            'mean_std': float(np.std(head_means)),
            'max_worst': float(np.max(head_maxs)),
            'max_best': float(np.min(head_maxs)),
            'percentile_95_avg': float(np.mean(head_95p)),
        }

    # Velocity stats
    vel_means = [d['velocity_mean'] for d in data if d['velocity_mean'] is not None]
    vel_maxs = [d['velocity_max'] for d in data if d['velocity_max'] is not None]

    if vel_means:
        stats['velocity'] = {
            'mean_avg': float(np.mean(vel_means)),
            'mean_median': float(np.median(vel_means)),
            'max': float(np.max(vel_maxs)),
        }

    return stats

def generate_categorized_report(all_stats, output_dir):
    """Generate categorized report"""

    successful_stats = [s for s in all_stats if s['success']]

    # Categorize
    vel_categories = categorize_by_velocity(successful_stats)
    scene_categories = categorize_by_scene(successful_stats)

    report = []
    report.append("=" * 100)
    report.append("MPC分析レポート - 速度別・走行シーン別")
    report.append("MPC Analysis Report - By Velocity and Scene")
    report.append("=" * 100)
    report.append(f"生成日時: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    report.append(f"総ファイル数: {len(all_stats)}")
    report.append(f"成功: {len(successful_stats)}")
    report.append("=" * 100)
    report.append("")

    # Velocity-based analysis
    report.append("=" * 100)
    report.append("## 速度別分析 (Analysis by Velocity)")
    report.append("=" * 100)
    report.append("")

    for cat_name, cat_info in vel_categories.items():
        stats = calculate_category_stats(cat_info['data'])
        if not stats:
            continue

        report.append(f"### {cat_info['range']} ({stats['count']} files)")
        report.append("-" * 80)

        if stats['lateral_error']:
            le = stats['lateral_error']
            report.append("横偏差 (Lateral Error):")
            report.append(f"  平均の平均:     {le['mean_avg']:.4f} m")
            report.append(f"  平均の中央値:   {le['mean_median']:.4f} m")
            report.append(f"  最大値(最悪):   {le['max_worst']:.4f} m")
            report.append(f"  最大値(最良):   {le['max_best']:.4f} m")
            report.append(f"  95%tile平均:    {le['percentile_95_avg']:.4f} m")

        if stats['heading_error']:
            he = stats['heading_error']
            report.append("")
            report.append("ヘディング誤差 (Heading Error):")
            report.append(f"  平均の平均:     {he['mean_avg']:.4f} deg")
            report.append(f"  平均の中央値:   {he['mean_median']:.4f} deg")
            report.append(f"  最大値(最悪):   {he['max_worst']:.4f} deg")
            report.append(f"  最大値(最良):   {he['max_best']:.4f} deg")
            report.append(f"  95%tile平均:    {he['percentile_95_avg']:.4f} deg")

        if stats['velocity']:
            v = stats['velocity']
            report.append("")
            report.append("速度 (Velocity):")
            report.append(f"  平均の平均:     {v['mean_avg']:.2f} m/s")
            report.append(f"  最大速度:       {v['max']:.2f} m/s")

        report.append("")

    # Scene-based analysis
    report.append("=" * 100)
    report.append("## 走行シーン別分析 (Analysis by Scene/Time)")
    report.append("=" * 100)
    report.append("")

    for scene_time, scene_data in scene_categories.items():
        stats = calculate_category_stats(scene_data)
        if not stats:
            continue

        report.append(f"### シーン: {scene_time} ({stats['count']} files)")
        report.append("-" * 80)

        if stats['velocity']:
            v = stats['velocity']
            report.append(f"速度: 平均 {v['mean_avg']:.2f} m/s, 最大 {v['max']:.2f} m/s")
            report.append("")

        if stats['lateral_error']:
            le = stats['lateral_error']
            report.append("横偏差:")
            report.append(f"  平均 {le['mean_avg']:.4f} m, 最大 {le['max_worst']:.4f} m, 95%tile {le['percentile_95_avg']:.4f} m")

        if stats['heading_error']:
            he = stats['heading_error']
            report.append("ヘディング誤差:")
            report.append(f"  平均 {he['mean_avg']:.4f} deg, 最大 {he['max_worst']:.4f} deg, 95%tile {he['percentile_95_avg']:.4f} deg")

        report.append("")

    # Velocity vs Error correlation
    report.append("=" * 100)
    report.append("## 速度と誤差の相関分析 (Velocity-Error Correlation)")
    report.append("=" * 100)
    report.append("")

    report.append("速度帯別の性能サマリー:")
    report.append("")
    report.append(f"{'速度帯':<20} {'ファイル数':<10} {'横偏差[m]':<15} {'ヘディング[deg]':<15}")
    report.append("-" * 80)

    for cat_name, cat_info in vel_categories.items():
        stats = calculate_category_stats(cat_info['data'])
        if not stats:
            continue

        lat_err = stats['lateral_error']['mean_avg'] if stats.get('lateral_error') else 0
        head_err = stats['heading_error']['mean_avg'] if stats.get('heading_error') else 0

        report.append(f"{cat_info['range']:<20} {stats['count']:<10} {lat_err:<15.4f} {head_err:<15.4f}")

    report.append("")
    report.append("観察:")
    report.append("  - 速度が上がるにつれて誤差が増加する場合: 動的応答性の問題")
    report.append("  - 低速でも誤差が大きい場合: 軌道生成・予測ロジックの問題")
    report.append("  - 特定シーンで誤差が大きい場合: 環境依存の課題")
    report.append("")

    # Find worst scenes
    report.append("=" * 100)
    report.append("## 最も問題のあるシーン Top 5")
    report.append("=" * 100)
    report.append("")

    scene_stats_list = []
    for scene_time, scene_data in scene_categories.items():
        stats = calculate_category_stats(scene_data)
        if stats and stats.get('heading_error'):
            scene_stats_list.append({
                'scene': scene_time,
                'count': stats['count'],
                'heading_max': stats['heading_error']['max_worst'],
                'lateral_max': stats['lateral_error']['max_worst'],
                'velocity_avg': stats['velocity']['mean_avg'],
            })

    # Sort by heading error
    scene_stats_list.sort(key=lambda x: x['heading_max'], reverse=True)

    report.append("ヘディング誤差が最悪のシーン:")
    for i, scene_stat in enumerate(scene_stats_list[:5], 1):
        report.append(f"{i}. シーン {scene_stat['scene']}: "
                     f"ヘディング最大 {scene_stat['heading_max']:.2f} deg, "
                     f"横偏差最大 {scene_stat['lateral_max']:.2f} m, "
                     f"平均速度 {scene_stat['velocity_avg']:.2f} m/s "
                     f"({scene_stat['count']} files)")

    report.append("")

    # Recommendations
    report.append("=" * 100)
    report.append("## 推奨事項 (Recommendations)")
    report.append("=" * 100)
    report.append("")

    # Analyze velocity correlation
    vel_cat_list = []
    for cat_name, cat_info in vel_categories.items():
        stats = calculate_category_stats(cat_info['data'])
        if stats and stats.get('heading_error'):
            vel_cat_list.append({
                'name': cat_name,
                'range': cat_info['range'],
                'heading_avg': stats['heading_error']['mean_avg'],
                'lateral_avg': stats['lateral_error']['mean_avg'],
            })

    if len(vel_cat_list) >= 2:
        low_speed = vel_cat_list[0]
        high_speed = vel_cat_list[-1]

        heading_ratio = high_speed['heading_avg'] / max(low_speed['heading_avg'], 0.1)

        report.append("### 速度依存性の分析:")
        if heading_ratio > 1.5:
            report.append(f"  ⚠️ 高速時のヘディング誤差が低速時の{heading_ratio:.1f}倍")
            report.append("  → 予測ホライゾンの延長が必要")
            report.append("  → mpc_prediction_horizon: 70-80 を推奨")
            report.append("  → mpc_weight_heading_error_squared_vel の増加")
        elif heading_ratio < 0.8:
            report.append(f"  ✓ 高速時の方がヘディング誤差が小さい")
            report.append("  → 低速時の制御安定性に注意")
            report.append("  → mpc_weight_heading_error の増加")
        else:
            report.append("  ✓ 速度による顕著な差はない")
            report.append("  → 軌道生成ロジック自体の改善が必要")

    report.append("")
    report.append("### シーン別の対策:")
    for scene_stat in scene_stats_list[:3]:
        if scene_stat['heading_max'] > 20.0:
            report.append(f"  ⚠️ シーン {scene_stat['scene']}: ヘディング誤差 {scene_stat['heading_max']:.1f} deg")
            report.append(f"     このシーンでの走行データを詳細分析し、軌道の妥当性を確認してください")

    report.append("")

    # Save reports
    txt_path = os.path.join(output_dir, 'mpc_categorized_report.txt')
    with open(txt_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(report))

    # Save category data as JSON
    category_data = {
        'velocity_categories': {},
        'scene_categories': {},
    }

    for cat_name, cat_info in vel_categories.items():
        stats = calculate_category_stats(cat_info['data'])
        if stats:
            category_data['velocity_categories'][cat_name] = {
                'range': cat_info['range'],
                'stats': stats,
            }

    for scene_time, scene_data in scene_categories.items():
        stats = calculate_category_stats(scene_data)
        if stats:
            category_data['scene_categories'][scene_time] = stats

    json_path = os.path.join(output_dir, 'mpc_categorized_data.json')
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(category_data, f, indent=2, ensure_ascii=False)

    print('\n'.join(report))

    return txt_path, json_path

def main():
    if len(sys.argv) < 2:
        print("Usage: python analyze_by_category.py <json_analysis_file>")
        print("Example: python analyze_by_category.py mpc_batch_analysis.json")
        sys.exit(1)

    json_path = sys.argv[1]
    output_dir = os.path.dirname(json_path)

    if not os.path.exists(json_path):
        print(f"Error: {json_path} not found")
        sys.exit(1)

    print("Loading analysis results...")
    all_stats = load_analysis_results(json_path)

    print("Generating categorized report...")
    txt_path, json_path = generate_categorized_report(all_stats, output_dir)

    print(f"\n{'='*80}")
    print("レポート保存:")
    print(f"  テキスト: {txt_path}")
    print(f"  JSON:     {json_path}")
    print(f"{'='*80}")

if __name__ == "__main__":
    main()
