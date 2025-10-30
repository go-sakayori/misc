# MPC Analysis Tools

Autoware MPCコントローラーの性能分析ツール集

## 📁 ファイル構成

### 分析スクリプト（Git管理対象）

1. **`analyze_mpc.py`** - 単一MCAPファイルの詳細分析
2. **`quick_batch_analyze.py`** - 大量MCAPファイルの一括分析（20%サンプリング）
3. **`analyze_by_category.py`** - 速度別・シーン別の分類分析

### 生成されるレポート（Git管理対象外）

- `mpc_analysis.png` - 個別ファイルのグラフ
- `mpc_batch_analysis.json` - 一括分析の生データ
- `mpc_batch_analysis_report.txt` - 一括分析レポート
- `mpc_categorized_data.json` - 分類分析の生データ
- `mpc_categorized_report.txt` - 分類分析レポート

---

## 🚀 使い方

### 1. 単一MCAPファイルの詳細分析

個別のMCAPファイルを詳細分析し、グラフを生成します。

```bash
python3 analyze_mpc.py <mcap_file_path>
```

**例:**
```bash
python3 analyze_mpc.py "/media/npc2301030/Extreme SSD/MPC/18-35/file.mcap"
```

**出力:**
- `mpc_analysis.png` - 4つのグラフ（ステアリング、横偏差、ヘディング誤差、速度）
- コンソールに統計サマリー

**用途:**
- パラメータチューニング後の効果確認
- 問題のあるファイルの詳細調査

---

### 2. 大量MCAPファイルの一括分析

ディレクトリ内の全MCAPファイルを高速分析します（20%サンプリング）。

```bash
python3 quick_batch_analyze.py <mcap_directory>
```

**例:**
```bash
python3 quick_batch_analyze.py "/media/npc2301030/Extreme SSD/MPC/"
```

**出力:**
- `mpc_batch_analysis.json` - 全ファイルの統計データ（JSON）
- `mpc_batch_analysis_report.txt` - テキストレポート
  - 全体統計（横偏差、ヘディング誤差、速度）
  - 最悪ファイル Top 10（横偏差・ヘディング誤差別）
  - 指標の説明

**処理時間:**
- 135ファイル（各5GB）で約20-30分（16コア並列処理）

**用途:**
- 大量の走行データから問題傾向を把握
- パラメータチューニング前のベースライン測定

---

### 3. 速度別・シーン別の分類分析

一括分析の結果を速度帯とシーン（時刻別）で分類して分析します。

```bash
python3 analyze_by_category.py mpc_batch_analysis.json
```

**前提条件:**
- `quick_batch_analyze.py`を先に実行して`mpc_batch_analysis.json`を生成

**出力:**
- `mpc_categorized_data.json` - 分類データ（JSON）
- `mpc_categorized_report.txt` - 分類レポート
  - 速度別分析（0-1, 1-3, 3-6, 6-10, 10+ m/s）
  - シーン別分析（時刻別ディレクトリごと）
  - 速度と誤差の相関分析
  - 問題シーン Top 5
  - パラメータチューニング推奨

**用途:**
- 速度依存性の把握
- 特定シーンでの問題特定
- 的確なパラメータチューニング戦略の策定

---

## 📊 典型的なワークフロー

### ステップ1: ベースライン測定

```bash
# 現在のパラメータで大量データを収集
cd /home/npc2301030/misc/mpc_analysis

# 一括分析（20-30分）
python3 quick_batch_analyze.py "/media/npc2301030/Extreme SSD/MPC/"

# 分類分析（数秒）
python3 analyze_by_category.py mpc_batch_analysis.json
```

### ステップ2: 問題ファイルの詳細分析

```bash
# 分類レポートから問題シーン・ファイルを特定
cat mpc_categorized_report.txt

# 問題ファイルを詳細分析
python3 analyze_mpc.py "/media/npc2301030/Extreme SSD/MPC/17-57/worst_file.mcap"

# グラフを確認
# mpc_analysis.png をビューワーで開く
```

### ステップ3: パラメータチューニング

```bash
# MPCパラメータを編集
# pilot-auto/src/autoware/launcher/autoware_launch/config/control/trajectory_follower/lateral/mpc.param.yaml

# 変更をコミット
cd ~/pilot-auto
git add src/autoware/launcher/autoware_launch/config/control/trajectory_follower/lateral/mpc.param.yaml
git commit -m "tune: MPC parameters for low-speed stability"
```

### ステップ4: 効果確認

```bash
# 新パラメータで走行データを収集
# 再度一括分析
python3 quick_batch_analyze.py "/media/npc2301030/Extreme SSD/MPC_new/"
python3 analyze_by_category.py mpc_batch_analysis.json

# 改善を確認
diff mpc_categorized_report.txt mpc_categorized_report_old.txt
```

---

## 📈 レポートの読み方

### 指標の意味

#### 横偏差 (Lateral Error) [m]
- **定義**: 目標軌道からの横方向のずれ
- **データソース**: `/control/trajectory_follower/lateral/diagnostic` トピックの `data[0]`
- **評価**: 値が小さいほど良い
- **目標値**: 平均 < 0.05m、最大 < 0.20m

#### ヘディング誤差 (Heading Error) [deg]
- **定義**: 目標軌道との車両姿勢角のずれ
- **データソース**: `/control/trajectory_follower/lateral/diagnostic` トピックの `data[1]`（rad→deg変換）
- **評価**: 値が小さいほど良い（自車両中心軌道生成では最重要指標）
- **目標値**: 平均 < 2.0deg、最大 < 10.0deg

#### 速度 (Velocity) [m/s]
- **定義**: 車両の移動速度
- **データソース**: `/localization/kinematic_state` トピック
- **用途**: 速度依存性の分析

### 統計値の意味

- **平均の平均**: 各ファイルの平均値を全ファイルで平均 → **典型的な性能**
- **平均の中央値**: 各ファイルの平均値の中央値 → **外れ値の影響を受けにくい代表値**
- **最大値（最悪）**: 全ファイル中の最大値 → **パラメータチューニングで改善すべき目標**
- **95パーセンタイル**: 値の95%がこの値以下 → **実用的な最大値**

---

## 🔧 依存関係

### 必須Pythonパッケージ

```bash
pip3 install mcap mcap-ros2-support numpy matplotlib
```

### システム要件

- Python 3.8+
- 十分なメモリ（大きなMCAPファイルの場合は8GB以上推奨）
- マルチコアCPU（並列処理用）

---

## 📝 注意事項

### サンプリングについて

`quick_batch_analyze.py`は高速化のため20%サンプリング（5メッセージに1つ）を使用しています。
- **利点**: 処理時間が大幅短縮
- **制約**: 短時間の異常は検出されない可能性
- **推奨**: 問題ファイルは`analyze_mpc.py`で全データ分析

### MCAPファイルサイズ

- 各MCAPファイルは5GB程度と大きいため、処理には時間がかかります
- SSDからの読み込みを推奨（HDDは非常に遅い）

### 並列処理

- `quick_batch_analyze.py`は自動的に全CPUコアを使用
- 他の重い処理と同時実行しない推奨

---

## 🐛 トラブルシューティング

### メモリ不足エラー

```bash
# サンプリング率を下げる（スクリプト内の sample_counter % 5 を % 10 に変更）
# または、ファイルを分割して処理
```

### タイムアウト

```bash
# バックグラウンド実行
nohup python3 quick_batch_analyze.py "/path/to/mcap/" > analysis.log 2>&1 &

# 進捗確認
tail -f analysis.log
```

### ROS2メッセージのデコードエラー

```bash
# mcap-ros2-supportのバージョン確認・更新
pip3 install --upgrade mcap-ros2-support
```

---

## 📄 ライセンス

このツール群はAutowareプロジェクトの一部として開発されました。

## 👤 メンテナー

質問や問題があれば、チーム内で共有してください。
