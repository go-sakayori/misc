# ROS2 Bag Comparison Tool

2つのROS2 bagファイル(.db3または.mcap)を比較するツールです。

## 機能

- .db3と.mcap両方のフォーマットに対応
- 複数トピックの同時比較
- **bag1の全メッセージがbag2に含まれているかをチェック**
- 見つからないメッセージの詳細表示

## 比較ロジック

このツールは、bag1の各メッセージがbag2に含まれているかを確認します。
- bag2にbag1より多くのメッセージがあっても問題ありません
- bag1の全メッセージがbag2に存在すればチェックは成功します

## 必要な依存関係

```bash
pip install pyyaml
```

ROS2環境が必要です：
```bash
source /opt/ros/<your-ros2-distro>/setup.bash
```

## 使い方

### 1. 設定ファイルの編集

`config.yaml`で比較したいトピックを指定します：

```yaml
topics:
  - /camera/image_raw
  - /imu/data
  - /lidar/points

comparison:
  show_differences: true
  max_differences_to_show: 10
```

### 2. 実行

```bash
python3 compare.py <bag1_path> <bag2_path> -c config.yaml
```

例：
```bash
# ディレクトリ形式のbag
python3 compare.py /path/to/bag1 /path/to/bag2

# .mcapファイル
python3 compare.py /path/to/bag1.mcap /path/to/bag2.mcap

# カスタム設定ファイル
python3 compare.py bag1.db3 bag2.db3 -c my_config.yaml
```

### 3. 出力例

```
============================================================
Comparing rosbags:
  Bag 1: /path/to/bag1
  Bag 2: /path/to/bag2
============================================================

Checking topic: /camera/image_raw
----------------------------------------
  Reading from bag 1...
  Reading from bag 2...
  Bag 1: 1000 messages
  Bag 2: 1200 messages
  ✓ All 1000 messages from bag1 are contained in bag2

Checking topic: /imu/data
----------------------------------------
  Reading from bag 1...
  Reading from bag 2...
  Bag 1: 5000 messages
  Bag 2: 4999 messages
  ✗ 100 messages from bag1 NOT found in bag2
    Message #50 from bag1 not in bag2
    Message #51 from bag1 not in bag2
    ...

============================================================
✗ Some messages from bag1 are NOT in bag2
============================================================
```

## オプション

- `-c, --config`: 設定ファイルのパス（デフォルト: config.yaml）

## 終了コード

- `0`: bag1の全メッセージがbag2に含まれている
- `1`: bag1の一部メッセージがbag2に含まれていない、またはエラー発生
