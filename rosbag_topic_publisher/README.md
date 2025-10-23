# Rosbag Multi-Topic Publisher

指定した時刻に最も近い複数のトピックメッセージをrosbagから抽出し、同時にpublishするツール。

## 必要要件

- ROS2 Humble
- Python 3.10+
- rosbag2_py
- PyYAML

## セットアップ

1. ROS2ワークスペースをsource（必須）:
```bash
source ~/pilot-auto/install/setup.bash
```

2. PyYAMLをインストール（必要な場合）:
```bash
pip install pyyaml
```

## 使い方

### 基本的な使用方法

```bash
cd ~/misc/rosbag_topic_publisher
python3 publish_topics.py /path/to/rosbag.mcap 1761113345.458
```

### コマンドラインオプション

```bash
# 基本的な使い方（デフォルトのconfig.yamlを使用）
python3 publish_topics.py /path/to/rosbag.mcap 1761113345.458

# 別の設定ファイルを指定
python3 publish_topics.py /path/to/rosbag.mcap 1761113345.458 --config my_config.yaml

# テスト実行（publishしない）
python3 publish_topics.py /path/to/rosbag.mcap 1761113345.458 --no-publish
```

## 設定ファイル（config.yaml）

```yaml
# Publishするトピックのリスト
topics:
  # source_topic: rosbagから抽出するトピック名
  # publish_topic: publishする際のトピック名（省略可、デフォルトはsource_topicと同じ）
  - source_topic: /planning/generator/diffusion_planner/candidate_trajectories
    publish_topic: /planning/diffusion_planner/candidate_trajectories

  - source_topic: /planning/scenario_planning/lane_driving/behavior_planning/path
    publish_topic: /planning/scenario_planning/lane_driving/behavior_planning/path

  - source_topic: /localization/kinematic_state
    publish_topic: /localization/kinematic_state

# Publishオプション
publish_options:
  # 各メッセージを何回publishするか（信頼性向上のため）
  num_publishes: 5

  # publish間隔（秒）
  publish_interval: 0.1

  # subscriber接続待ち時間（秒）
  connection_wait_time: 1.0
```

## 設定ファイルの詳細

### トピック設定

各トピックには以下のフィールドを指定できます：

- `source_topic` (必須): rosbagから抽出するトピック名
- `publish_topic` (任意): publishする際のトピック名。省略した場合は`source_topic`と同じ名前でpublishされます

### Publishオプション

- `num_publishes`: 各メッセージを繰り返しpublishする回数（デフォルト: 5）
  - ROS2のpublisher/subscriber接続は時間がかかるため、複数回送信することで確実に配信

- `publish_interval`: publish間の待機時間（秒、デフォルト: 0.1）
  - メッセージ送信の間隔

- `connection_wait_time`: subscriber接続待機時間（秒、デフォルト: 1.0）
  - publisher起動後、subscriberが接続するまでの待機時間

## 使用例

### 例1: 複数トピックを同時にpublish

```bash
# 設定ファイルで指定した複数トピックを一度にpublish
source ~/pilot-auto/install/setup.bash
cd ~/misc/rosbag_topic_publisher
python3 publish_topics.py ~/pilot-auto/rosbag.mcap 1761113345.458
```

別のターミナルで受信：
```bash
source ~/pilot-auto/install/setup.bash
ros2 topic echo /planning/diffusion_planner/candidate_trajectories
```

### 例2: 異なるタイムスタンプで実行

```bash
# 異なるタイムスタンプを指定
python3 publish_topics.py ~/pilot-auto/rosbag.mcap 1761113350.0
```

### 例3: テスト実行（publishせずにメッセージ検索のみ）

```bash
# メッセージが見つかるか確認
python3 publish_topics.py ~/pilot-auto/rosbag.mcap 1761113345.458 --no-publish
```

### 例4: カスタム設定ファイルを使用

```bash
# 別の設定ファイルを指定
python3 publish_topics.py ~/pilot-auto/rosbag.mcap 1761113345.458 --config my_topics.yaml
```

## トラブルシューティング

### エラー: "No module named 'autoware_internal_planning_msgs'"

**原因**: ROS2ワークスペースがsourceされていない

**解決策**:
```bash
source ~/pilot-auto/install/setup.bash
python3 publish_topics.py --config config.yaml
```

### エラー: "Rosbag file not found"

**原因**: 設定ファイルのrosbag_pathが正しくない

**解決策**: config.yamlの`rosbag_path`を正しいパスに修正

### エラー: "Topic 'XXX' not found in rosbag"

**原因**: 指定したトピックがrosbagに存在しない

**解決策**:
```bash
# rosbag内のトピックを確認
ros2 bag info <rosbag_path>
```

## ファイル構成

```
rosbag_topic_publisher/
├── publish_topics.py    # メインスクリプト
├── config.yaml          # 設定ファイル例
└── README.md           # このファイル
```

## 機能

- ✅ 複数トピックの同時publish
- ✅ YAML設定ファイルによる管理
- ✅ トピック名の変更（source_topic → publish_topic）
- ✅ タイムスタンプの柔軟な指定（秒/ナノ秒）
- ✅ 信頼性の高いpublish（複数回送信）
- ✅ .db3 / .mcap 両対応

## ライセンス

MIT License
