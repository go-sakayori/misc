---
name: arxiv-daily
description: arXivから自動運転(ML-Planner / E2E / データ整備が重点)の論文を20本選んで日本語で要約し、論文の図を入れた固定フォーマットのGoogleスライドにしてDriveへアップロードする。ユーザーが /arxiv-daily と打ったとき、または「今日の論文まとめ」を頼んだときに使う。
---

# arXiv Daily(自動運転)

- スキルの場所:`~/.claude/skills/arxiv-daily/`(`scripts/`、設定は `config.json`、既読IDは `state/seen.json`)
- 出力先:`~/arxiv-daily/<YYYY-MM-DD>/`
- 保存先:Driveの `arXiv Daily/<YYYY-MM>/`(rclone の remote `gdrive`)
- 初回のセットアップは `README.md` を参照(通常の実行では読まない)

## トークン節約のルール(必ず守る)
- `candidates.json` は Read ツールで **1回だけ** 読む。再読しない。Grepで部分的に読まない。
- pptx・画像・base64は読まない。出力もしない。アップロードは必ず rclone で行い、Driveのコネクタは使わない。
- `summaries.json` は1回の Write で書き切る。タイトル・著者・日付・カテゴリは書かない(スクリプトが補完する)。
- Web検索・WebFetch・arXivページの閲覧はしない。アブストラクトだけで要約する。
- 図の取得とレイアウトはスクリプトが行う。Claudeは図を見ない。

## 手順
1. **候補を取得する**:`python3 ~/.claude/skills/arxiv-daily/scripts/fetch.py`
   → 表示された `candidates.json` のパスを Read で1回読む。
   `new` は最新の投稿(重点テーマの語で絞り込み済み)、`related` は今日の関連テーマ(`related_topics`)に沿った過去の論文。既読の論文は除かれている。
2. **20本を選ぶ**(本数は `total`)。目安は `new` から約14本、`related` から約6本。ただし質を優先して配分は調整してよい。
   - 重点テーマ(この3つを最優先):
     1. **ML-Planner**:学習ベースのplanner(imitation / RL / diffusion / Transformer)、closed-loopでの評価(nuPlan / NAVSIM / Bench2Drive など)
     2. **E2E**:end-to-end driving、VLA、world modelを使ったplanning
     3. **それに向けたデータ整備**:data curation / data engine / auto-labeling / scenario mining / long-tail / 合成データ / 新しいデータセット
   - その次に優先するもの:主要機関の研究、手法として新しいもの、実車・公開ベンチマークでの評価
   - perceptionだけの論文(検出・セグメンテーション)は、planningやE2Eとつながるものに限って採用する
   - 下げるもの:自動運転が応用例として一言触れられているだけの論文、サーベイの焼き直し、自動運転以外(マイクロモビリティ・一般的なロボット・LLMセキュリティなど)
   - 並び順は重要度の高い順
3. **`~/arxiv-daily/<date>/summaries.json` を書く**。形式:
   ```json
   {"papers": [{"id": "2609.01234", "tag": "ML-Planner",
     "oneliner": "…", "problem": "…", "method": "…", "result": "…", "why": "…"}]}
   ```
   - 本文はすべて日本語。手法名・データセット名・指標名は原語のまま書く
   - `tag`:ML-Planner / E2E / Data / World Model / VLA / Eval / Prediction のどれか
   - `oneliner`:40字以内。何をして何が良くなったか
   - `problem` / `method` / `result` / `why`:各80字以内(スライドに収まる長さ)
   - `result`:アブストラクトにある数値をそのまま入れる。数値がなければ捏造しない。「(抜粋範囲に数値なし)」と明記し、定性的に書く
   - `why`:自動運転の開発者にとっての意味(実用性・既存手法との違い・制約)
4. **スライドを作ってアップロードする**:`~/.claude/skills/arxiv-daily/scripts/publish.sh ~/arxiv-daily/<date>`
   - 各論文の代表図(overview や pipeline の図を優先し、なければ図1)をarXivのHTML版から取得し、図の形に合わせてレイアウトを選ぶ。縦長の図は右側に、横長の図は下部に全幅で置き、図がなければテキストだけにする
   - 採用した論文のIDを `state/seen.json` に記録する
   - 同じ日付のファイルがDriveにあれば、ゴミ箱に移してから置き換える
   - 最後の行にGoogleスライドのURLが出る
5. **報告は短くする**:スライドのURL、NEW/関連の本数、図が入った本数、上位3本の `oneliner` だけ。

## 失敗したとき
- `fetch.py` がタイムアウトしたら、1回だけ再実行する(arXiv APIは混雑しやすい)。
- `publish.sh` で rclone のエラーが出たら、`rclone config reconnect gdrive:` での再認証をユーザーに案内する。コネクタでのアップロードには切り替えない。
- 要約を直してスライドだけ作り直すときは、`summaries.json` を修正して `publish.sh` を再実行する(fetchは不要)。
- 候補から選び直すときは、`state/seen.json` からその日に採用したIDを消してから `fetch.py` をやり直す(消さないと、その日の論文が既読として候補から外れる)。
