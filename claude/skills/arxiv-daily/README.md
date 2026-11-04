# arxiv-daily セットアップ

`/arxiv-daily` を実行すると、次の流れでスライドができる。

arXiv APIで候補を取得 → Claudeが20本を選んで要約 → python-pptxで図入りのスライドを作成 → rcloneでDriveにGoogleスライドとしてアップロード

## ファイル構成
| パス | 役割 |
|---|---|
| `SKILL.md` | Claudeへの実行手順 |
| `config.json` | 検索条件・本数・保存先 |
| `scripts/fetch.py` | arXiv APIから新着と関連テーマの候補を取得する。既読は除外する |
| `scripts/figures.py` | arXivのHTML版から代表図を1枚取得する |
| `scripts/build.py` | 要約と図を16:9の固定レイアウトに流し込む(表紙・一覧2枚・論文20枚) |
| `scripts/upload.sh` | rcloneでアップロードし、Googleスライドに変換する |
| `scripts/publish.sh` | build.py と upload.sh をまとめて実行する |
| `state/seen.json` | 採用済みのarXiv ID(重複して載せないため) |

## 初回セットアップ
1. Pythonのライブラリ:`pip install --user python-pptx pillow`
2. rclone:`sudo apt install -y rclone` → `rclone config`
   - name は `gdrive`、Storage は `drive`、scope は `1`(フルアクセス)
   - root_folder_id は空欄でよい(`upload.sh` がフォルダIDを指定する)
3. Driveに保存先のフォルダを作り、そのIDを `config.json` の `drive_folder_id` に入れる
   - 現在の設定:`arXiv Daily`(`19f0j3R21u5SflDonSWcYh56nTAVNc5pf`)
4. 確認:`rclone lsd gdrive:` がエラーにならなければよい

## 設定の変え方(`config.json`)
- `total`:1日に載せる本数
- `core_terms`:自動運転の論文と判定する語(アブストラクトに含むもの)
- `focus_terms`:新着をさらに絞り込む重点テーマの語
- `related_topics`:関連論文の枠のテーマ。毎日2つずつ順番に入れ替わる
- `new_candidates` / `related_candidates` / `abstract_chars`:Claudeが読む量。増やすほどトークンが増える
- `categories`:arXivのカテゴリ

## トークンの目安(1回あたり、推定)
- 候補の読み込み:約1万
- 要約20本の出力:約1〜1.5万
- 図・スライド作成・アップロード:0(スクリプトとrcloneが処理する)

## メモ
- Driveのコネクタでpptxを送る方法は、base64にすると約5.5万トークンになり(実測)、それを出力し直す必要があるため採用しなかった。
- 論文の図の著作権は、論文ごとのライセンスに従う。社外に共有するときは注意する。
