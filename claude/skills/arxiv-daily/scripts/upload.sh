#!/usr/bin/env bash
# Upload <day_dir>/arxiv-daily-<date>.pptx to Drive as Google Slides via rclone.
# Prints the Slides URL. Usage: upload.sh <day_dir>
set -euo pipefail
SKILL_DIR="$(cd "$(dirname "$0")/.." && pwd)"
DAY_DIR="$1"
cfg() { python3 -c "import json,sys;print(json.load(open('$SKILL_DIR/config.json'))['$1'])"; }
REMOTE="$(cfg rclone_remote)"
FOLDER_ID="$(cfg drive_folder_id)"
DATE="$(basename "$DAY_DIR")"
FILE="$DAY_DIR/arxiv-daily-$DATE.pptx"
DEST="${DATE:0:7}"   # monthly subfolder, e.g. 2026-09
FLAGS=(--drive-root-folder-id "$FOLDER_ID" --drive-import-formats pptx)

# replace an earlier upload of the same day (moved to Drive trash, not purged)
rclone delete "${FLAGS[@]}" --drive-use-trash --include "arxiv-daily-$DATE*" "$REMOTE:$DEST" 2>/dev/null || true
rclone copyto "${FLAGS[@]}" "$FILE" "$REMOTE:$DEST/arxiv-daily-$DATE.pptx"
ID="$(rclone lsjson "${FLAGS[@]}" "$REMOTE:$DEST" \
  | python3 -c "import json,sys;print(next(f['ID'] for f in json.load(sys.stdin) if f['Name'].startswith('arxiv-daily-$DATE')))")"
echo "https://docs.google.com/presentation/d/$ID/edit"
