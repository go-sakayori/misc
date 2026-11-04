#!/usr/bin/env bash
# Link this repository's Claude Code config into ~/.claude and install the
# dependencies of the skills. Safe to run again: existing links are left as is,
# and existing files are moved to <name>.bak.<timestamp> before being replaced.
#
# Usage: install.sh [--no-deps]
#   --no-deps  only create the links; skip pip, apt and rclone checks
set -euo pipefail

REPO_DIR="$(cd "$(dirname "$0")" && pwd)"
CLAUDE_DIR="$HOME/.claude"
STAMP="$(date +%Y%m%d-%H%M%S)"
INSTALL_DEPS=1

for arg in "$@"; do
  case "$arg" in
    --no-deps) INSTALL_DEPS=0 ;;
    -h|--help) sed -n '2,8p' "$0"; exit 0 ;;
    *) echo "unknown option: $arg" >&2; exit 1 ;;
  esac
done

# link <source> <target>
link() {
  local src="$1" dst="$2"
  if [ -L "$dst" ] && [ "$(readlink -f "$dst")" = "$(readlink -f "$src")" ]; then
    echo "ok       $dst"
    return
  fi
  if [ -e "$dst" ] || [ -L "$dst" ]; then
    mv "$dst" "$dst.bak.$STAMP"
    echo "backup   $dst -> $dst.bak.$STAMP"
  fi
  ln -s "$src" "$dst"
  echo "linked   $dst -> $src"
}

echo "== Links"
mkdir -p "$CLAUDE_DIR/skills"
link "$REPO_DIR/CLAUDE.md" "$CLAUDE_DIR/CLAUDE.md"
for skill in "$REPO_DIR"/skills/*/; do
  skill="${skill%/}"
  link "$skill" "$CLAUDE_DIR/skills/$(basename "$skill")"
done

if [ "$INSTALL_DEPS" -eq 0 ]; then
  echo "== Dependencies skipped (--no-deps)"
  exit 0
fi

echo "== Python libraries (arxiv-daily)"
if python3 -c "import pptx, PIL" 2>/dev/null; then
  echo "ok       python-pptx, pillow"
else
  python3 -m pip install --user python-pptx pillow
fi

echo "== rclone (arxiv-daily)"
if command -v rclone >/dev/null; then
  echo "ok       $(rclone version | head -1)"
else
  sudo apt-get install -y rclone
fi

REMOTE="$(python3 -c "import json;print(json.load(open('$REPO_DIR/skills/arxiv-daily/config.json'))['rclone_remote'])")"
if rclone listremotes | grep -qx "$REMOTE:"; then
  echo "ok       rclone remote '$REMOTE'"
else
  cat <<EOF

rclone remote '$REMOTE' is not configured yet. Run it yourself (browser login):
  rclone config
    name: $REMOTE / Storage: drive / scope: 1 / root_folder_id: leave empty
  rclone lsd $REMOTE:   # should list folders without errors
EOF
fi

echo
echo "Done. Restart Claude Code to load the new CLAUDE.md and skills."
