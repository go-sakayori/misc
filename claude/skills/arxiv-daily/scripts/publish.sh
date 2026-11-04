#!/usr/bin/env bash
# Build the deck (with figures) and upload it to Drive. Prints the Slides URL.
# Usage: publish.sh <day_dir>
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
python3 "$DIR/build.py" "$1"
"$DIR/upload.sh" "$1"
