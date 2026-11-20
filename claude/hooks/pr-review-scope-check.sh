#!/usr/bin/env bash
# PreToolUse(Bash) hook: before a PR review is posted, show how many files the PR changes
# against its base (base...head on GitHub), so the review's stated scope can be compared
# with what was actually read. It asks the user to confirm; other commands pass through.
#
# Fires on:
#   gh pr review ...                                   (approve / request-changes / comment)
#   gh api ... pulls/<N>/reviews with a write flag     (-X POST, --method POST, --input, -f/-F)
set -uo pipefail

command=$(jq -r '.tool_input.command // empty')
[ -n "$command" ] || exit 0

repo="" pr=""
if [[ "$command" =~ gh[[:space:]]+api[[:space:]].*repos/([^/[:space:]]+/[^/[:space:]]+)/pulls/([0-9]+)/reviews ]]; then
  repo="${BASH_REMATCH[1]}" pr="${BASH_REMATCH[2]}"
  # Only writes: listing reviews with a plain GET must not prompt.
  [[ "$command" =~ (-X[[:space:]]*POST|--method[[:space:]]+POST|--input|[[:space:]]-[fF][[:space:]]|--field|--raw-field) ]] || exit 0
elif [[ "$command" =~ gh[[:space:]]+pr[[:space:]]+review([[:space:]]+([0-9]+))? ]]; then
  pr="${BASH_REMATCH[2]}"
  [[ "$command" =~ (-R|--repo)[[:space:]]+([^[:space:]]+) ]] && repo="${BASH_REMATCH[2]}"
else
  exit 0
fi

args=(pr view)
[ -n "$pr" ] && args+=("$pr")
[ -n "$repo" ] && args+=(--repo "$repo")
info=$(gh "${args[@]}" --json number,baseRefName,headRefOid,changedFiles,files \
  --jq '"#\(.number): \(.changedFiles) files changed against \(.baseRefName) at head \(.headRefOid[0:7])\n" + ([.files[].path] | join("\n"))' 2>&1) \
  || info="could not read the PR from GitHub: $info"

reason="PR review post. Scope check: ${info}

Before posting, confirm that the review read every one of these files in full against the base, not only the commits since the previous review. The scope stated in the review (files read, tests run or skipped) must match this list."

jq -n --arg reason "$reason" '{
  hookSpecificOutput: {
    hookEventName: "PreToolUse",
    permissionDecision: "ask",
    permissionDecisionReason: $reason,
    additionalContext: $reason
  }
}'
