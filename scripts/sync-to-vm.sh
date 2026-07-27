#!/usr/bin/env bash
# Sync the full repo to the Ubuntu VM's ROS2 workspace (~/ads_ws).
# Usage: ./scripts/sync-to-vm.sh [extra rsync args, e.g. --dry-run]
#
# VM IP has changed across restarts before — override with VM_HOST=<ip> if
# the default below is stale.
set -euo pipefail

VM_USER="${VM_USER:-projectspace}"
VM_HOST="${VM_HOST:-192.168.64.5}"
VM_PATH="${VM_PATH:-~/ads_ws}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Pinned to the system binary: a conda/homebrew rsync earlier in PATH has been
# observed to mis-parse the remote destination ("remote file in list of local
# sources") on this machine. /usr/bin/rsync is known-good.
RSYNC_BIN="${RSYNC_BIN:-/usr/bin/rsync}"

"$RSYNC_BIN" -avz \
  --exclude-from="$REPO_ROOT/.gitignore" \
  --exclude '.git/' \
  --exclude '.claude/' \
  "$REPO_ROOT/" "$VM_USER@$VM_HOST:$VM_PATH/" "$@"
