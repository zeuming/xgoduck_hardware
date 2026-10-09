#!/bin/sh
set -eu
# No stored password. SSH prompts normally, or uses the user's existing key.
task_target="${1:-root@192.168.128.10}"
if [ "$#" -gt 1 ]; then
  echo "用法: $0 [SSH用户名@X5地址]" >&2
  exit 2
fi
set -- -T -o ExitOnForwardFailure=yes -o ConnectTimeout=8 \
  -o ServerAliveInterval=10 -o ServerAliveCountMax=3 \
  -L 127.0.0.1:8765:127.0.0.1:8765
if [ -n "${XGO_X5_CONTROL_PATH:-}" ]; then
  set -- "$@" -S "$XGO_X5_CONTROL_PATH" -o BatchMode=yes
fi
echo "连接 X5；网页地址 http://127.0.0.1:8765/。按 Ctrl+C 关闭隧道。"
exec ssh "$@" "$task_target" \
  'python3 /root/xgoduck-servo-workbench/start_x5.py && exec tail -f /dev/null'
