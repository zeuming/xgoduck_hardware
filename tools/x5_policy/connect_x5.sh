#!/bin/sh
set -eu
task_target="${1:-${XGO_X5_TARGET:-root@192.168.1.11}}"
if [ "$#" -gt 1 ]; then
  echo "用法: $0 [SSH用户名@X5地址]" >&2
  exit 2
fi
set -- -T -o ExitOnForwardFailure=yes -o ConnectTimeout=8 \
  -o ServerAliveInterval=10 -o ServerAliveCountMax=3 \
  -L 127.0.0.1:8767:127.0.0.1:8767
if [ -n "${XGO_X5_CONTROL_PATH:-}" ]; then
  set -- "$@" -S "$XGO_X5_CONTROL_PATH" -o BatchMode=yes
fi
echo "网页地址 http://127.0.0.1:8767/。按 Ctrl+C 关闭连接。"
exec ssh "$@" "$task_target" \
  'python3 /root/xgoduck-policy/start_web.py && exec tail -f /dev/null'
