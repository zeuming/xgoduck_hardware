#!/bin/sh
set -eu
duck_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
duck_target=${1:-${XGO_X5_TARGET:-root@192.168.1.11}}
if [ "$#" -gt 1 ]; then
  echo "用法: bash tools/duck_sim/connect_live.sh [用户名@X5地址]" >&2
  exit 2
fi
duck_socket_dir=$(mktemp -d /tmp/duck-live-ssh.XXXXXX)
chmod 700 "$duck_socket_dir"
export XGO_X5_CONTROL_PATH="$duck_socket_dir/control"
export XGO_X5_TARGET="$duck_target"
cleanup() {
  ssh -S "$XGO_X5_CONTROL_PATH" -O exit "$duck_target" 2>/dev/null || true
  rmdir "$duck_socket_dir" 2>/dev/null || true
}
trap cleanup EXIT HUP INT TERM
# Normal SSH authentication prompt / agent; no password stored in this script.
ssh -M -S "$XGO_X5_CONTROL_PATH" -o ControlPersist=60 -o ConnectTimeout=8 \
  -o StrictHostKeyChecking=yes -fNT "$duck_target"
echo "打开 http://127.0.0.1:8788/ 的‘实机随动’，点击‘连接实机反馈’。"
bash "$duck_dir/run.sh"
