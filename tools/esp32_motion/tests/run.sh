#!/bin/sh
set -eu
task_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
task_build=$(mktemp -d /tmp/xgoduck-motion-tests.XXXXXX)
c++ -std=c++11 -Wall -Wextra -Werror -fsanitize=address,undefined -g \
  "$task_dir/tests/test_motion.cpp" -o "$task_build/protocol"
"$task_build/protocol"
c++ -std=c++11 -Wall -Wextra -Werror -fsanitize=address,undefined -g \
  -I "$task_dir/tests/mocks" "$task_dir/tests/test_firmware.cpp" -o "$task_build/firmware"
"$task_build/firmware"
c++ -std=c++11 -Wall -Wextra -Werror -fsanitize=address,undefined -g \
  -I "$task_dir/tests/mocks" "$task_dir/tests/test_policy_position.cpp" -o "$task_build/policy-position"
"$task_build/policy-position"
c++ -std=c++11 -Wall -Wextra -Werror -fsanitize=address,undefined -g \
  -I "$task_dir/tests/mocks" "$task_dir/tests/test_timing.cpp" -o "$task_build/timing"
"$task_build/timing"
c++ -std=c++11 -Wall -Wextra -Werror -fsanitize=address,undefined -g \
  -I "$task_dir/tests/mocks" "$task_dir/tests/test_stop_diagnostics.cpp" -o "$task_build/stop-diagnostics"
"$task_build/stop-diagnostics"
c++ -std=c++11 -Wall -Wextra -Werror -fsanitize=address,undefined -g \
  -I "$task_dir/tests/mocks" "$task_dir/tests/test_goal_diagnostics.cpp" -o "$task_build/goal-diagnostics"
"$task_build/goal-diagnostics"
c++ -std=c++11 -Wall -Wextra -Werror -fsanitize=address,undefined -g \
  -I "$task_dir/tests/mocks" "$task_dir/tests/test_off_register_details.cpp" -o "$task_build/off-register-details"
"$task_build/off-register-details"
c++ -std=c++11 -Wall -Wextra -Werror -fsanitize=address,undefined -g \
  -I "$task_dir/tests/mocks" "$task_dir/tests/test_maintenance.cpp" -o "$task_build/maintenance"
XGODUCK_CENTER_TRACE_FIXTURE="$task_build/center.bin" "$task_build/maintenance"
PYTHONPATH="$task_dir/../x5_recovery" python3 -B - "$task_build/center.bin" <<'PY'
import sys
from pathlib import Path
from maintenance_diagnostics import decode_center_failure
d = decode_center_failure(Path(sys.argv[1]).read_bytes(), 31)
assert (d['stage'], d['address'], d['expected'], d['actual']) == (19, 40, 0, 0)
assert d['found'] and d['invalid_reply_seen'] and d['received_count'] == 21
assert not d['received_truncated'] and d['drained_count'] == 0
assert d['received_hex'] == 'ffff1f04022801b1ffff1f0200deffff1f030000dd'
print('PASS firmware CENTER failure ACK decoded by production Python decoder')
PY
