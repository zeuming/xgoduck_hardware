#!/usr/bin/env python3
"""RAM-only, supported-robot calibration page; ARM and TARGET are forbidden.

This adapter reuses the pinned model supervisor without modifying its live
source. Storage evidence and this separate calibration capability are both
required. The specified serial port is used only for STOP, identity, feedback
and volatile CONFIG. Calibration snapshots persist only on the Mac.
"""

import argparse
import hashlib
import inspect
import json
import os
from pathlib import Path, PurePosixPath
import re
import stat
import sys
import tempfile
import time
import uuid

import run_ram_model_web as base
import calibration_restore
import imu_wizard
import imu_wizard_ui

BASE_SHA256 = "5a57a9151e2d397f98aba440969a021ed7a03d60fb5dfdb72173f3ccf2304903"
ARCHIVE_SHA256 = "032e0ab8b8fc2bc5c6e5e6d819c7de9819196313b9787b1b464a691f7347597e"
ARCHIVE_BYTES = 39209311
SCOPE = "ram_motion_calibration_only"
MEMBERS = frozenset(base.ALLOWED_MEMBERS | {"policy/motion.py"})
MODEL_RECORDER = base.Recorder
RESTORE_RECORD = None


def validate_serial_review(device):
    if (not isinstance(device, dict) or set(device) != {"path", "vid", "pid", "serial_number", "syspath"}
            or device["path"] != "/dev/ttyUSB0"
            or not re.fullmatch(r"[0-9a-f]{4}", device["vid"])
            or not re.fullmatch(r"[0-9a-f]{4}", device["pid"])
            or not isinstance(device["serial_number"], str) or len(device["serial_number"]) > 128):
        raise ValueError("A complete, reviewed ttyUSB0 USB identity is required")
    path = device["syspath"]
    if (not isinstance(path, str) or not path.startswith("/sys/devices/")
            or str(PurePosixPath(path)) != path
            or any(part in (".", "..") for part in path.split("/"))):
        raise ValueError("Reviewed USB syspath must be a canonical /sys/devices path")


def validate_review(review):
    if (review.get("scope") != "ram_motion_calibration_only"
            or review.get("physical_support_confirmed") is not True
            or review.get("servo_arm_authorized") is not False
            or review.get("servo_motion_authorized_by_this_review") is not False):
        raise ValueError("A separate supported, calibration-only capability is required")
    if str(uuid.UUID(review["expected_boot_id"])) != review["expected_boot_id"]:
        raise ValueError("Invalid reviewed boot ID")
    if not re.fullmatch(r"[0-9a-f]{32}", review["expected_cid"]):
        raise ValueError("Invalid reviewed card CID")
    root = review["expected_root"]
    if (set(root) != {"device", "source", "filesystem"} or root["device"] != "179:2"
            or root["filesystem"] != "ext4" or root["source"] not in ("/dev/root", "/dev/mmcblk1p2")):
        raise ValueError("Unexpected reviewed root identity")
    if review.get("required_unmounted_device") != "179:1":
        raise ValueError("CONFIG must remain unmounted")
    if not re.fullmatch(r"[0-9a-f]{64}", review["kernel_log_sha256"]):
        raise ValueError("Invalid reviewed kernel hash")
    validate_serial_review(review["serial_device"])


def serial_device_proof(expected, opened_fd=None):
    """Read-only identity proof; USB ancestor is pinned even without a serial."""
    validate_serial_review(expected)
    node = os.stat(expected["path"], follow_symlinks=False)
    if not stat.S_ISCHR(node.st_mode) or os.major(node.st_rdev) != 188 or os.minor(node.st_rdev) != 0:
        raise RuntimeError("Reviewed ttyUSB0 is not the expected character node")
    tty = Path("/sys/class/tty/ttyUSB0/device").resolve(strict=True)
    usb = None
    for parent in (tty, *tty.parents):
        if (parent / "idVendor").is_file() and (parent / "idProduct").is_file():
            usb = parent
            break
    if usb is None or str(usb) != expected["syspath"]:
        raise RuntimeError("USB physical path changed")
    actual = {"path": expected["path"], "syspath": str(usb),
              "vid": (usb / "idVendor").read_text().strip().lower(),
              "pid": (usb / "idProduct").read_text().strip().lower(),
              "serial_number": (usb / "serial").read_text().strip() if (usb / "serial").is_file() else ""}
    if actual != expected:
        raise RuntimeError("USB VID/PID/serial identity changed")
    if opened_fd is not None:
        opened = os.fstat(opened_fd)
        if (not stat.S_ISCHR(opened.st_mode) or opened.st_rdev != node.st_rdev
                or opened.st_dev != node.st_dev or opened.st_ino != node.st_ino):
            raise RuntimeError("Opened serial descriptor differs from the reviewed node")
    return (node.st_dev, node.st_ino, node.st_rdev)


def check_guard(path, review, now=None):
    status = base.read_regular_json(path, private=True)
    if (status.get("safe") is not False or status.get("safe_for_ram_model_web") is not False
            or status.get("safe_for_ram_motion_calibration") is not True
            or status.get("capability_scope") != SCOPE
            or status.get("physical_support_confirmed") is not True
            or status.get("servo_arm_authorized") is not False
            or status.get("serial_device") != review["serial_device"]
            or status.get("requires_serial_identity_proof") is not True
            or status.get("serial_identity_runtime_verified_by_guard") is not False):
        raise RuntimeError("The live guard does not grant this separate calibration scope")
    pid = status.get("local_pid")
    if type(pid) is not int or pid <= 1 or pid == os.getpid():
        raise RuntimeError("Invalid storage guard PID")
    os.kill(pid, 0)
    heartbeat = status.get("last_heartbeat_monotonic")
    if isinstance(heartbeat, bool) or not isinstance(heartbeat, (int, float)):
        raise RuntimeError("Missing storage guard heartbeat")
    age = (time.monotonic() if now is None else now) - heartbeat
    if not 0 <= age <= 4:
        raise RuntimeError("Storage guard heartbeat is stale")
    if (status.get("boot_id") != review["expected_boot_id"] or status.get("card_cid") != review["expected_cid"]
            or status.get("kernel_log_sha256") != review["kernel_log_sha256"]
            or {key: status.get("root", {}).get(key) for key in ("device", "source", "filesystem")} != review["expected_root"]):
        raise RuntimeError("Guard identity/kernel evidence differs from this review")
    return status


# SIGTERM allows a surviving web process to send STOP if its supervisor dies.
# Native import preflight retains the model runner's SIGKILL parent protection.
MOTION_PARENT_GUARD = base.CHILD_PARENT_GUARD.replace("signal.SIGKILL", "signal.SIGTERM")

MOTION_WRAPPER = MOTION_PARENT_GUARD + r'''
import inspect, json, sys, threading, time
from pathlib import Path
expected = json.loads(os.environ["XGODUCK_SERIAL_REVIEW"])
exit_requested = False
closing = False
shutdown_lock = threading.Lock()
shutdown_done = threading.Event()
shutdown_proof = {"stop_ack": False, "release_confirmed": False, "release_status": "unknown", "serial_closed": False, "reason": "Not yet stopped"}
opened_transports = []
server = None
controller = None
def request_exit(signum, frame):
    global exit_requested
    exit_requested = True
signal.signal(signal.SIGTERM, request_exit)
signal.signal(signal.SIGHUP, request_exit)
sys.path.insert(0, os.environ["XGODUCK_POLICY_DIR"])
def audit(event, args):
    if event == "open" and args and isinstance(args[0], (str, bytes)):
        path = os.fsdecode(args[0])
        if path.startswith(("/dev/tty", "/dev/serial", "/dev/bus/usb")) and path != expected["path"]:
            raise PermissionError("Only the reviewed feedback serial port is allowed")
        mode = args[1] if len(args) > 1 else None
        flags = args[2] if len(args) > 2 else 0
        writes = (isinstance(mode, str) and any(c in mode for c in "wax+")) or (isinstance(flags, int) and flags & (os.O_WRONLY | os.O_RDWR | os.O_CREAT | os.O_TRUNC | os.O_APPEND))
        if writes and path != expected["path"] and not os.path.abspath(path).startswith(os.environ["XGODUCK_RAM_ROOT"] + "/"):
            raise PermissionError("Calibration file writes are restricted to temporary RAM")
sys.addaudithook(audit)
import web_app, motion
__IMU_WIZARD_RUNTIME__
__CALIBRATION_RESTORE_RUNTIME__
def release_state_ok(state):
    return (state["online_mask"] == motion.ALL_MASK and state["fault_mask"] == 0
            and state["torque_mask"] == 0 and state["armed"] is False)
class FeedbackTransport(motion.SerialTransport):
    def open(self):
        if controller is not None and getattr(controller, '_feedback_fault', None) is not None:
            raise motion.MotionError('反馈故障已锁定，本次会话禁止重新打开串口')
        serial_device_proof(expected)
        opened_transports.append(self)
        super().open()
        try:
            serial_device_proof(expected, self.serial.fileno())
            # A CH340 open may reset this ESP32 despite preset DTR/RTS. Open
            # once, allow a bounded boot interval, then let _connect_once send
            # its first STOP. No ARM/TARGET or uncertain reconnect is allowed.
            deadline = time.monotonic() + .75
            while time.monotonic() < deadline:
                if exit_requested:
                    raise RuntimeError('Calibration startup was stopped during ESP32 settling')
                serial_device_proof(expected, self.serial.fileno())
                time.sleep(min(.025, max(0., deadline - time.monotonic())))
        except BaseException:
            self.close()
            raise
    def request(self, kind, payload=b''):
        if (controller is not None and getattr(controller, '_feedback_fault', None) is not None
                and getattr(controller, '_release_proof', None) is not None):
            raise motion.MotionError('反馈故障后的串口已关闭，本次会话不再发送请求')
        if kind not in (motion.STOP, motion.HELLO, motion.STATE, motion.CONFIG):
            raise motion.MotionError("本次仅允许校准与反馈；舵机使能和目标发送被禁止")
        serial_device_proof(expected, self.serial.fileno() if self.serial is not None else None)
        return super().request(kind, payload)
class CalibrationController(ImuWizardMixin, motion.MotionController):
    def _require_mutable_session_locked(self):
        if getattr(self, '_feedback_fault', None) is not None:
            raise motion.MotionError('反馈故障已锁定，本页只读；检查连接后需重新开启校准会话')
    def _require_off_locked(self):
        self._require_mutable_session_locked()
        return super()._require_off_locked()
    def _connect_once(self):
        with self._lock:
            self._require_mutable_session_locked()
            if self._closed.is_set():
                raise motion.MotionError('已关闭的校准会话禁止重新连接')
        return super()._connect_once()
    def start(self):
        with self._lock:
            self._require_mutable_session_locked()
            if self._closed.is_set():
                raise motion.MotionError('已关闭的校准会话禁止重启反馈线程')
        return super().start()
    def _require_feedback_locked(self):
        super()._require_feedback_locked()
        if self._closed.is_set() or not release_state_ok(self._state):
            raise motion.MotionError('校准反馈必须全部在线、无故障且扭矩关闭')
    def status(self):
        with self._lock:
            result = super().status()
            result['imu_feedback_ok'] = self._state is not None and self._state['imu_ok'] is True
            fault = getattr(self, '_feedback_fault', None)
            proof = getattr(self, '_release_proof', None) or {}
            stopped = self._thread is None or not self._thread.is_alive()
            serial_closed = (self._transport is None and self._connected is False
                             and all(t.serial is None for t in opened_transports)
                             and proof.get('serial_closed') is True)
            result.update(paused=fault is not None,
                          read_only=fault is not None and stopped and serial_closed,
                          pause_reason=fault['error'] if fault is not None else None,
                          pause_feedback=motion.copy.deepcopy(fault['feedback']) if fault is not None else None,
                          pause_release_status=proof.get('release_status', 'unknown') if fault is not None else None,
                          pause_release_confirmed=proof.get('release_confirmed') is True if fault is not None else False,
                          pause_worker_stopped=stopped, pause_serial_closed=serial_closed)
            if fault is not None:
                result['error'] = fault['error']
            return result
    def _latch_feedback_fault_locked(self, error):
        if getattr(self, '_feedback_fault', None) is None:
            self._feedback_fault = {'error': str(error),
                                    'feedback': motion.copy.deepcopy(self._feedback_diagnostic_locked())}
            print('MOTION_FAULT ' + json.dumps(self._feedback_fault, ensure_ascii=False, allow_nan=False), flush=True)
    def stop(self):
        with self._lock:
            if getattr(self, '_feedback_fault', None) is not None:
                self._mode, self._pending_stop = 'off', False
                self._imu_wizard_cancel_locked('反馈采样已停止；本页只读，已保存校准保持不变')
                return self.status()
            return super().stop()
    def heartbeat(self):
        with self._lock:
            if getattr(self, '_feedback_fault', None) is not None:
                return self.status()
            return super().heartbeat()
    def imu_wizard_action(self, action, expected_generation, *, support_confirmed=False, mounted_confirmed=False):
        with self._lock:
            if getattr(self, '_feedback_fault', None) is not None:
                if action != 'cancel':
                    self._require_mutable_session_locked()
                self._imu_wizard_cancel_locked('反馈采样已停止；本页只读，已保存校准保持不变')
                return self.status()
            return super().imu_wizard_action(action, expected_generation,
                support_confirmed=support_confirmed, mounted_confirmed=mounted_confirmed)
    def capture_all_zeros(self, confirmed, expected_generation, *, support_confirmed=False):
        if support_confirmed is not True:
            raise motion.MotionError('请先确认机身由稳固垫块或支架承重、双脚悬空')
        if confirmed is not True:
            raise motion.MotionError('请先勾选已按参考摆好全部关节零姿态、嘴巴闭合')
        with self._lock:
            self._require_no_imu_wizard_locked()
            if self._closed.is_set() or type(expected_generation) is not int or expected_generation != self._generation:
                raise motion.MotionError('校准会话已变化，请刷新真机状态后重试')
            self._require_off_locked()
            self._require_feedback_locked()
            if not release_state_ok(self._state):
                raise motion.MotionError('采集要求 15 个舵机全部在线、无故障且全部扭矩关闭')
            if not self._state['imu_ok'] or self._imu_age_locked() >= motion.FRESH_SECONDS:
                raise motion.MotionError('IMU 反馈尚未就绪或已过期')
            # All 15 positions come from this one accepted STATE. Neither a
            # serial request nor a worker update can split this transaction.
            positions = list(self._state['positions'])
            if len(positions) != 15 or any(type(value) is not int or not 0 <= value <= 4095 for value in positions):
                raise motion.MotionError('关节编码器反馈无效')
            previous, dirty = self._config, self._config_dirty
            updated = motion.copy.deepcopy(previous)
            for joint, value in zip(updated['joints'], positions):
                joint['zero'] = value
            updated['mechanical_reference_confirmed'] = True
            self._config, self._config_dirty = updated, True
            try:
                self._save_locked()
            except BaseException:
                self._config, self._config_dirty = previous, dirty
                raise
            self._filter = motion.ImuFilter()
            self._filtered = None
        return self.status()
    def _feedback_diagnostic_locked(self):
        state = self._state
        age, imu_age = self._age_locked(), self._imu_age_locked()
        return {**{key: state.get(key) if state is not None else None for key in
                   ('online_mask', 'fault_mask', 'torque_mask', 'armed', 'sample_us', 'imu_us')},
                'age_ms': round(age * 1000, 3) if motion.math.isfinite(age) else None,
                'imu_age_ms': round(imu_age * 1000, 3) if motion.math.isfinite(imu_age) else None}
    def _finalize_serial_locked(self, reason):
        # _lock owns the one proof; _io_lock keeps STOP, its following STATE
        # and disconnect indivisible with respect to the feedback worker.
        if getattr(self, '_release_proof', None) is not None:
            return
        self._closed.set()
        self._stop_locked(reason)
        proof = {'stop_ack': False, 'release_confirmed': False, 'release_status': 'unknown',
                 'serial_closed': False, 'reason': 'No connected transport for STOP/STATE confirmation'}
        with self._io_lock:
            try:
                if self._connected and self._transport is not None:
                    previous = self._state['sample_us'] if self._state else None
                    self._request(motion.STOP)
                    proof['stop_ack'] = True
                    self._pending_stop = False
                    time.sleep(.04)
                    state = motion.decode_state(self._request(motion.STATE))
                    self._accept_state_locked(state)
                    delta = None if previous is None else (state['sample_us'] - previous) & 0xffffffff
                    fresh = delta is not None and 0 < delta < 0x80000000 and self._age_locked() < motion.FRESH_SECONDS
                    proof['feedback'] = self._feedback_diagnostic_locked()
                    proof['release_confirmed'] = release_state_ok(state) and fresh
                    proof['release_status'] = 'confirmed' if proof['release_confirmed'] else 'unknown'
                    proof['reason'] = None if proof['release_confirmed'] else 'Release feedback was incomplete or not fresh'
            except BaseException as error:
                proof['reason'] = str(error)
            finally:
                try:
                    super()._disconnect(reason)
                except BaseException as error:
                    proof['release_confirmed'] = False
                    proof['release_status'] = 'unknown'
                    proof['reason'] = 'Serial close failed: ' + str(error)
        proof['serial_closed'] = all(t.serial is None for t in opened_transports)
        self._pending_stop = False
        fault = getattr(self, '_feedback_fault', None)
        if fault is not None:
            self._error = fault['error']
        self._release_proof = proof
        shutdown_proof.clear()
        shutdown_proof.update(proof)
    def _disconnect(self, reason):
        with self._lock:
            self._finalize_serial_locked(reason)
    def _save_locked(self):
        self._require_mutable_session_locked()
        super()._save_locked()
        print("CALIBRATION_SNAPSHOT " + json.dumps(self._config, ensure_ascii=False, allow_nan=False), flush=True)
    def start_mode(self, mode, support_confirmed=False, *, expected_generation=None):
        if mode != 'shadow':
            raise motion.MotionError('本次不开放站姿或行走')
        with self._lock:
            self._require_mutable_session_locked()
            self._require_no_imu_wizard_locked()
            if not self._calibrated_locked():
                raise motion.MotionError('完成关节、机械参考和 IMU 校准后才能进行真实反馈推理')
        return super().start_mode(mode, support_confirmed=support_confirmed, expected_generation=expected_generation)
    def command(self, *args):
        raise motion.MotionError('本次不发送速度或舵机目标')
    def _cycle(self):
        with self._lock:
            if self._closed.is_set():
                return
            self._service_stop_locked()
            state = motion.decode_state(self._request(motion.STATE))
            self._accept_state_locked(state)
            if not release_state_ok(state):
                error = motion.MotionError('舵机反馈不完整、有故障或扭矩未释放；本次会话已停止')
                self._latch_feedback_fault_locked(error)
                self._finalize_serial_locked(str(error))
                raise error
            self._require_feedback_locked()
            if self._mode == 'off':
                self._apply_config_locked()
                return
            if self._mode != 'shadow':
                raise motion.MotionError('校准会话不允许动作模式')
            if self._heartbeat is None or self._clock() - self._heartbeat >= motion.HEARTBEAT_SECONDS:
                self._stop_locked('网页反馈心跳超时')
            self._service_stop_locked()
        self._control_once()
    def _run(self):
        global exit_requested
        try:
            while not self._closed.is_set():
                started = self._clock()
                if not self._connected:
                    raise motion.MotionError('串口失联；本次会话不自动重新连接')
                self._cycle()
                self._closed.wait(max(.001, motion.PERIOD - (self._clock() - started)))
        except BaseException as error:
            with self._lock:
                self._latch_feedback_fault_locked(error)
                self._finalize_serial_locked(str(error))
            self._closed.set()
        finally:
            self._disconnect('校准反馈线程已停止')
    def close(self):
        self._closed.set()
        with self._lock:
            self._finalize_serial_locked('校准会话已关闭')
        # Do not join while holding either serial/controller lock: the worker
        # may be finishing the same idempotent shutdown path.
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=1)
        with self._lock:
            shutdown_proof['worker_stopped'] = self._thread is None or not self._thread.is_alive()
def shutdown():
    global closing
    with shutdown_lock:
        already_closing = closing
        closing = True
    if already_closing:
        shutdown_done.wait(timeout=2)
        return
    try:
        if controller is not None:
            controller.close()
        # Handle a connection interrupted before MotionController publishes it.
        for transport in opened_transports:
            if transport.serial is not None:
                try:
                    # A latched worker fault already attempted its one STOP /
                    # STATE proof and disconnected. Never restart serial I/O
                    # after that worker has stopped, even on failed close.
                    if controller is None or getattr(controller, '_feedback_fault', None) is None:
                        transport.request(motion.STOP)
                        shutdown_proof['stop_ack'] = True
                except BaseException as error:
                    shutdown_proof['reason'] = str(error)
                finally:
                    transport.close()
        shutdown_proof['serial_closed'] = all(t.serial is None for t in opened_transports)
        print('MOTION_SHUTDOWN ' + json.dumps(shutdown_proof, ensure_ascii=False), flush=True)
    finally:
        shutdown_done.set()
def feedback_pause_status():
    # Joining outside the controller lock permits an in-flight final STOP /
    # STATE / close to finish. A live worker or unclosed serial cannot keep a
    # web page alive; this bounded proof never reopens or requests the port.
    worker = controller._thread
    if worker is not None and worker is not threading.current_thread():
        worker.join(timeout=1)
    with controller._lock:
        result = controller.status()
        if (result['paused'] is not True or result['read_only'] is not True
                or not controller._closed.is_set()):
            raise RuntimeError('Feedback pause lacks proof of a stopped worker and closed serial')
        shutdown_proof['worker_stopped'] = True
        return result
def supervise_page():
    global exit_requested
    pause_announced = False
    while not exit_requested:
        try:
            if getattr(controller, '_feedback_fault', None) is not None:
                paused = feedback_pause_status()
                if not pause_announced:
                    print('MOTION_PAUSED ' + json.dumps({key: paused[key] for key in
                          ('pause_reason', 'pause_feedback', 'pause_release_status',
                           'pause_release_confirmed', 'pause_worker_stopped', 'pause_serial_closed')},
                          ensure_ascii=False, allow_nan=False), flush=True)
                    pause_announced = True
            else:
                serial_device_proof(expected)
                if controller._closed.is_set():
                    raise RuntimeError('Feedback controller stopped')
        except BaseException as error:
            print('MOTION_FAULT ' + str(error), flush=True)
            exit_requested = True
            break
        time.sleep(.25)
    try:
        shutdown()
    finally:
        server.shutdown()
def calibration_handler(workbench):
    # Extend only the pinned route whitelist. The original host/origin/token,
    # JSON/length checks and Workbench lock continue to gate this POST.
    source = inspect.getsource(web_app.make_handler)
    routes = "'/api/motion/stop','/api/motion/heartbeat','/api/motion/capture',"
    if source.count(routes) != 1:
        raise RuntimeError('Reviewed HTTP route whitelist changed')
    source = source.replace(routes, routes + "'/api/motion/capture-all-zeros',", 1)
    source = source.replace("'/api/motion/capture-all-zeros',", "'/api/motion/capture-all-zeros','/api/motion/imu-wizard',", 1)
    stop_branch = "if self.path in ('/api/motion/stop','/api/motion/heartbeat'):"
    if source.count(stop_branch) != 1:
        raise RuntimeError('Reviewed cancellation route gate changed')
    source = source.replace(stop_branch, stop_branch[:-1] + " or (self.path == '/api/motion/imu-wizard' and body.get('action') == 'cancel'):", 1)
    namespace = dict(web_app.__dict__)
    exec(compile(source, '<calibration-http-route>', 'exec'), namespace)
    return namespace['make_handler'](workbench)
try:
    workbench = web_app.Workbench()
    if workbench.motion is not None or len(workbench.sessions) != 4:
        raise RuntimeError('Model preflight failed: ' + repr(workbench.errors))
    calibration_path = Path(os.environ['XGODUCK_POLICY_DIR']) / 'calibration.json'
    if calibration_path.exists():
        raise RuntimeError('A fresh calibration session must not inherit a previous file')
    controller = CalibrationController(workbench.sessions, config_path=calibration_path,
                                       port=expected['path'], transport_factory=FeedbackTransport)
    controller._connect_once()
    if not release_state_ok(controller._state):
        raise RuntimeError('Initial feedback must confirm 15 online, no faults and all torque released')
    with controller._lock:
        controller._require_feedback_locked()
        if not controller._state['imu_ok'] or controller._imu_age_locked() >= motion.FRESH_SECONDS:
            raise RuntimeError('Initial IMU feedback must be fresh')
        restore = json.loads(os.environ.get('XGODUCK_CALIBRATION_RESTORE', 'null'), object_pairs_hook=_restore_json_pairs)
        if restore is not None:
            restored = validate_restore_record(restore,
                boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                cid=Path('/sys/block/mmcblk1/device/cid').read_text().strip(), serial_device=expected)
            previous, dirty = controller._config, controller._config_dirty
            controller._config, controller._config_dirty = motion.copy.deepcopy(restored), True
            try:
                controller._save_locked()
            except BaseException:
                controller._config, controller._config_dirty = previous, dirty
                raise
            print('CALIBRATION_RESTORE_VERIFIED ' + restore['sha256'], flush=True)
    if exit_requested:
        raise RuntimeError('Calibration startup interrupted')
    workbench.motion = controller
    original_action = workbench.motion_action
    def calibration_action(path, body):
        if closing and path != '/api/motion/stop':
            raise ValueError('校准会话正在停止')
        with controller._lock:
            if getattr(controller, '_feedback_fault', None) is not None:
                if path == '/api/motion/stop':
                    return controller.stop()
                if path == '/api/motion/heartbeat':
                    return controller.heartbeat()
                if not (path == '/api/motion/imu-wizard' and body.get('action') == 'cancel'):
                    controller._require_mutable_session_locked()
        if path == '/api/motion/imu-wizard':
            if (set(body) != {'action', 'generation', 'support_confirmed', 'mounted_confirmed'}
                    or type(body.get('generation')) is not int
                    or type(body.get('support_confirmed')) is not bool
                    or type(body.get('mounted_confirmed')) is not bool):
                raise ValueError('请刷新状态并按向导确认安装与支撑')
            return controller.imu_wizard_action(body.get('action'), body['generation'],
                support_confirmed=body['support_confirmed'], mounted_confirmed=body['mounted_confirmed'])
        if path == '/api/motion/capture-all-zeros':
            if set(body) != {'mechanical_reference_confirmed', 'support_confirmed', 'generation'} or type(body.get('generation')) is not int:
                raise ValueError('请刷新真机状态并明确确认零姿态')
            return controller.capture_all_zeros(body.get('mechanical_reference_confirmed'), body['generation'], support_confirmed=body.get('support_confirmed'))
        if path == '/api/motion/command' or (path == '/api/motion/start' and body.get('mode') != 'shadow'):
            raise ValueError('本次仅开放校准及真实反馈推理，站姿和行走尚未开放')
        if path in ('/api/motion/stop', '/api/motion/heartbeat'):
            return original_action(path, body)
        with controller._lock:
            controller._require_no_imu_wizard_locked()
            return original_action(path, body)
    workbench.motion_action = calibration_action
    controller.start()
    original = calibration_handler(workbench)
    class Handler(original):
        def valid_host(self):
            return self.headers.get('Host') in tuple(host + ':' + port for host in ('127.0.0.1', 'localhost') for port in (os.environ['XGODUCK_LOCAL_PORT'], os.environ['XGODUCK_REMOTE_PORT']))
    server = web_app.ThreadingHTTPServer(('127.0.0.1', int(os.environ['XGODUCK_REMOTE_PORT'])), Handler)
    threading.Thread(target=supervise_page, daemon=True).start()
    print('MODEL_WEB_READY', flush=True)
    server.serve_forever()
finally:
    shutdown()
    if server is not None:
        server.server_close()
'''

MOTION_WRAPPER = MOTION_WRAPPER.replace('__IMU_WIZARD_RUNTIME__', imu_wizard.runtime_source(), 1)
MOTION_WRAPPER = MOTION_WRAPPER.replace('__CALIBRATION_RESTORE_RUNTIME__', calibration_restore.runtime_source(), 1)


def replace_once(source, old, new):
    if source.count(old) != 1:
        raise RuntimeError("Pinned supervisor adapter no longer matches: " + old[:60])
    return source.replace(old, new, 1)


BULK_ZERO_PANEL = '''<div class="motion-body" id="bulkZeroPanel">
<label class="support-check"><input id="captureZerosReference" type="checkbox"><span>已按参考摆好全部关节零姿态，嘴巴闭合</span></label>
<div class="motion-actions"><button class="button primary" id="captureAllZeros" type="button" disabled>采集全部零点</button></div>
<p class="motion-note">一次保存 15 个关节当前零点，舵机保持释放。无需持续手托；正方向、安全范围和 IMU 仍需继续核对。</p>
<p class="motion-note" id="captureAllZerosResult" role="status" aria-live="polite">摆好后勾选确认，再点击采集。</p></div>'''

BULK_ZERO_SCRIPT = r'''<script>
let captureAllZerosBusy=false;
let lastMotionReceivedAt=null;
const hardwareFreshBeforeBulkZeros=hardwareFresh;
hardwareFresh=function(){
  const elapsed=lastMotionReceivedAt===null?Infinity:performance.now()-lastMotionReceivedAt;
  return hardwareFreshBeforeBulkZeros()&&Number.isFinite(elapsed)&&elapsed>=0&&elapsed<1500;
};
const receiveMotionBeforeBulkZeros=receiveMotion;
receiveMotion=function(status){
  if(status&&typeof status==='object'&&typeof status.mode==='string')lastMotionReceivedAt=performance.now();
  return receiveMotionBeforeBulkZeros(status);
};
const renderBeforeBulkZeros=renderMotion;
function canCaptureAllZeros(){
  const s=motion.status;
  return !!state.token&&hardwareFresh()&&s?.connected===true&&s.mode==='off'&&s.armed===false&&Number.isInteger(s.generation)
    &&Array.isArray(s.torque_ids)&&s.torque_ids.length===0&&Array.isArray(s.fault_ids)&&s.fault_ids.length===0
    &&Array.isArray(s.online_ids)&&s.online_ids.length===15&&motorIds.every(id=>s.online_ids.includes(id))
    &&Number.isFinite(s.age_ms)&&s.age_ms>=0&&s.age_ms<100&&s.imu_feedback_ok===true
    &&Number.isFinite(s.imu_age_ms)&&s.imu_age_ms>=0&&s.imu_age_ms<100&&$('supportConfirmed').checked&&$('captureZerosReference').checked;
}
renderMotion=function(){
  renderBeforeBulkZeros();
  $('captureAllZeros').disabled=captureAllZerosBusy||motion.busy||motion.stopping||!canCaptureAllZeros();
  $('captureAllZeros').textContent=captureAllZerosBusy?'正在采集…':'采集全部零点';
  $('captureZerosReference').disabled=captureAllZerosBusy;
};
$('captureZerosReference').onchange=()=>renderMotion();
$('zeroReferenceConfirmed').closest('label').style.display='none';
$('captureAllZeros').onclick=async()=>{
  if(captureAllZerosBusy||motion.busy||motion.stopping||!canCaptureAllZeros())return;
  const body={mechanical_reference_confirmed:$('captureZerosReference').checked,support_confirmed:$('supportConfirmed').checked,generation:motion.status.generation};
  captureAllZerosBusy=true;motion.busy=true;renderMotion();
  $('captureAllZerosResult').textContent='正在读取并保存 15 个关节零点…';
  const controller=new AbortController();const timeout=setTimeout(()=>controller.abort(),10000);
  try{
    const status=await api('/api/motion/capture-all-zeros',body,controller);receiveMotion(status);motionError(status.error||'');
    $('captureAllZerosResult').textContent='15 / 15 零点已保存。机身继续由支架承重，可以调整关节核对正方向、安全范围和 IMU，无需维持零姿态等待。';
  }catch(error){
    const message=error.name==='AbortError'?'采集请求超时，请刷新状态确认结果后再操作。':error.message;
    $('captureAllZerosResult').textContent='未确认采集成功：'+message;motionError(message);
  }finally{clearTimeout(timeout);captureAllZerosBusy=false;motion.busy=false;renderMotion();}
};
renderMotion();
setInterval(renderMotion,250);
</script>'''

PAUSED_FEEDBACK_SCRIPT = r'''<script>
const hardwareFreshBeforeFeedbackPause=hardwareFresh;
hardwareFresh=function(){return motion.status?.paused!==true&&motion.status?.read_only!==true&&hardwareFreshBeforeFeedbackPause();};
const imuWizardCanBeforeFeedbackPause=imuWizardCan;
imuWizardCan=function(action){if(motion.status?.paused===true)return action==='cancel'&&!!state.token&&!imuWizardCancelBusy;return imuWizardCanBeforeFeedbackPause(action);};
const renderBeforeFeedbackPause=renderMotion;
renderMotion=function(){
  renderBeforeFeedbackPause();const s=motion.status;if(s?.paused!==true)return;
  motion.owned=false;
  const confirmed=s.pause_release_confirmed===true&&s.pause_release_status==='confirmed';
  const release=confirmed?'停止时已确认舵机释放':'舵机释放未确认';
  const first=s.pause_feedback||{};
  const offline=Number.isInteger(first.online_mask)?motorIds.filter((id,i)=>!(first.online_mask&(1<<i))):[];
  const fault=Number.isInteger(first.fault_mask)?motorIds.filter((id,i)=>first.fault_mask&(1<<i)):[];
  const cause=offline.length?'首次离线 ID '+offline.join(', '):fault.length?'首次故障 ID '+fault.join(', '):s.pause_reason||'反馈异常';
  $('hardwareConnection').textContent='反馈已暂停 · 本页只读';
  $('motionMode').textContent='反馈暂停 · '+release;
  $('hardwareDetail').textContent=cause;
  $('motionReadiness').textContent=cause+'。'+release+'。已保存校准保留；检查连接后需重新开启校准会话。';
  $('motionGate').textContent='本次反馈故障已锁定，采集和真实反馈推理均已禁用。本页不再发送串口指令。';
  $('motionHeartbeat').textContent='反馈线程已停止 · 不再发送串口指令';
  $('imuWizardGate').textContent='反馈已暂停；之前保存的零点与 IMU 校准保留。';
  for(const element of document.querySelectorAll('#motionPanel button,#motionPanel input,#motionPanel select'))element.disabled=true;
  $('imuWizardCancel').disabled=!state.token||imuWizardCancelBusy;
  $('imuWizardCancel').textContent='清除未完成向导（本页仍只读）';
};
renderMotion();
</script>'''


def remote_source(config):
    if RESTORE_RECORD is not None:
        calibration_restore.validate_restore_record(RESTORE_RECORD,
            boot_id=config['review']['expected_boot_id'], cid=config['review']['expected_cid'],
            serial_device=config['review']['serial_device'])
    config = {**config, 'restore_calibration': RESTORE_RECORD}
    main = base.REMOTE_MAIN
    main = replace_once(main, '    return tmp\n', '    serial_device_proof(CONFIG["review"]["serial_device"])\n    return tmp\n')
    main = replace_once(main, '                try:\n                    child.wait(timeout=1)\n                except subprocess.TimeoutExpired:',
                        '                try:\n                    child.wait(timeout=3)\n                except subprocess.TimeoutExpired:')
    main = replace_once(main, '        snapshot()\n        emit("unpacked"',
                        '        env["XGODUCK_SERIAL_REVIEW"] = json.dumps(CONFIG["review"]["serial_device"])\n        env["XGODUCK_CALIBRATION_RESTORE"] = json.dumps(CONFIG["restore_calibration"])\n        snapshot()\n        emit("unpacked"')
    main = replace_once(main, '当前是 X5 临时模型测试，重启后需重新开启。舵机控制尚未启用。',
                        '当前仅开放关节、IMU 校准与真实反馈推理。舵机保持释放，站立及行走尚未开放。校准会同步保存到笔记本。')
    anchor = '        with open(html, "w", encoding="utf-8") as stream:\n'
    ui = r'''
        notice = '<style>#holdMotion,#walkMotion,#zeroMotionCommand,.motion-command{display:none!important}</style><script>const originalCalibrationRender=renderMotion;renderMotion=function(){originalCalibrationRender();for(const id of ["holdMotion","walkMotion","zeroMotionCommand"])document.getElementById(id).disabled=true;document.getElementById("shadowMotion").disabled=document.getElementById("shadowMotion").disabled||!motion.status?.calibrated;document.getElementById("realMotionTitle").textContent="真实反馈检查";document.getElementById("motionGate").textContent=motion.status?.calibrated?"校准已保存，可以检查真实反馈推理；本次不开放舵机使能。":"完成关节、机械参考及 IMU 校准后，再检查真实反馈推理。";};renderMotion();</script>'
        capture_anchor = '<div class="motion-table-wrap">'
        if content.count(capture_anchor) != 1:
            raise RuntimeError('Reviewed joint calibration panel changed')
        content = content.replace(capture_anchor, __BULK_ZERO_PANEL__ + capture_anchor, 1)
        content = content.replace('</body>', notice + __BULK_ZERO_SCRIPT__ + '</body>')
        content = transform_imu_wizard_html(content)
        content = content.replace('</body>', __PAUSED_FEEDBACK_SCRIPT__ + '</body>')
        content = content.replace('已依据可靠的模型零姿态参考，确认每个关节的 q = 0 和正方向', '已按参考摆好全部关节零姿态，嘴巴闭合')
        content = content.replace('已支撑机身、双脚悬空，释放舵机后不会跌落', '机身已用稳固垫块或支架承重，双脚悬空；无需持续手托')
        content = content.replace('先以默认站姿使能舵机，确认机身与双腿朝向正确，再启动低速行走。始终支撑机身；不符合预期时立即停止并释放。', '先完成关节和 IMU 校准，再查看真实反馈推理。始终稳固支撑机身，本次不会使能舵机。')
        content = content.replace('只开放站姿和低速行走。心跳每 150 ms 发送；关闭页面、切走或失去焦点会停止并释放。失联或倾斜超限也会停止；请持续使用支撑。', '本次仅采集校准及真实反馈推理，不开放站立或行走。请持续使用稳固支撑。')
'''
    ui = ui.replace('__BULK_ZERO_PANEL__', repr(BULK_ZERO_PANEL)).replace('__BULK_ZERO_SCRIPT__', repr(BULK_ZERO_SCRIPT)).replace('__PAUSED_FEEDBACK_SCRIPT__', repr(PAUSED_FEEDBACK_SCRIPT))
    main = replace_once(main, anchor, ui + '\n' + anchor)
    functions = (base.safe_relative_path, base.normalized_kernel_sha256, validate_serial_review,
                 validate_review, serial_device_proof, base.verify_root_block_node,
                 base.inspect_tar, base.inspect_wheels)
    # The same identity helper is included in the child before it opens serial.
    child_helpers = ("import re,stat\nfrom pathlib import Path,PurePosixPath\n"
                     + inspect.getsource(validate_serial_review) + inspect.getsource(serial_device_proof))
    config = {**config, "server_wrapper": child_helpers + config["server_wrapper"]}
    return ("import hashlib,io,json,os,re,signal,stat,subprocess,sys,tarfile,tempfile,time,uuid,zipfile\n"
            "from pathlib import Path,PurePosixPath\nMAX_EXPANDED_WHEEL_BYTES=%d\n" % base.MAX_EXPANDED_WHEEL_BYTES
            + imu_wizard_ui.runtime_source()
            + "\n".join(inspect.getsource(function) for function in functions)
            + main.replace("__CONFIG__", repr(config), 1))


class Recorder(MODEL_RECORDER):
    def __init__(self, directory):
        super().__init__(directory)
        self.status(capability_scope=SCOPE, servo_arm_authorized=False, serial_shutdown_confirmed=False)

    def event(self, event):
        super().event(event)
        line = event.get("line", "") if event.get("event") == "child_log" else ""
        if line.startswith("CALIBRATION_SNAPSHOT "):
            snapshot = json.loads(line[len("CALIBRATION_SNAPSHOT "):])
            if snapshot.get("version") != 1 or len(snapshot.get("joints", [])) != 15:
                raise RuntimeError("Malformed RAM calibration snapshot")
            fd, temporary = tempfile.mkstemp(prefix=".calibration-", dir=str(self.directory))
            try:
                with os.fdopen(fd, "w", encoding="utf-8") as stream:
                    json.dump(snapshot, stream, ensure_ascii=False, indent=2, allow_nan=False)
                    stream.write("\n")
                os.replace(temporary, self.directory / "calibration-latest.json")
            finally:
                if os.path.exists(temporary):
                    os.unlink(temporary)
            self.status(calibration_snapshot_path=str(self.directory / "calibration-latest.json"))
        elif line.startswith("MOTION_SHUTDOWN "):
            proof = json.loads(line[len("MOTION_SHUTDOWN "):])
            self.report["serial_shutdown_proof"] = proof
            self.report["serial_shutdown_confirmed"] = all(proof.get(key) is True for key in ("stop_ack", "release_confirmed", "serial_closed", "worker_stopped"))
            self.status()
        elif line.startswith("MOTION_PAUSED "):
            paused = json.loads(line[len("MOTION_PAUSED "):])
            if (paused.get('pause_worker_stopped') is not True
                    or paused.get('pause_serial_closed') is not True
                    or paused.get('pause_release_status') not in ('unknown', 'confirmed')):
                raise RuntimeError('Malformed stopped-feedback pause evidence')
            self.report.setdefault('feedback_pause', paused)
            self.status(feedback_paused=True, feedback_read_only=True,
                        servo_motion_authorized=False)


def configure_adapter():
    if hashlib.sha256(Path(base.__file__).read_bytes()).hexdigest() != BASE_SHA256:
        raise RuntimeError("The reviewed base supervisor source changed; review before adapting it")
    base.APPROVED_ARCHIVE_SHA256 = ARCHIVE_SHA256
    base.APPROVED_ARCHIVE_BYTES = ARCHIVE_BYTES
    base.ALLOWED_MEMBERS = MEMBERS
    base.validate_review = validate_review
    base.check_guard = check_guard
    base.remote_source = remote_source
    base.SERVER_WRAPPER = MOTION_WRAPPER
    base.Recorder = Recorder
    # Change only truthful console wording and cleanup grace in this process;
    # the running model runner's source file is never modified.
    source = inspect.getsource(base.supervise)
    source = replace_once(source, "RAM model web: checking the live reviewed board; no serial or motor output.",
                          "RAM calibration web: checking the reviewed feedback port; motor output disabled.")
    source = replace_once(source, "time.monotonic() + 4\n", "time.monotonic() + 6\n")
    exec(compile(source, "<pinned-calibration-supervisor-adapter>", "exec"), base.__dict__)


def self_test():
    import unittest
    from unittest import mock

    class Tests(unittest.TestCase):
        def controller_namespace(self):
            import ast
            import importlib.util
            import threading
            policy = Path(__file__).resolve().parents[1] / "x5_policy"
            sys.path.insert(0, str(policy))
            try:
                spec = importlib.util.spec_from_file_location("motion_for_ram_wrapper_mocks", policy / "motion.py")
                module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(module)
                from test_motion import FakeTransport, ready_config
            finally:
                sys.path.remove(str(policy))
            # Extract only class/helper definitions; omit signal handlers,
            # prctl, audit-hook installation and actual app startup.
            definitions = MOTION_WRAPPER[MOTION_WRAPPER.index("def release_state_ok"):MOTION_WRAPPER.index("try:\n    workbench = web_app.Workbench()")]
            namespace = {"motion": module, "os": os, "time": time, "threading": threading,
                         "json": json, "Path": Path, "expected": self.fixture()["serial_device"],
                         "opened_transports": [], "exit_requested": False, "closing": False,
                         "shutdown_lock": threading.Lock(), "shutdown_done": threading.Event(),
                         "controller": None, "server": None, "shutdown_proof": {},
                         "serial_device_proof": mock.Mock(return_value=(0, 0, 0))}
            # Import only what the complete child wrapper actually imports;
            # omit hardware/app imports and every startup/signal/audit action.
            imports = [node for node in ast.parse(MOTION_WRAPPER).body if isinstance(node, (ast.Import, ast.ImportFrom))
                       and not (isinstance(node, ast.Import) and any(alias.name in ('motion', 'web_app') for alias in node.names))]
            exec(compile(ast.Module(body=imports, type_ignores=[]), '<actual-wrapper-imports>', 'exec'), namespace)
            exec(compile(imu_wizard.runtime_source(), '<actual-imu-wizard-runtime>', 'exec'), namespace)
            exec(compile(calibration_restore.runtime_source(), '<actual-calibration-restore-runtime>', 'exec'), namespace)
            exec(compile(definitions, "<actual-controller-fake-transport>", "exec"), namespace)
            return namespace, FakeTransport, ready_config

        def fixture(self):
            return {"scope": SCOPE, "physical_support_confirmed": True, "servo_arm_authorized": False,
                    "servo_motion_authorized_by_this_review": False,
                    "expected_boot_id": str(uuid.uuid4()), "expected_cid": "a"*32,
                    "expected_root": {"device": "179:2", "source": "/dev/root", "filesystem": "ext4"},
                    "kernel_log_sha256": "b"*64, "required_unmounted_device": "179:1",
                    "serial_device": {"path": "/dev/ttyUSB0", "vid": "1a86", "pid": "7523", "serial_number": "", "syspath": "/sys/devices/platform/example/usb1/1-1/1-1.3"}}

        def test_capability_is_separate_and_restricted(self):
            review = self.fixture()
            validate_review(review)
            for changes in ({"scope": "ram_model_web_only"}, {"physical_support_confirmed": False},
                            {"servo_arm_authorized": True}, {"servo_motion_authorized_by_this_review": True}):
                with self.assertRaises(ValueError):
                    validate_review({**review, **changes})
            for changes in ({"path": "/dev/ttyUSB1"}, {"syspath": "/sys/devices/../other"}, {"vid": "XXXX"}):
                with self.assertRaises(ValueError):
                    validate_serial_review({**review["serial_device"], **changes})

        def test_remote_and_wrapper_compile(self):
            compile(remote_source({"review": self.fixture(), "server_wrapper": MOTION_WRAPPER}), "<mock-calibration-remote>", "exec")
            compile(MOTION_WRAPPER, "<mock-calibration-wrapper>", "exec")

        def test_new_guard_cannot_reuse_old_model_grant(self):
            review = self.fixture()
            status = {"safe": False, "safe_for_ram_model_web": False, "safe_for_ram_motion_calibration": True,
                      "capability_scope": SCOPE, "physical_support_confirmed": True, "servo_arm_authorized": False,
                      "serial_device": review["serial_device"], "requires_serial_identity_proof": True,
                      "serial_identity_runtime_verified_by_guard": False, "local_pid": os.getpid()+100,
                      "last_heartbeat_monotonic": 100, "boot_id": review["expected_boot_id"], "card_cid": review["expected_cid"],
                      "root": review["expected_root"], "kernel_log_sha256": review["kernel_log_sha256"]}
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "guard.json"
                path.write_text(json.dumps(status)); path.chmod(0o600)
                with mock.patch("os.kill"):
                    check_guard(path, review, now=101)
                    for changes in ({"safe_for_ram_model_web": True}, {"capability_scope": "ram_model_web_only"},
                                    {"servo_arm_authorized": True}, {"physical_support_confirmed": False},
                                    {"last_heartbeat_monotonic": 90}):
                        path.write_text(json.dumps({**status, **changes}))
                        with self.assertRaises(RuntimeError):
                            check_guard(path, review, now=101)

        def test_private_mac_snapshot_and_unknown_release(self):
            with tempfile.TemporaryDirectory() as directory:
                recorder = Recorder(directory)
                snapshot = {"version": 1, "joints": [{"id": i} for i in range(15)]}
                recorder.event({"event": "child_log", "line": "CALIBRATION_SNAPSHOT " + json.dumps(snapshot)})
                self.assertEqual((Path(directory)/"calibration-latest.json").stat().st_mode & 0o777, 0o600)
                recorder.event({"event": "child_log", "line": "MOTION_SHUTDOWN " + json.dumps({"stop_ack": True, "release_confirmed": False, "serial_closed": True, "worker_stopped": True})})
                self.assertFalse(recorder.report["serial_shutdown_confirmed"])
                recorder.close()

        def test_adapter_pin_and_scope_transport_constraints(self):
            self.assertEqual(hashlib.sha256(Path(base.__file__).read_bytes()).hexdigest(), BASE_SHA256)
            self.assertEqual(len(MEMBERS), 22)
            self.assertIn("kind not in (motion.STOP, motion.HELLO, motion.STATE, motion.CONFIG)", MOTION_WRAPPER)
            self.assertNotIn("transport.request(motion.ARM", MOTION_WRAPPER)
            self.assertNotIn("transport.request(motion.TARGET", MOTION_WRAPPER)

        def test_actual_controller_starts_off_and_closes_with_fresh_release_proof(self):
            namespace, factory, _ = self.controller_namespace()
            with tempfile.TemporaryDirectory() as directory:
                transport = factory("fake")
                controller = namespace["CalibrationController"]({}, config_path=Path(directory)/"calibration.json", transport_factory=lambda _: transport)
                controller._connect_once()
                self.assertEqual(controller._mode, "off")
                self.assertFalse(controller.status()["calibrated"])
                controller.close()
                self.assertTrue(namespace["shutdown_proof"]["release_confirmed"])
                self.assertTrue(transport.closed)
                self.assertTrue(all(kind not in (namespace["motion"].ARM, namespace["motion"].TARGET) for kind, _ in transport.requests))

        def test_all_zeros_one_state_one_save_and_direction_unchanged(self):
            namespace, factory, _ = self.controller_namespace()
            with tempfile.TemporaryDirectory() as directory:
                transport = factory("fake")
                positions = list(range(1200, 1215))
                transport.positions = positions.copy()
                controller = namespace["CalibrationController"]({}, config_path=Path(directory)/"calibration.json", transport_factory=lambda _: transport)
                controller._connect_once()
                transport.positions = list(range(2200, 2215))
                requests = len(transport.requests)
                before = namespace["motion"].copy.deepcopy(controller._config)
                with mock.patch.object(controller, "_save_locked", wraps=controller._save_locked) as save, mock.patch("builtins.print"):
                    status = controller.capture_all_zeros(True, controller._generation, support_confirmed=True)
                    save.assert_called_once_with()
                self.assertEqual([joint["zero"] for joint in controller._config["joints"]], positions)
                self.assertTrue(status["mechanical_reference_confirmed"])
                self.assertEqual(len(transport.requests), requests)
                for old, new in zip(before["joints"], controller._config["joints"]):
                    self.assertEqual({key: value for key, value in old.items() if key != "zero"}, {key: value for key, value in new.items() if key != "zero"})
                    self.assertFalse(new["direction_confirmed"])
                self.assertTrue(status["imu_feedback_ok"])
                controller.close()

        def test_all_zeros_rejects_unsafe_feedback_and_confirmation(self):
            cases = ("reference", "support", "generation", "closed", "disconnected", "mode", "offline", "fault", "torque", "armed", "age", "imu_age", "imu_bad")
            for case in cases:
                namespace, factory, _ = self.controller_namespace()
                with tempfile.TemporaryDirectory() as directory:
                    clock = [100.]
                    transport = factory("fake")
                    controller = namespace["CalibrationController"]({}, config_path=Path(directory)/"calibration.json", transport_factory=lambda _: transport, clock=lambda: clock[0])
                    controller._connect_once()
                    confirmed, support, generation = True, True, controller._generation
                    if case == "reference": confirmed = False
                    elif case == "support": support = False
                    elif case == "generation": generation += 1
                    elif case == "closed": controller._closed.set()
                    elif case == "disconnected": controller._connected = False
                    elif case == "mode": controller._mode = "shadow"
                    elif case == "offline": controller._state["online_mask"] = 0
                    elif case == "fault": controller._state["fault_mask"] = 1
                    elif case == "torque": controller._state["torque_mask"] = 1
                    elif case == "armed": controller._state["armed"] = True
                    elif case == "age": clock[0] += .11
                    elif case == "imu_age": controller._state["imu_us"] = (controller._state["sample_us"] - 110000) & 0xffffffff
                    else: controller._state["imu_ok"] = False
                    before = namespace["motion"].copy.deepcopy(controller._config)
                    with mock.patch.object(controller, "_save_locked") as save:
                        with self.assertRaises(namespace["motion"].MotionError, msg=case):
                            controller.capture_all_zeros(confirmed, generation, support_confirmed=support)
                        save.assert_not_called()
                    self.assertEqual(controller._config, before, case)
                    controller.close()

        def test_all_zeros_save_failure_restores_entire_config_and_dirty(self):
            namespace, factory, _ = self.controller_namespace()
            with tempfile.TemporaryDirectory() as directory:
                transport = factory("fake")
                controller = namespace["CalibrationController"]({}, config_path=Path(directory)/"calibration.json", transport_factory=lambda _: transport)
                controller._connect_once()
                controller._config_dirty = False
                before = namespace["motion"].copy.deepcopy(controller._config)
                with mock.patch.object(controller, "_save_locked", side_effect=OSError("mock write failed")) as save:
                    with self.assertRaises(OSError):
                        controller.capture_all_zeros(True, controller._generation, support_confirmed=True)
                    save.assert_called_once_with()
                self.assertEqual(controller._config, before)
                self.assertFalse(controller._config_dirty)
                controller.close()

        def test_existing_capture_rejects_global_fault_and_closed_worker(self):
            for case in ("fault", "offline", "closed"):
                namespace, factory, _ = self.controller_namespace()
                with tempfile.TemporaryDirectory() as directory:
                    transport = factory("fake")
                    controller = namespace["CalibrationController"]({}, config_path=Path(directory)/"calibration.json", transport_factory=lambda _: transport)
                    controller._connect_once()
                    if case == "fault": controller._state["fault_mask"] = 1 << 5
                    elif case == "offline": controller._state["online_mask"] ^= 1 << 5
                    else: controller._closed.set()
                    with mock.patch.object(controller, "_save_locked") as save:
                        with self.assertRaises(namespace["motion"].MotionError):
                            controller.capture(namespace["motion"].MOTOR_IDS[0], "zero")
                        save.assert_not_called()
                    controller.close()

        def test_capture_http_inherits_auth_body_and_workbench_lock(self):
            import ast, io, secrets, threading, types
            from http.server import BaseHTTPRequestHandler
            namespace, factory, ready = self.controller_namespace()
            app_path = Path(__file__).resolve().parents[1] / "x5_policy" / "web_app.py"
            tree = ast.parse(app_path.read_text())
            handler_node = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "make_handler")
            handler_globals = {"BaseHTTPRequestHandler": BaseHTTPRequestHandler, "json": json, "secrets": secrets}
            exec(compile(ast.Module(body=[handler_node], type_ignores=[]), str(app_path), "exec"), handler_globals)
            namespace["web_app"] = types.SimpleNamespace(**handler_globals)
            action_node = next(node for node in ast.walk(ast.parse(MOTION_WRAPPER)) if isinstance(node, ast.FunctionDef) and node.name == "calibration_action")
            exec(compile(ast.Module(body=[action_node], type_ignores=[]), "<actual-calibration-action>", "exec"), namespace)
            with tempfile.TemporaryDirectory() as directory:
                transport = factory("fake")
                controller = namespace["CalibrationController"]({}, config_path=Path(directory)/"calibration.json", transport_factory=lambda _: transport)
                controller._connect_once()
                namespace["controller"] = controller
                namespace["original_action"] = mock.Mock()
                workbench = types.SimpleNamespace(token="mock-session", lock=threading.Lock(), motion_action=mock.Mock(side_effect=namespace["calibration_action"]))
                handler_class = namespace["calibration_handler"](workbench)
                valid_body = {"mechanical_reference_confirmed": True, "support_confirmed": True, "generation": controller._generation}
                def request(changes=None, body=None, raw=None, path="/api/motion/capture-all-zeros"):
                    data = json.dumps(valid_body if body is None else body).encode() if raw is None else raw
                    instance = object.__new__(handler_class)
                    instance.server = types.SimpleNamespace(server_address=("127.0.0.1", 8767))
                    instance.path = path
                    instance.headers = {"Host": "127.0.0.1:8767", "Origin": "http://127.0.0.1:8767", "X-Session-Token": workbench.token, "Content-Type": "application/json", "Content-Length": str(len(data))}
                    instance.headers.update(changes or {})
                    instance.rfile = io.BytesIO(data)
                    instance.reply = mock.Mock()
                    instance.do_POST()
                    return instance.reply.call_args.args[0]
                for changes, code in (({"X-Session-Token": ""}, 403), ({"X-Session-Token": "wrong"}, 403), ({"Host": "external.example:8767"}, 403), ({"Origin": "http://external.example"}, 403), ({"Content-Type": "text/plain"}, 415), ({"Content-Length": "0"}, 413), ({"Content-Length": "4097"}, 413), ({"Content-Length": "invalid"}, 400)):
                    self.assertEqual(request(changes), code)
                    workbench.motion_action.assert_not_called()
                self.assertEqual(request(raw=b"not-json"), 400)
                self.assertEqual(request(raw=b"[]"), 400)
                workbench.motion_action.assert_not_called()
                with workbench.lock:
                    self.assertEqual(request(), 409)
                workbench.motion_action.assert_not_called()
                with mock.patch.object(controller, "_save_locked") as save:
                    self.assertEqual(request(body={**valid_body, "mechanical_reference_confirmed": False}), 400)
                    self.assertEqual(request(body={**valid_body, "support_confirmed": False}), 400)
                    self.assertEqual(request(body={"generation": controller._generation}), 400)
                    self.assertEqual(request(body={**valid_body, "unexpected": True}), 400)
                    save.assert_not_called()
                    self.assertEqual(request(), 200)
                    save.assert_called_once_with()
                wizard_path = '/api/motion/imu-wizard'
                wizard_body = dict(action='level', generation=controller._generation,
                                   support_confirmed=True, mounted_confirmed=True)
                self.assertEqual(request({'X-Session-Token': 'wrong'}, body=wizard_body, path=wizard_path), 403)
                with workbench.lock:
                    self.assertEqual(request(body=wizard_body, path=wizard_path), 409)
                    self.assertEqual(request(body={**wizard_body, 'action': 'cancel', 'generation': 0,
                                                  'support_confirmed': False, 'mounted_confirmed': False},
                                             path=wizard_path), 200)
                with mock.patch.object(controller, '_save_locked') as save:
                    self.assertEqual(request(body={**wizard_body, 'extra': True}, path=wizard_path), 400)
                    self.assertEqual(request(body={**wizard_body, 'mounted_confirmed': False}, path=wizard_path), 400)
                    self.assertEqual(request(body=wizard_body, path=wizard_path), 200)
                    self.assertTrue(controller.status()['imu_wizard']['busy'])
                    self.assertEqual(request(), 400, 'Joint captures must be frozen throughout the wizard')
                    self.assertEqual(request(body={'axes': ['x', 'y', 'z'], 'axes_confirmed': True},
                                             path='/api/motion/imu-axes'), 400)
                    save.assert_not_called()
                    controller._config = ready()
                    with self.assertRaises(namespace['motion'].MotionError):
                        controller.start_mode('shadow')
                    self.assertEqual(request(body={**wizard_body, 'action': 'cancel'}, path=wizard_path), 200)
                controller.close()

        def test_off_mode_fault_or_missing_torque_feedback_stops(self):
            for field, value in (("online", 0), ("fault", 1), ("torque", 1), ("armed", True)):
                namespace, factory, _ = self.controller_namespace()
                with tempfile.TemporaryDirectory() as directory:
                    transport = factory("fake")
                    controller = namespace["CalibrationController"]({}, config_path=Path(directory)/"calibration.json", transport_factory=lambda _: transport)
                    controller._connect_once()
                    setattr(transport, field, value)
                    with self.assertRaises(namespace["motion"].MotionError):
                        controller._cycle()
                    self.assertEqual([kind for kind, _ in transport.requests[-2:]],
                                     [namespace["motion"].STOP, namespace["motion"].STATE])
                    self.assertTrue(transport.closed)
                    controller.close()

        def test_shadow_needs_full_calibration_and_arm_target_are_blocked(self):
            namespace, factory, ready = self.controller_namespace()
            with tempfile.TemporaryDirectory() as directory:
                transport = factory("fake")
                controller = namespace["CalibrationController"]({}, config_path=Path(directory)/"calibration.json", transport_factory=lambda _: transport)
                controller._connect_once()
                for mode in ("shadow", "hold", "walk"):
                    with self.assertRaises(namespace["motion"].MotionError):
                        controller.start_mode(mode)
                controller._config = ready()
                self.assertEqual(controller.start_mode("shadow")["mode"], "shadow")
                with self.assertRaises(namespace["motion"].MotionError):
                    controller.command(0, 0, 0)
                controller.close()
            serial = namespace["FeedbackTransport"]("/dev/ttyUSB0")
            serial.serial = mock.Mock()
            serial.serial.fileno.return_value = 123
            with mock.patch.object(namespace["motion"].SerialTransport, "request", return_value=b"ok") as request:
                for kind in (namespace["motion"].ARM, namespace["motion"].TARGET):
                    with self.assertRaises(namespace["motion"].MotionError):
                        serial.request(kind)
                request.assert_not_called()
                self.assertEqual(serial.request(namespace["motion"].STATE), b"ok")

        def test_frozen_sample_is_unknown_and_worker_never_reconnects(self):
            namespace, factory, _ = self.controller_namespace()
            with tempfile.TemporaryDirectory() as directory:
                transport = factory("fake")
                controller = namespace["CalibrationController"]({}, config_path=Path(directory)/"calibration.json", transport_factory=lambda _: transport)
                controller._connect_once()
                transport.freeze_sample = True
                controller.close()
                self.assertFalse(namespace["shutdown_proof"]["release_confirmed"])
                self.assertTrue(transport.closed)
            namespace, _, _ = self.controller_namespace()
            with tempfile.TemporaryDirectory() as directory:
                factory = mock.Mock()
                controller = namespace["CalibrationController"]({}, config_path=Path(directory)/"calibration.json", transport_factory=factory)
                controller._run()
                factory.assert_not_called()
                self.assertFalse(namespace["exit_requested"])
                self.assertTrue(controller.status()["paused"])
                self.assertTrue(controller._closed.is_set())

        def paused_controller(self, namespace, factory, directory):
            transport = factory('fake')
            controller = namespace['CalibrationController']({}, config_path=Path(directory)/'calibration.json',
                                                            transport_factory=lambda _: transport)
            controller._connect_once()
            namespace['controller'] = controller
            transport.online = namespace['motion'].ALL_MASK ^ (1 << 12)
            with mock.patch('builtins.print'):
                controller._run()
            return controller, transport

        def test_paused_fault_is_immutable_and_safe_actions_never_request_or_reopen(self):
            namespace, factory, ready = self.controller_namespace()
            with tempfile.TemporaryDirectory() as directory:
                controller, transport = self.paused_controller(namespace, factory, directory)
                controller._config = ready()
                before = namespace['motion'].copy.deepcopy(controller._config)
                fault = namespace['motion'].copy.deepcopy(controller._feedback_fault)
                proof = namespace['motion'].copy.deepcopy(controller._release_proof)
                count, generation = len(transport.requests), controller._generation
                with mock.patch('builtins.print') as output:
                    controller._latch_feedback_fault_locked('later failure must not replace the first')
                    output.assert_not_called()
                for action in (controller.stop, controller.heartbeat,
                               lambda: controller.imu_wizard_action('cancel', -1)):
                    for _ in range(2):
                        status = action()
                        self.assertTrue(status['paused'])
                        self.assertFalse(status['connected'])
                        self.assertFalse(status['pending_stop'])
                        self.assertFalse(status['motor_output'])
                        self.assertEqual('off', status['mode'])
                        self.assertEqual(fault['error'], status['error'])
                        self.assertEqual(fault['feedback'], status['pause_feedback'])
                        self.assertFalse(status['pause_release_confirmed'])
                        self.assertEqual('unknown', status['pause_release_status'])
                denied = (controller._connect_once, controller.start,
                          lambda: controller.capture(10, 'zero'),
                          lambda: controller.configure_joint(10, 1, True),
                          lambda: controller.configure_reference(True),
                          lambda: controller.set_imu_axes(['x', 'y', 'z'], True), controller.calibrate_imu,
                          lambda: controller.imu_wizard_action('level', generation, support_confirmed=True, mounted_confirmed=True),
                          lambda: controller.capture_all_zeros(True, generation, support_confirmed=True),
                          lambda: controller.start_mode('shadow'), controller._save_locked)
                for action in denied:
                    with self.assertRaises(namespace['motion'].MotionError):
                        action()
                self.assertEqual(fault, controller._feedback_fault)
                self.assertEqual(proof, controller._release_proof)
                self.assertEqual(before, controller._config)
                self.assertEqual(generation, controller._generation)
                self.assertEqual(count, len(transport.requests))
                self.assertIsNone(controller._transport)
                self.assertTrue(transport.closed)

        def test_stop_race_cannot_requeue_a_serial_stop_after_worker_fault(self):
            import threading
            namespace, factory, _ = self.controller_namespace()
            with tempfile.TemporaryDirectory() as directory:
                transport = factory('fake')
                controller = namespace['CalibrationController']({}, config_path=Path(directory)/'calibration.json',
                                                                transport_factory=lambda _: transport)
                controller._connect_once()
                namespace['controller'] = controller
                entered, fault_done = threading.Event(), threading.Event()
                def competing_fault():
                    entered.wait(1)
                    with controller._lock:
                        controller._latch_feedback_fault_locked('concurrent feedback fault')
                        controller._finalize_serial_locked('concurrent feedback fault')
                    fault_done.set()
                worker = threading.Thread(target=competing_fault)
                original_stop = namespace['motion'].MotionController.stop
                def delayed_parent_stop(instance):
                    entered.set()
                    fault_done.wait(.05)
                    return original_stop(instance)
                worker.start()
                with mock.patch('builtins.print'), mock.patch.object(namespace['motion'].MotionController, 'stop', delayed_parent_stop):
                    controller.stop()
                    worker.join(timeout=1)
                self.assertFalse(worker.is_alive())
                self.assertTrue(fault_done.is_set())
                status = controller.status()
                self.assertTrue(status['paused'])
                self.assertFalse(status['pending_stop'])
                self.assertEqual('concurrent feedback fault', status['error'])
                count = len(transport.requests)
                controller.stop()
                self.assertEqual(count, len(transport.requests))

        def test_only_stopped_worker_and_all_closed_ports_can_keep_readonly_http(self):
            import types
            for failure in ('alive-worker', 'open-port', 'close-proof-missing'):
                namespace, factory, _ = self.controller_namespace()
                with tempfile.TemporaryDirectory() as directory:
                    controller, transport = self.paused_controller(namespace, factory, directory)
                    healthy = namespace['feedback_pause_status']()
                    self.assertTrue(healthy['read_only'])
                    self.assertEqual('unknown', healthy['pause_release_status'])
                    if failure == 'alive-worker':
                        controller._thread = types.SimpleNamespace(join=mock.Mock(), is_alive=lambda: True)
                    elif failure == 'open-port':
                        namespace['opened_transports'].append(types.SimpleNamespace(serial=object()))
                    else:
                        controller._release_proof['serial_closed'] = False
                    with self.assertRaises(RuntimeError):
                        namespace['feedback_pause_status']()
                    namespace['shutdown'] = mock.Mock()
                    namespace['server'] = types.SimpleNamespace(shutdown=mock.Mock())
                    with mock.patch('builtins.print'):
                        namespace['supervise_page']()
                    self.assertTrue(namespace['exit_requested'])
                    namespace['shutdown'].assert_called_once_with()
                    namespace['server'].shutdown.assert_called_once_with()
                    self.assertTrue(transport.closed)

        def test_pause_keeps_http_without_global_shutdown_but_parent_exit_still_closes_it(self):
            import types
            namespace, factory, _ = self.controller_namespace()
            with tempfile.TemporaryDirectory() as directory:
                controller, transport = self.paused_controller(namespace, factory, directory)
                count = len(transport.requests)
                namespace['shutdown'] = mock.Mock()
                namespace['server'] = types.SimpleNamespace(shutdown=mock.Mock())
                namespace['serial_device_proof'].reset_mock()
                def parent_requests_exit(delay):
                    namespace['shutdown'].assert_not_called()
                    namespace['server'].shutdown.assert_not_called()
                    self.assertFalse(namespace['closing'])
                    self.assertFalse(namespace['shutdown_done'].is_set())
                    self.assertEqual(count, len(transport.requests))
                    namespace['exit_requested'] = True
                with mock.patch('time.sleep', side_effect=parent_requests_exit), mock.patch('builtins.print') as output:
                    namespace['supervise_page']()
                paused = [call.args[0] for call in output.call_args_list if call.args[0].startswith('MOTION_PAUSED ')]
                self.assertEqual(1, len(paused))
                namespace['serial_device_proof'].assert_not_called()
                namespace['shutdown'].assert_called_once_with()
                namespace['server'].shutdown.assert_called_once_with()
                self.assertEqual(count, len(transport.requests))

        def test_fatal_cleanup_error_cannot_leave_http_running_or_retry_serial(self):
            import types
            namespace, factory, _ = self.controller_namespace()
            with tempfile.TemporaryDirectory() as directory:
                controller, transport = self.paused_controller(namespace, factory, directory)
                count = len(transport.requests)
                namespace['opened_transports'].append(types.SimpleNamespace(serial=object()))
                namespace['shutdown'] = mock.Mock(side_effect=RuntimeError('mock close failure'))
                namespace['server'] = types.SimpleNamespace(shutdown=mock.Mock())
                with mock.patch('builtins.print'), self.assertRaises(RuntimeError):
                    namespace['supervise_page']()
                namespace['server'].shutdown.assert_called_once_with()
                self.assertTrue(namespace['exit_requested'])
                self.assertEqual(count, len(transport.requests))

        def test_pause_recorder_keeps_first_fault_and_never_claims_unknown_release(self):
            with tempfile.TemporaryDirectory() as directory:
                recorder = Recorder(Path(directory)/'private-pause-record')
                first = {'pause_reason': 'first offline', 'pause_feedback': {'online_mask': 0},
                         'pause_release_status': 'unknown', 'pause_release_confirmed': False,
                         'pause_worker_stopped': True, 'pause_serial_closed': True}
                recorder.event({'event': 'child_log', 'line': 'MOTION_PAUSED ' + json.dumps(first)})
                recorder.event({'event': 'child_log', 'line': 'MOTION_PAUSED ' + json.dumps({**first, 'pause_reason': 'later'})})
                self.assertEqual(first, recorder.report['feedback_pause'])
                self.assertTrue(recorder.report['feedback_read_only'])
                self.assertFalse(recorder.report['serial_shutdown_confirmed'])
                for field in ('pause_worker_stopped', 'pause_serial_closed'):
                    with self.assertRaises(RuntimeError):
                        recorder.event({'event': 'child_log', 'line': 'MOTION_PAUSED ' + json.dumps({**first, field: False})})
                recorder.close()

        def test_fault_masks_logged_and_saved_proof_survives_shutdown(self):
            namespace, factory, _ = self.controller_namespace()
            with tempfile.TemporaryDirectory() as directory:
                transport = factory("fake")
                controller = namespace["CalibrationController"]({}, config_path=Path(directory)/"calibration.json", transport_factory=lambda _: transport)
                controller._connect_once()
                transport.online = namespace["motion"].ALL_MASK ^ (1 << 3)
                transport.fault, transport.torque, transport.armed = 1 << 5, 1 << 7, True
                namespace["controller"] = controller
                with mock.patch("builtins.print") as output:
                    controller._run()
                    lines = [call.args[0] for call in output.call_args_list]
                    fault = json.loads(next(line[len("MOTION_FAULT "):] for line in lines if line.startswith("MOTION_FAULT ")))
                    feedback = fault["feedback"]
                    self.assertEqual(feedback["online_mask"], transport.online)
                    self.assertEqual(feedback["fault_mask"], 1 << 5)
                    self.assertEqual(feedback["torque_mask"], 1 << 7)
                    self.assertTrue(feedback["armed"])
                    self.assertTrue(all(key in feedback for key in ("sample_us", "imu_us", "age_ms", "imu_age_ms")))
                    self.assertNotIn("config", fault)
                    self.assertNotIn("token", fault)
                    self.assertEqual([kind for kind, _ in transport.requests[-2:]], [namespace["motion"].STOP, namespace["motion"].STATE])
                    self.assertTrue(transport.closed)
                    saved = dict(namespace["shutdown_proof"])
                    request_count = len(transport.requests)
                    namespace["shutdown"]()
                    namespace["shutdown"]()
                    self.assertEqual(len(transport.requests), request_count)
                    self.assertTrue(saved["stop_ack"])
                    self.assertFalse(saved["release_confirmed"])
                    self.assertEqual(saved["release_status"], "unknown")
                    self.assertEqual(namespace["shutdown_proof"]["reason"], saved["reason"])
                    proofs = [json.loads(call.args[0][len("MOTION_SHUTDOWN "):]) for call in output.call_args_list if call.args[0].startswith("MOTION_SHUTDOWN ")]
                    self.assertEqual(len(proofs), 1)
                    self.assertTrue(proofs[0]["worker_stopped"])
                    self.assertTrue(proofs[0]["serial_closed"])
                    self.assertTrue(all(kind not in (namespace["motion"].ARM, namespace["motion"].TARGET) for kind, _ in transport.requests))

        def test_worker_fault_stop_can_confirm_release_before_disconnect(self):
            namespace, factory, _ = self.controller_namespace()
            with tempfile.TemporaryDirectory() as directory:
                transport = factory("fake")
                controller = namespace["CalibrationController"]({}, config_path=Path(directory)/"calibration.json", transport_factory=lambda _: transport)
                controller._connect_once()
                transport.torque, transport.armed = 1, True
                with mock.patch("builtins.print"):
                    controller._run()
                self.assertTrue(namespace["shutdown_proof"]["release_confirmed"])
                self.assertEqual(namespace["shutdown_proof"]["release_status"], "confirmed")
                self.assertTrue(transport.closed)
                controller.close()
                self.assertTrue(namespace["shutdown_proof"]["release_confirmed"])
                self.assertIsNone(namespace["shutdown_proof"]["reason"])

        def test_wrong_or_frozen_release_feedback_is_unknown(self):
            for change in ("frozen", "backward", "offline", "fault"):
                namespace, factory, _ = self.controller_namespace()
                with tempfile.TemporaryDirectory() as directory:
                    transport = factory("fake")
                    controller = namespace["CalibrationController"]({}, config_path=Path(directory)/"calibration.json", transport_factory=lambda _: transport)
                    controller._connect_once()
                    if change == "frozen":
                        transport.freeze_sample = True
                    elif change == "backward":
                        transport.sample_us -= 40000
                        transport.freeze_sample = True
                    elif change == "offline":
                        transport.online = 0
                    else:
                        transport.fault = 1
                    controller.close()
                    self.assertTrue(namespace["shutdown_proof"]["stop_ack"])
                    self.assertFalse(namespace["shutdown_proof"]["release_confirmed"])
                    self.assertEqual(namespace["shutdown_proof"]["release_status"], "unknown")
                    self.assertTrue(transport.closed)

        def test_concurrent_close_and_worker_serialize_one_stop_proof(self):
            import threading
            namespace, factory, _ = self.controller_namespace()
            class StrictTransport(factory):
                closes = 0
                def request(self, kind, payload=b''):
                    if self.closed:
                        raise AssertionError("Request after transport close")
                    return super().request(kind, payload)
                def close(self):
                    self.closes += 1
                    super().close()
            with tempfile.TemporaryDirectory() as directory:
                transport = StrictTransport("fake")
                controller = namespace["CalibrationController"]({}, config_path=Path(directory)/"calibration.json", transport_factory=lambda _: transport)
                controller._connect_once()
                with mock.patch("builtins.print"):
                    controller.start()
                    threads = [threading.Thread(target=controller.close) for _ in range(2)]
                    for thread in threads:
                        thread.start()
                    for thread in threads:
                        thread.join(timeout=2)
                        self.assertFalse(thread.is_alive())
                self.assertFalse(controller._thread.is_alive())
                self.assertEqual(transport.closes, 1)
                self.assertTrue(namespace["shutdown_proof"]["release_confirmed"])
                self.assertTrue(namespace["shutdown_proof"]["worker_stopped"])
                self.assertTrue(all(kind not in (namespace["motion"].ARM, namespace["motion"].TARGET) for kind, _ in transport.requests))

        def test_serial_opens_once_then_settles_without_output(self):
            namespace, _, _ = self.controller_namespace()
            stream = mock.Mock()
            stream.fileno.return_value = 123
            transport = namespace["FeedbackTransport"]("/dev/ttyUSB0")
            def fake_open(instance):
                instance.serial = stream
            with mock.patch.object(namespace["motion"].SerialTransport, "open", fake_open), mock.patch("time.monotonic", side_effect=[100, 100, 100.1, 100.8]), mock.patch("time.sleep"), mock.patch.object(namespace["motion"].SerialTransport, "request") as request:
                transport.open()
                self.assertEqual(namespace["opened_transports"], [transport])
                request.assert_not_called()
            namespace["exit_requested"] = True
            transport = namespace["FeedbackTransport"]("/dev/ttyUSB0")
            with mock.patch.object(namespace["motion"].SerialTransport, "open", fake_open), mock.patch("time.monotonic", return_value=100), mock.patch("time.sleep"):
                with self.assertRaises(RuntimeError):
                    transport.open()
                self.assertIsNone(transport.serial)

    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(Tests))
    return 0 if result.wasSuccessful() else 1


def main():
    global RESTORE_RECORD
    if "--self-test" in sys.argv:
        return self_test()
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--restore-calibration')
    parser.add_argument('--restore-calibration-sha256')
    args, remaining = parser.parse_known_args()
    if bool(args.restore_calibration) != bool(args.restore_calibration_sha256):
        raise ValueError('恢复校准必须同时显式提供备份路径与 SHA256')
    if args.restore_calibration:
        try:
            review_path = remaining[remaining.index('--reviewed-baseline') + 1]
        except (ValueError, IndexError):
            raise ValueError('恢复校准需要当前板子的审核记录')
        review = base.read_regular_json(review_path)
        validate_review(review)
        RESTORE_RECORD = calibration_restore.read_restore_record(args.restore_calibration,
            args.restore_calibration_sha256, review)
    sys.argv = [sys.argv[0], *remaining]
    configure_adapter()
    return base.main()


if __name__ == "__main__":
    sys.exit(main())
