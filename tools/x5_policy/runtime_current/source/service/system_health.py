"""Passive Linux telemetry. The launcher samples; HTTP only reads RAM files.

No UART, controller RPC, sysfs writes, governor changes or motion decisions.
Linux units/semantics: admin-guide/pm/cpufreq and driver-api/thermal/sysfs-api.
"""
from collections import deque
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import stat
import threading
import time

PERIOD = 2.0
STALE_SECONDS = 6.0
HISTORY_LIMIT = 300
MAX_FILE = 2 * 1024 * 1024
LOG_LIMIT = 4 * 1024 * 1024


def text(path):
    try:
        with open(path) as f:
            return f.read(8192).strip()
    except (OSError, UnicodeError):
        return None


def number(path, scale=1, minimum=0, maximum=10**12):
    try:
        value = int(text(path)) / scale
        return value if minimum <= value <= maximum else None
    except (ValueError, TypeError):
        return None


def read_json(path):
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    with os.fdopen(fd, 'rb') as f:
        node = os.fstat(f.fileno())
        if (not stat.S_ISREG(node.st_mode) or node.st_uid != os.getuid()
                or node.st_mode & 0o022 or node.st_size > MAX_FILE):
            raise ValueError('Invalid telemetry file')
        raw = f.read(MAX_FILE + 1)
    if len(raw) > MAX_FILE:
        raise ValueError('Telemetry file exceeds bound')
    return json.loads(raw)


def classify(zones, policies, cooling):
    cpu_cooling = [c for c in cooling if c['type'].startswith('cpufreq-')]
    if any((c['state'] or 0) > 0 for c in cpu_cooling):
        return 'thermal_limited', '内核 CPU 热限速已启用；请结合运控耗时排查散热'
    if any(p['max_mhz'] is not None and p['hardware_max_mhz'] is not None
           and p['max_mhz'] < p['hardware_max_mhz'] for p in policies):
        return 'frequency_capped', 'CPU 频率上限受限；尚不能确定由温度引起'
    if any(z['celsius'] is not None and any(t['celsius'] is not None
           and t['type'] in ('passive', 'hot', 'critical')
           and z['celsius'] >= t['celsius'] for t in z['trips']) for z in zones):
        return 'trip_reached', '温度已达到内核热管理阈值；检查散热与限速状态'
    cpu_zones = [z for z in zones if 'cpu' in z['type'].lower()]
    if (not cpu_zones or any(z['celsius'] is None for z in cpu_zones)
            or not cpu_cooling or any(c['state'] is None for c in cpu_cooling)
            or not policies or any(p['max_mhz'] is None or p['hardware_max_mhz'] is None
                                  or p['current_mhz'] is None for p in policies)):
        return 'unknown', '监控指标不完整，无法判断是否热限速'
    return 'no_limit_observed', '本次采样未见 CPU 热限速或频率上限限制'


class LinuxSampler:
    def __init__(self, sys_root='/sys', proc_root='/proc'):
        self.sys, self.proc = Path(sys_root), Path(proc_root)
        self.previous_cpu = None

    def sample(self):
        zones, cooling, policies = [], [], []
        for node in sorted((self.sys / 'class/thermal').glob('thermal_zone*'))[:32]:
            trips = []
            for p in sorted(node.glob('trip_point_*_temp'))[:32]:
                stem = p.name[:-5]
                trips.append(dict(type=text(node / (stem + '_type')),
                                  celsius=number(p, 1000, -40, 200)))
            zones.append(dict(name=node.name, type=text(node / 'type') or node.name,
                celsius=number(node / 'temp', 1000, -40, 200), trips=trips))
        for node in sorted((self.sys / 'class/thermal').glob('cooling_device*'))[:64]:
            cooling.append(dict(name=node.name, type=text(node / 'type') or node.name,
                state=number(node / 'cur_state'), max_state=number(node / 'max_state')))
        for node in sorted((self.sys / 'devices/system/cpu/cpufreq').glob('policy*'))[:64]:
            current = number(node / 'cpuinfo_cur_freq', 1000, 1, 10000)
            source = 'cpuinfo_cur_freq'
            if current is None:
                current, source = number(node / 'scaling_cur_freq', 1000, 1, 10000), 'scaling_cur_freq'
            policies.append(dict(name=node.name, cpus=text(node / 'affected_cpus'),
                current_mhz=current, current_source=source, governor=text(node / 'scaling_governor'),
                max_mhz=number(node / 'scaling_max_freq', 1000, 1, 10000),
                hardware_max_mhz=number(node / 'cpuinfo_max_freq', 1000, 1, 10000)))
        utilization = None
        try:
            fields = (text(self.proc / 'stat') or '').splitlines()[0].split()
            if fields[0] != 'cpu':
                raise ValueError('CPU counters unavailable')
            ticks = list(map(int, fields[1:9]))  # guest is already included in user/nice
            total, idle = sum(ticks), ticks[3] + ticks[4]
            if self.previous_cpu is not None:
                dt, di = total - self.previous_cpu[0], idle - self.previous_cpu[1]
                if dt > 0 and 0 <= di <= dt:
                    utilization = round(100 * (dt - di) / dt, 1)
            self.previous_cpu = total, idle
        except (ValueError, IndexError):
            self.previous_cpu = None
        state, reason = classify(zones, policies, cooling)
        return dict(zones=zones, policies=policies, cooling=cooling,
                    cpu_busy_percent=utilization, state=state, reason=reason)


def control_context(path, now):
    """Whitelist a published display packet; no model, UART or IPC access."""
    try:
        packet = read_json(path)
        age = (now - packet['published_at']) * 1000
        if not math.isfinite(age) or age < 0:
            raise ValueError('Invalid controller publication time')
        m = packet['snapshot']['motion']; h = m.get('hardware') or {}
        timing = h.get('timing') or {}; deadline = h.get('policy_deadline') or {}
        stop = (h.get('candidate_profile') or {}).get('last_trial_stop') or {}
        return dict(available=age <= 1000, snapshot_age_ms=round(age, 1),
            phase=m.get('phase'), mode=h.get('mode'), generation=h.get('generation'),
            feedback_age_ms=h.get('age_ms'), inference_ms=h.get('inference_ms'),
            cycle_max_ms=(timing.get('cycle') or {}).get('max_ms'),
            state_request_max_ms=(timing.get('state_request') or {}).get('max_ms'),
            discarded_policy_frames=deadline.get('discarded_frames'),
            last_trial_stop={k:stop[k] for k in ('outcome','reason','recorded_monotonic') if k in stop},
            fault=str(m.get('error') or h.get('error') or '')[:512])
    except (OSError, ValueError, TypeError, KeyError):
        return dict(available=False)


def atomic_json(path, value):
    raw = json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':')).encode()
    if len(raw) > MAX_FILE:
        raise ValueError('Health history exceeds bound')
    tmp = path.with_suffix('.tmp')
    fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, 'wb') as f:
        f.write(raw)
    os.replace(tmp, path)


class HealthMonitor:
    def __init__(self, root, directory, *, sampler=None):
        self.root, self.directory = Path(root), Path(directory)
        self.sampler = sampler or LinuxSampler()
        self.stop = threading.Event()
        self.thread = None
        self.history = deque(maxlen=HISTORY_LIMIT)
        self.peak = None
        self.limited_samples = 0
        self.last_limited = None
        self.previous_event = None
        self.last_log = -math.inf
        self.boot_id = text('/proc/sys/kernel/random/boot_id')
        self.started_at = time.time()

    def append_log(self, value):
        self.directory.mkdir(mode=0o700, parents=True, exist_ok=True)
        node = self.directory.lstat()
        if not stat.S_ISDIR(node.st_mode) or node.st_uid != os.getuid() or node.st_mode & 0o077:
            raise ValueError('Health log directory must be private')
        path = self.directory / 'samples.jsonl'
        if path.exists() and path.stat().st_size >= LOG_LIMIT:
            for i in (2, 1):
                old = self.directory / ('samples.jsonl' if i == 1 else 'samples.1.jsonl')
                if old.exists():
                    os.replace(old, self.directory / ('samples.%d.jsonl' % i))
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND | os.O_NOFOLLOW, 0o600)
        with os.fdopen(fd, 'w') as f:
            f.write(json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':')) + '\n')

    def tick(self):
        start = time.monotonic(); cpu_start = time.thread_time()
        value = self.sampler.sample()
        now = time.monotonic()
        value.update(protocol=1, sampled_monotonic=now, sampled_at=time.time(),
            sampled_utc=datetime.now(timezone.utc).isoformat(), boot_id=self.boot_id,
            sample_period_s=PERIOD, monitor_started_at=self.started_at,
            control=control_context(self.root / 'web-state.json', now))
        temps = [z['celsius'] for z in value['zones'] if 'cpu' in z['type'].lower() and z['celsius'] is not None]
        if temps:
            self.peak = max(temps + ([self.peak] if self.peak is not None else []))
        if value['state'] == 'thermal_limited':
            self.limited_samples += 1
            self.last_limited = value['sampled_utc']
        value.update(peak_cpu_celsius=self.peak, thermal_limited_samples=self.limited_samples,
            last_thermal_limited_utc=self.last_limited, recording_error=None)
        event = (value['state'], value['control'].get('mode'), value['control'].get('generation'),
                 value['control'].get('fault'), str(value['control'].get('last_trial_stop')))
        # Durable records: every sample during motion/thermal limiting; 10s at
        # idle, and immediately on observed state changes. Bounded to 3 files.
        active = value['control'].get('mode') not in (None, 'off') or value['state'] != 'no_limit_observed'
        if event != self.previous_event or now - self.last_log >= (PERIOD if active else 10):
            try:
                self.append_log(value)
                self.last_log, self.previous_event = now, event
            except (OSError, ValueError) as error:
                value['recording_error'] = type(error).__name__ + ': ' + str(error)[:160]
        value.update(sample_wall_ms=round((time.monotonic()-start)*1000, 2),
                     sample_cpu_ms=round((time.thread_time()-cpu_start)*1000, 2))
        # Compact recent history keeps RAM publication and downloads bounded.
        self.history.append({k:value[k] for k in ('sampled_at','sampled_monotonic','state',
            'cpu_busy_percent','control','sample_wall_ms','sample_cpu_ms')} | dict(
            cpu_celsius=max(temps) if temps else None,
            frequency_mhz=[p['current_mhz'] for p in value['policies']],
            cap_mhz=[p['max_mhz'] for p in value['policies']],
            cpu_cooling_states=[c['state'] for c in value['cooling'] if c['type'].startswith('cpufreq-')]))
        atomic_json(self.root / 'system-health.json', value)
        atomic_json(self.root / 'system-health-history.json', dict(protocol=1,
            boot_id=self.boot_id, sample_period_s=PERIOD, samples=list(self.history)))

    def run(self):
        # Only this Linux thread gets lower scheduling priority; never the
        # launcher/controller or its subsequently spawned processes.
        try:
            os.setpriority(os.PRIO_PROCESS, threading.get_native_id(), 10)
        except (AttributeError, OSError):
            pass
        while not self.stop.is_set():
            start = time.monotonic()
            try:
                self.tick()
            except Exception:
                # A failed monitor cannot terminate a controller or release
                # torque. The page marks its old publication stale after 6s.
                pass
            self.stop.wait(max(.1, PERIOD - (time.monotonic()-start)))

    def start(self):
        self.thread = threading.Thread(target=self.run, name='x5-health', daemon=True)
        self.thread.start()

    def close(self):
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=1)


def snapshot(root, *, now=None):
    try:
        value = read_json(Path(root) / 'system-health.json')
        age = (time.monotonic() if now is None else now) - value['sampled_monotonic']
        if not math.isfinite(age) or age < 0:
            raise ValueError('Invalid telemetry timestamp')
        value.update(age_ms=round(age * 1000, 1), stale=age > STALE_SECONDS)
        if value['stale']:
            value.update(state='stale', reason='监控数据已过期，当前热限速状态未知')
        return value
    except (OSError, ValueError, TypeError, KeyError):
        return dict(protocol=1, state='unavailable', stale=True, reason='等待 X5 温度与性能监控数据')


def toolbar(page):
    panel = Path(__file__).with_name('system_health_panel.html').read_text()
    marker = '<div class="capability-note" id="walkModelTest"'
    html = page.decode()
    if html.count(marker) != 1:
        raise ValueError('Health panel insertion point changed')
    return html.replace(marker, panel + '\n' + marker, 1).encode()
