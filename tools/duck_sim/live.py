"""Read-only SSH feedback session for Duck Sim Lab. Credentials stay in SSH."""
import collections
import copy
import json
import math
import os
from pathlib import Path
import re
import subprocess
import threading
import time

import numpy as np
from rl_core import ImuFilter, State, POLICY_INDEX, MOTOR_IDS


class LiveFeedback:
    def __init__(self):
        self.target = os.environ.get('XGO_X5_TARGET', 'root@192.168.1.11')
        if not re.fullmatch(r'[a-zA-Z_][a-zA-Z0-9_-]*@[a-zA-Z0-9.:-]+', self.target):
            raise ValueError('无效 SSH 目标')
        self.control_path = os.environ.get('XGO_X5_CONTROL_PATH')
        self.lock = threading.RLock()
        self.process = None
        self.generation = 0
        self.result = None
        self.error = None
        self.received = None
        self.cfg = None
        self.filter = ImuFilter()
        self.last_sample = None
        self.times = collections.deque(maxlen=90)
        self.touched = time.monotonic()
        self.min_clock_offset = None
        threading.Thread(target=self._lease, daemon=True).start()

    def start(self):
        with self.lock:
            self.touched = time.monotonic()
            if self.process and self.process.poll() is None:
                return self.snapshot()
            self.generation += 1
            generation = self.generation
            self.result = self.received = self.cfg = self.error = self.last_sample = self.min_clock_offset = None
            self.filter = ImuFilter(); self.times.clear()
            command = ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
                       '-o', 'StrictHostKeyChecking=yes', '-o', 'ServerAliveInterval=2', '-o', 'ServerAliveCountMax=2']
            if self.control_path:
                command.extend(['-S', self.control_path])
            command.extend([self.target, 'python3 -u -'])
            self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                            stderr=subprocess.PIPE, text=True, bufsize=1)
            process = self.process
            process.stdin.write(Path(__file__).with_name('live_reader.py').read_text())
            process.stdin.close()
            threading.Thread(target=self._read, args=(process,generation), daemon=True).start()
            return self.snapshot()

    def stop(self):
        with self.lock:
            self.generation += 1
            process = self.process
            self.process = None
            self.error = None
        if process and process.poll() is None:
            process.terminate()
            try: process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait(timeout=1)
        return self.snapshot()

    def _lease(self):
        while True:
            time.sleep(1)
            with self.lock:
                expired = self.process is not None and time.monotonic()-self.touched > 6
            if expired:
                self.stop()

    def _read(self, process, generation):
        try:
            for line in process.stdout:
                if len(line)>65536: raise ValueError('实时反馈消息过长')
                row = json.loads(line)
                with self.lock:
                    if generation != self.generation: break
                    if row['type']=='calibration':
                        self.cfg=row['calibration'];self.calibration_sha=row['sha256'];self.revision=row['revision']
                        self._validate_calibration()
                    elif row['type']=='error':
                        self.error=row['message']
                    elif row['type']=='state':
                        self._consume(row)
            error = process.stderr.read(4096).strip()
            with self.lock:
                if generation==self.generation:
                    self.error=self.error or ('SSH 连接结束：'+error if error else '反馈连接已结束，请重新连接')
        except Exception as exc:
            with self.lock:
                if generation==self.generation:self.error=str(exc)
            if process.poll() is None: process.terminate()

    def _validate_calibration(self):
        cfg=self.cfg
        if [j['id'] for j in cfg['joints']]!=MOTOR_IDS:raise ValueError('实时校准关节顺序不符')
        axes=cfg['imu']['axes']
        if len(axes)!=3 or set(a.lstrip('-') for a in axes)!=set('xyz'):raise ValueError('实时 IMU 轴映射无效')
        self.matrix=np.array([[(-1 if a.startswith('-') else 1) if a.lstrip('-')==v else 0 for v in 'xyz'] for a in axes])
        if round(np.linalg.det(self.matrix))!=1:raise ValueError('IMU 轴映射不是右手坐标系')

    def _consume(self,row):
        if not self.cfg:raise ValueError('尚未收到保存校准')
        now=time.monotonic()
        offset=now-row['remote_time']
        self.min_clock_offset=offset if self.min_clock_offset is None else min(self.min_clock_offset,offset)
        backlog_ms=max(0,(offset-self.min_clock_offset)*1000)
        cfg=self.cfg
        angles=[(p-j['zero'])*360/4095*j['sign'] for p,j in zip(row['positions'],cfg['joints'])]
        gravity=None
        required=(row['online_mask']&0x3fff)==0x3fff
        imu_ready=row['imu_ok'] and cfg['imu']['axes_confirmed'] and cfg['imu']['calibrated']
        if required and imu_ready:
            dt=.033 if self.last_sample is None else ((row['sample_us']-self.last_sample)&0xffffffff)/1e6
            acc=self.matrix@np.asarray(row['acc'])
            gyro=self.matrix@(np.asarray(row['gyro'])-np.asarray(cfg['imu']['bias_dps']))
            if row['sample_us'] != self.last_sample:
                value=self.filter.update(State([acc[1],acc[2],-acc[0]],[-gyro[1],-gyro[2],gyro[0]],angles,[0]*15),dt)
                self.gravity=value.grav;self.last_sample=row['sample_us']
            gravity=getattr(self,'gravity',None)
        self.times.append(now);self.received=now
        online=[sid for i,sid in enumerate(MOTOR_IDS) if row['online_mask']&(1<<i)]
        self.result=dict(sequence=row['sequence'], sample_us=row['sample_us'],
                         q=[angles[i] for i in POLICY_INDEX] + [angles[14] if row['online_mask'] & (1<<14) else None],
                         gravity=gravity,pose_valid=required and gravity is not None,
                         online_ids=online,torque_ids=[sid for i,sid in enumerate(MOTOR_IDS) if row['torque_mask']&(1<<i)],
                         armed=row['armed'],imu_ok=imu_ready,fault_mask=row['fault_mask'],firmware_error=row['firmware_error'],
                         sample_unchanged_ms=row['sample_unchanged_ms'],imu_unchanged_ms=row['imu_unchanged_ms'],
                         uart_rtt_ms=row['uart_rtt_ms'],backlog_ms=backlog_ms,
                         calibration_sha256=self.calibration_sha,calibration_revision=self.revision)

    def snapshot(self,touch=False):
        with self.lock:
            now=time.monotonic()
            if touch:self.touched=now
            active=self.process is not None and self.process.poll() is None
            age=(now-self.received)*1000 if self.received is not None else None
            fresh=bool(active and not self.error and self.result and self.result['pose_valid'] and
                       age+max(self.result['sample_unchanged_ms'],self.result['imu_unchanged_ms'])+self.result['backlog_ms']<500)
            hz=(len(self.times)-1)/(self.times[-1]-self.times[0]) if len(self.times)>1 and self.times[-1]>self.times[0] else 0
            return dict(active=active,fresh=fresh,status='live' if fresh else 'connecting' if active and not self.result else 'stale' if active else 'disconnected',
                        target=self.target,receive_age_ms=age,rate_hz=hz,error=self.error,feedback=copy.deepcopy(self.result),
                        protocol='STATE only',commands_sent={'ARM':0,'TARGET':0,'STOP':0,'CONFIG':0})
