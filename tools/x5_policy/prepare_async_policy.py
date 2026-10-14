"""Assemble a reviewed native candidate from allowlisted source only; no install."""
from pathlib import Path
import hashlib
import json
import shutil
from prepare_policy_deadline import once

HERE=Path(__file__).resolve().parent

def digest(data):return hashlib.sha256(data).hexdigest()

def method(source, name, following):
    return source[source.index('    def '+name+'('):source.index('    '+following,source.index('    def '+name+'('))]

def prepare(source, destination):
    source,destination=Path(source),Path(destination)
    destination.mkdir(parents=True,exist_ok=False)
    for folder in ('policy','service'):shutil.copytree(source/folder,destination/folder)
    # Accept only the inspected deployed baseline or the frozen offline v2.
    original=(source/'policy/production_motion.py').read_bytes()
    baseline_hash = digest(original)
    if baseline_hash not in ('8e735e32d957f9e76dcea2b87f5d3fa1a69b8f65fa1735a94ac2e3af0f447aa1',
            '95013af080e9c24d94861eae21130416143359cf78687853ea2a31ff23ce97e8'):
        raise ValueError('Unexpected native controller baseline')
    common=(HERE/'motion.py').read_text()
    deployed=original.decode()
    if baseline_hash.startswith('8e735'):
        deployed=once(deployed,'class MotionError(ValueError):\n    pass',
            'class MotionError(ValueError):\n    pass\n\n\nclass FeedbackExpired(MotionError):\n    """No command may use this feedback; distinct from a device fault."""\n    pass')
        deployed=once(deployed,'        self._sample_received = None\n',
            '        self._sample_received = None\n        self._late_policy_frames = 0\n        self._last_late_policy_frame = None\n')
        deployed=once(deployed,"raise MotionError('舵机反馈已过期')", "raise FeedbackExpired('舵机反馈已过期')")
        deployed=once(deployed,'                        timing=self._timing_summary_locked(),\n',
            '                        timing=self._timing_summary_locked(),\n'
            '                        policy_deadline=dict(discarded_frames=self._late_policy_frames,\n'
            '                            last_discard=copy.deepcopy(self._last_late_policy_frame),\n'
            '                            observation_limit_ms=FRESH_SECONDS*1000),\n')
    deployed=once(deployed,'import numpy as np\n','import numpy as np\nfrom async_policy import AsyncPolicyMixin\nfrom policy_worker import PolicyWorkerError\n')
    deployed=once(deployed,'class MotionController:', 'class MotionController(AsyncPolicyMixin):')
    deployed=once(deployed,'        self.sessions = sessions\n','        self.sessions = sessions\n        self._policy_worker = None\n        self._policy_job = None\n        self._command_revision = 0\n')
    deployed=once(deployed,'            self._command[:] = np.clip(values,','            self._command_revision += 1\n            self._command[:] = np.clip(values,')
    deployed=once(deployed,'    def _stop_locked(self, reason=None):\n        self._generation += 1','    def _stop_locked(self, reason=None):\n        self._cancel_policy_job()\n        self._generation += 1')
    deployed=once(deployed,'    def _disconnect(self, exc):\n        with self._lock:\n            self._generation += 1','    def _disconnect(self, exc):\n        with self._lock:\n            self._cancel_policy_job()\n            self._generation += 1')
    old=method(deployed,'_infer_policy_target','def _control_once')
    new=method(common,'_prepare_policy_observation','def _control_once')
    deployed=once(deployed,old,new)
    old=method(deployed,'_control_once','@staticmethod')
    new=method(common,'_control_once','@staticmethod').replace('copy.deepcopy(self._filtered)','_copy_policy_state(self._filtered)')
    # Keep native outer/candidate timing wrappers and add bounded production
    # stage timings without copying the old synchronous phase assumptions.
    new=new.replace('    def _control_once(self):','    def _async_control_body(self):',1)
    for anchor,name in [('        with self._lock:\n','entry_lock_wait'),
                        ('        inference_ms, action = None, None\n','snapshot_ready'),
                        ('        with self._lock:\n            # STOP','commit_lock_wait'),
                        ('            if mode != \'shadow\':\n                self._request(TARGET,','target_write')]:
        new=new.replace(anchor,"        self._async_phase_rows.append(('"+name+"', time.perf_counter(), time.thread_time()))\n"+anchor,1)
    # The last anchor is nested; keep its marker at the same indentation.
    new=new.replace("        self._async_phase_rows.append(('target_write'", "            self._async_phase_rows.append(('target_write'")
    new='''    def _control_once(self):
        self._async_phase_rows = []
        try:
            return self._async_control_body()
        finally:
            self._last_production_control_phases = tuple(self._async_phase_rows)

'''+new
    deployed=once(deployed,old,new)
    deployed=once(deployed,'                        policy_deadline=dict(', '                        inference_worker=self._policy_worker.status() if self._policy_worker is not None else None,\n                        policy_deadline=dict(')
    (destination/'policy/production_motion.py').write_text(deployed)
    candidate=(source/'policy/candidate_controller.py').read_text()
    candidate=once(candidate,digest(original),digest(deployed.encode()))
    candidate=once(candidate,'            self._command[:]=np.clip(values,','            self._command_revision += 1\n            self._command[:]=np.clip(values,')
    old=method(candidate,'_infer_policy_target','def _record_policy_target_locked')
    decode=old[old.index('        if raw.shape'):]
    decode='''    def _decode_policy_action(self,selected,raw,previous,smooth,elapsed,inference_ms):
        profile=self._policy_profiles[selected]
'''+decode
    new='''    def _prepare_policy_observation(self,selected,st,previous,command,elapsed,*,head_command=None):
        self._require_research_model(selected)
        if selected!='walk':raise MotionError('Research inference only supports candidate walk')
        return super()._prepare_policy_observation(selected,st,previous,command,elapsed,
            head_command=head_command)

'''+decode
    candidate=once(candidate,old,new)
    (destination/'policy/candidate_controller.py').write_text(candidate)
    runtime=(source/'service/native_runtime.py').read_text()
    if 'timing policy_deadline rates' not in runtime:
        runtime=once(runtime,'imu_us state_request_sequence timing rates','imu_us state_request_sequence timing policy_deadline rates')
    runtime=once(runtime,'        self._persist()\n\n    @property','''        self._persist()
        from policy_worker import PolicyWorker
        self._policy_worker = PolicyWorker(self.directory / 'policy/models', self.deployment['model_hashes'])

    @property''')
    runtime=once(runtime,'                self._generation_floor += 1\n','                controller.attach_policy_worker(self._policy_worker)\n                self._generation_floor += 1\n')
    runtime=once(runtime,'imu_us state_request_sequence timing policy_deadline rates','imu_us state_request_sequence timing policy_deadline inference_worker rates')
    runtime=once(runtime,'                self._closed = True\n            return self.disconnect()\n','''                self._closed = True
            try:
                return self.disconnect()
            finally:
                self._policy_worker.close()
''')
    (destination/'service/native_runtime.py').write_text(runtime)
    for name in ('policy_worker.py','async_policy.py'):
        shutil.copyfile(HERE/name,destination/'policy'/name)
    bootstrap=(source/'service/bootstrap.py').read_text()
    bootstrap=once(bootstrap,'import native_calibration_store as calibration_storage,native_transports,native_process','import native_calibration_store as calibration_storage,native_transports,native_process,policy_worker')
    bootstrap=once(bootstrap,'native_process.allowed_web_command(args[0],args[1],root)):',
        'native_process.allowed_web_command(args[0],args[1],root)) and not (\n                policy_worker.allowed_worker_command(args[0],args[1],root)):')
    (destination/'service/bootstrap.py').write_text(bootstrap)
    changed={}
    for path in sorted(destination.rglob('*.py')):
        raw=path.read_bytes();compile(raw,str(path),'exec')
        rel=path.relative_to(destination).as_posix()
        if not (source/rel).exists() or raw!=(source/rel).read_bytes():changed[rel]=digest(raw)
    (destination/'async-manifest.json').write_text(json.dumps(dict(changed=changed,installed=False,
        motor_access=False,calibration_changed=False,thresholds_changed=False),indent=2)+'\n')
    return changed

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('source');parser.add_argument('destination')
    args=parser.parse_args();print(json.dumps(prepare(args.source,args.destination),indent=2))
