"""Generate an isolated deferred scheduler from hash-pinned v2; never deploy."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

BASE={
 'policy/production_motion.py':'edbb8f1a9d59e0c80317445c3679b901417b29a4a29c16abd026c7f7676aad6f',
 'policy/serial_owner.py':'139f9b45e6e617bf15ed585dc573fa18b8483df0a6d4ed0725d83ace4d6a911c',
 'policy/serial_owner_controller.py':'eca65e27dfe88157a2bfbfc83da90f9304189c31d2d0f1c4a38ccdaa368a3bf8',
 'policy/policy_worker.py':'d2e83347d1e9f8711dbedeadc4a02af5512f144e3632c4249f6f2d7b24fb14e6',
 'policy/candidate_controller.py':'d6fb18a7121dcaaed06331a2a9b77f6e3e890a2edee196f86a0f77d2cf14d4b8'}

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def once(text,before,after):
    if text.count(before)!=1:raise ValueError('Inspected source anchor changed: '+before[:70])
    return text.replace(before,after,1)

def prepare(source,destination):
    for name,digest in BASE.items():
        if sha(source/name)!=digest:raise ValueError('Unreviewed source '+name)
    if destination.exists():raise FileExistsError(destination)
    shutil.copytree(source,destination,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    def save(name,text):
        compile(text,name,'exec');(destination/name).write_text(text)
    helper=Path(__file__).with_name('deferred_control.py')
    shutil.copyfile(helper,destination/'policy/deferred_control.py')
    value=(source/'policy/production_motion.py').read_text()
    value=once(value,'from serial_owner_controller import SerialOwnerControllerMixin',
               'from serial_owner_controller import SerialOwnerControllerMixin\nfrom deferred_control import DeferredControlMixin')
    value=once(value,'class MotionController(SerialOwnerControllerMixin, AsyncPolicyMixin):',
               'class MotionController(DeferredControlMixin, SerialOwnerControllerMixin, AsyncPolicyMixin):')
    value=once(value,'            generation, mode = self._generation, self._mode\n            if self._serial_owner_enabled and self._io_pending is not None:\n                return',
               '            generation, mode = self._generation, self._mode\n            self._io_deferred_context = None\n            if self._serial_owner_enabled and self._io_pending is not None:\n                self._io_defer_control(self._policy_context_locked())\n                return')
    value=once(value,'                if ready is None:\n                    return\n                frame, target, action, smooth, inference_ms = ready',
               '                if ready is None:\n                    self._io_defer_control(context)\n                    return\n                frame, target, action, smooth, inference_ms = ready')
    save('policy/production_motion.py',value)
    value=(source/'policy/candidate_controller.py').read_text()
    save('policy/candidate_controller.py',once(value,BASE['policy/production_motion.py'],sha(destination/'policy/production_motion.py')))
    value=(source/'policy/policy_worker.py').read_text()
    method='''    def result_ready(self, request_id):
        """Non-consuming, nonblocking readiness for the control scheduler."""
        import select
        if not self._lock.acquire(blocking=False):
            return False
        try:
            self._alive()
            if self._pending != request_id or self._abandoned:
                raise PolicyWorkerError('Inference request no longer belongs to caller')
            return bool(select.select([self._socket], [], [], 0)[0])
        finally:
            self._lock.release()

'''
    value=once(value,'    def take(self, request_id):\n',method+'    def take(self, request_id):\n')
    save('policy/policy_worker.py',value)
    value=(source/'policy/serial_owner.py').read_text()
    value=once(value,'        self.next_poll = 0.\n','        self.next_poll = 0.\n        self.last_target_write_finished = None\n')
    value=once(value,"            ticket.transmitted = True\n            self.metrics['target_write_attempted'] += 1\n            yield",
'''            if (self.last_target_write_finished is not None and
                    time.monotonic() < self.last_target_write_finished + self.period):
                raise OwnerError('TARGET reached write guard before minimum write interval')
            ticket.transmitted = True
            self.metrics['target_write_attempted'] += 1
            try:
                yield
            finally:
                # Time after the nonblocking write, not enqueue or ACK time.
                # Partial/failed writes still consume the interval and fault.
                self.last_target_write_finished = time.monotonic()''')
    value=once(value,'''                    if self.target is not None and not self.target.done.is_set():
                        ticket=self.target;break
                    now=time.monotonic()
                    poll_at=max(self.next_poll,self.revalidation_until)''',
'''                    now=time.monotonic()
                    target_due=None
                    if self.target is not None and not self.target.done.is_set():
                        target_due=(now if self.last_target_write_finished is None else
                                    self.last_target_write_finished+self.period)
                        if now>=target_due:
                            ticket=self.target;break
                    poll_at=max(self.next_poll,self.revalidation_until)''')
    value=once(value,'                    timeout=max(0,poll_at-now) if self.sampling else None\n                    self.cv.wait(timeout)',
'''                    deadlines=([poll_at] if self.sampling else [])
                    if target_due is not None:deadlines.append(target_due)
                    timeout=max(0,min(deadlines)-now) if deadlines else None
                    self.cv.wait(timeout)''')
    save('policy/serial_owner.py',value)
    changed={str(p.relative_to(destination)):sha(p) for p in destination.rglob('*.py')
             if not (source/p.relative_to(destination)).exists() or sha(p)!=sha(source/p.relative_to(destination))}
    manifest=dict(kind='deferred_control_candidate',candidate_name=destination.name,base=BASE,changed=changed,
                  deployed=False,hardware_access=False,source=str(source.resolve()),
                  admission='same complete control path; context-bound deferred cycles only',
                  target_write_interval_s=.02,freshness_s=.1,watchdog_s=.25,
                  invariant='latest STATE write fence, original action deadline, ACK-only recurrence, STOP priority retained')
    (destination/'deferred-control-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',type=Path);parser.add_argument('destination',type=Path)
    a=parser.parse_args();print(json.dumps(prepare(a.source,a.destination),indent=2))
