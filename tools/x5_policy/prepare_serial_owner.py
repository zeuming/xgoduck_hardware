"""Build an isolated, pinned UART-owner candidate. Never opens any device."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import shutil

HERE=Path(__file__).resolve().parent
BASE={'policy/production_motion.py': '7cc465af67ed32ebb5b2ae5666d875e9f257958e5951379739d779995ca22abc', 'policy/motion.py': '19be7c6e1d9fc801d8678cfef73f42e0b637c68032166e3d2cdd177abf6d752d', 'policy/candidate_controller.py': '5a58e1126a93b8e71cff605c3be9c964b3aae7d14b99a2dd10b723834b6d6586', 'policy/current_pose_probe.py': 'aec3688553383b18f1dfe648ec3d0112317eb0a2f8e1a854ff2cb8f374267c1d', 'service/worker-classes.py': 'a1ed11c2a7f203794ca60379e99afed14837b31e8dc362fa53fdfee0fa106f22', 'service/native_runtime.py': 'beeba370911b65d89f64ea91ce438d4c6a5f52e48c8805686fa86e7a3f128835'}

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def once(value,old,new):
    if value.count(old)!=1:raise ValueError('Inspected source anchor changed: '+old[:90])
    return value.replace(old,new,1)
def prepare(source,destination):
    source,destination=Path(source),Path(destination)
    for name,expected in BASE.items():
        if sha(source/name)!=expected:raise ValueError('Unreviewed source: '+name)
    if destination.exists():raise ValueError('Destination must be new')
    shutil.copytree(source,destination,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    def save(name,value):
        compile(value,name,'exec');(destination/name).write_text(value)
    p=source/'policy/production_motion.py';value=p.read_text()
    value=once(value,'import collections\n','import collections\nfrom contextlib import nullcontext\nfrom serial_owner_controller import SerialOwnerControllerMixin\n')
    value=once(value,'class SerialTransport:\n','class SerialTransport:\n    supports_target_write_guard = True\n')
    value=once(value,"        written = (os.write(self.serial.fileno(), frame) if self._posix_write\n                   else self.serial.write(frame))",
        "        guard = getattr(self, '_request_write_guard', None)\n        with guard(kind) if guard is not None else nullcontext():\n            written = (os.write(self.serial.fileno(), frame) if self._posix_write\n                       else self.serial.write(frame))")
    value=once(value,'class MotionController(AsyncPolicyMixin):','class MotionController(SerialOwnerControllerMixin, AsyncPolicyMixin):')
    value=once(value,'                 allow_fixed_head_walk=False, hold_hip_pitch_bias_deg=0):','                 allow_fixed_head_walk=False, hold_hip_pitch_bias_deg=0, serial_owner=True):')
    value=once(value,'        self._clock = clock\n','        self._clock = clock\n        self._io_initialize(serial_owner)\n')
    value=once(value,"    def _request_transport(self, transport, kind, payload=b''):\n",
        "    def _request_transport(self, transport, kind, payload=b''):\n        if self._serial_owner_enabled:\n            result = transport.request(kind, payload)\n            if kind in (STATE, VOLTAGE_STATE):\n                self._state_request_sequence = transport.last_sync_state_sequence\n            return result\n")
    value=once(value,'        return decode_state(payload)\n',"        state = decode_state(payload)\n        if self._serial_owner_enabled:\n            state['_host_received_at'] = self._transport.last_sync_state_received\n        return state\n")
    value=once(value,'            state = decode_state(payload)\n            with self._lock:',
        "            state = decode_state(payload)\n            if self._serial_owner_enabled:\n                state['_host_received_at'] = transport.last_sync_state_received\n            with self._lock:")
    value=once(value,'                self._apply_config_locked()\n        except Exception:',
        '                self._apply_config_locked()\n                if self._serial_owner_enabled:\n                    transport.start_sampling(state_request, state_payload)\n        except Exception:')
    value=once(value,'        now = self._clock()\n        self._state_receive_times.append(now)',
        "        now = state.get('_host_received_at', self._clock())\n        if not math.isfinite(now) or now > self._clock():\n            raise MotionError('Invalid UART receive timestamp')\n        self._state_receive_times.append(now)")
    # Every generation mutation invalidates pending TX before its atomic write.
    value=value.replace('self._generation += 1','self._io_cancel_targets_locked()\n'+ 'INDENT_TOKENself._generation += 1')
    lines=value.splitlines()
    for i,line in enumerate(lines):
        if 'INDENT_TOKEN' in line:
            lines[i]=lines[i-1][:len(lines[i-1])-len(lines[i-1].lstrip())]+line.replace('INDENT_TOKEN','')
    value='\n'.join(lines)+'\n'
    value=once(value,'            self._command_revision += 1','            self._io_cancel_targets_locked()\n            self._command_revision += 1')
    value=once(value,'            self._closed.wait(max(.001, PERIOD - (self._clock() - started)))',
        '            self._io_wait_until(max(started + PERIOD, self._clock() + .001))')
    value=once(value,'    def _cycle_once(self):\n        with self._lock:',
        '    def _cycle_once(self):\n        error = getattr(self, "_io_event_error", None)\n        if error is not None:\n            self._io_event_error = None\n            raise error\n        self._io_drain_ack()\n        with self._lock:')
    value=once(value,'            state = self._read_state()\n            self._accept_state_locked(state)\n            if self._mode == \'off\':',
        '            state = self._io_cycle_state_locked()\n            if self._mode == \'off\':')
    value=once(value,'            generation, mode = self._generation, self._mode\n            if self._policy_job',
        '            generation, mode = self._generation, self._mode\n            if self._serial_owner_enabled and self._io_pending is not None:\n                return\n            if self._policy_job')
    value=once(value,"            if mode != 'shadow':\n                self._request(TARGET, struct.pack('<I15H', self._cookie, *raw))",
        "            if mode != 'shadow':\n                if self._serial_owner_enabled:\n                    self._io_submit_target_locked(struct.pack('<I15H', self._cookie, *raw),\n                        context=context, target=target, execution_target=execution_target,\n                        action=action, smooth=smooth, inference_ms=inference_ms,\n                        latest_frame=latest_frame, observation_received=observation_received)\n                    return\n                self._request(TARGET, struct.pack('<I15H', self._cookie, *raw))")
    save('policy/production_motion.py',value)
    value=(source/'policy/candidate_controller.py').read_text()
    value=once(value,'                 hold_hip_pitch_bias_deg=0):','                 hold_hip_pitch_bias_deg=0,serial_owner=True):')
    value=once(value,'require_head_clearance=True,allow_fixed_head_walk=research_walk_enabled,hold_hip_pitch_bias_deg=0)',
        'require_head_clearance=True,allow_fixed_head_walk=research_walk_enabled,hold_hip_pitch_bias_deg=0,serial_owner=serial_owner)')
    value=once(value,'            self._command_revision += 1','            self._io_cancel_targets_locked()\n            self._command_revision += 1')
    value=once(value,"            if (mode=='walk' and self._mode=='walk' and generation==self._generation",
        "            if (not self._serial_owner_enabled and mode=='walk' and self._mode=='walk' and generation==self._generation")
    value=once(value,BASE['policy/production_motion.py'],sha(destination/'policy/production_motion.py'))
    save('policy/candidate_controller.py',value)
    value=(source/'policy/motion.py').read_text()
    value=once(value,'                 hold_hip_pitch_bias_deg=0):','                 hold_hip_pitch_bias_deg=0, serial_owner=True):')
    value=once(value,'research_nonzero_command=False, hold_hip_pitch_bias_deg=hold_hip_pitch_bias_deg)',
        'research_nonzero_command=False, hold_hip_pitch_bias_deg=hold_hip_pitch_bias_deg, serial_owner=serial_owner)')
    value=once(value,'            self._forward.begin(self._clock(),self._generation)',
        '            self._io_cancel_targets_locked()\n            self._forward.begin(self._clock(),self._generation)')
    value=once(value,'    def _return_walk_to_hold_locked(self, reason, *, page_lost=False):\n',
        '    def _return_walk_to_hold_locked(self, reason, *, page_lost=False):\n        self._io_cancel_targets_locked()\n')
    save('policy/motion.py',value)
    value=(source/'policy/current_pose_probe.py').read_text()
    old="        def _request(self, kind, payload=b''):\n            if kind == target_kind and self._probe_active():"
    value=once(value,old,"        def _validate_async_target_locked(self, payload):\n            super()._validate_async_target_locked(payload)\n            self._validate_probe_target(payload)\n\n        def _validate_probe_target(self, payload):\n            if self._probe_active():")
    value=once(value,'            return super()._request(kind, payload)\n',
        "\n        def _request(self, kind, payload=b''):\n            if kind == target_kind:\n                self._validate_probe_target(payload)\n            return super()._request(kind, payload)\n")
    save('policy/current_pose_probe.py',value)
    for name in ('serial_owner.py','serial_owner_controller.py'):
        shutil.copyfile(HERE/name,destination/'policy'/name)
    # The production OneShot subclass overrides _run, so patch both entrypoints.
    value=(source/'service/worker-classes.py').read_text()
    value=once(value,"    def _stop_locked(self, reason=None):\n        before = super().status() if self._mode != 'off' else None\n",
        "    def _stop_locked(self, reason=None):\n        self._io_cancel_targets_locked()\n        try:\n            before = super().status() if self._mode != 'off' else None\n        except Exception as error:\n            before = dict(diagnostic_error=str(error))\n")
    value=once(value,"        transport_diagnostic = timing() if before is not None and callable(timing) else None",
        "        try:\n            transport_diagnostic = timing() if before is not None and callable(timing) else None\n        except Exception as error:\n            transport_diagnostic = dict(diagnostic_error=str(error))")
    value=once(value,'                self._closed.wait(max(0.001, motion.PERIOD - (self._clock() - began)))',
        '                self._io_wait_until(max(began + motion.PERIOD, self._clock() + .001))')
    save('service/worker-classes.py',value)
    value=(source/'service/native_runtime.py').read_text()
    value=once(value,'                    hold_hip_pitch_bias_deg=self._hold_hip_pitch_bias_deg)',
        '                    hold_hip_pitch_bias_deg=self._hold_hip_pitch_bias_deg, serial_owner=True)')
    value=once(value,'        return (controller._transport is None and controller._connected is False',
        '        owner = getattr(controller, "_io_owner", None)\n        return ((owner is None or (owner.finished and not owner.thread.is_alive()))\n                and controller._transport is None and controller._connected is False')
    save('service/native_runtime.py',value)
    # All production controllers default on; assert no emitted source disables
    # the owner. CPU fixtures must opt out explicitly, outside this candidate.
    for path in destination.rglob('*.py'):
        tree=ast.parse(path.read_text())
        for node in ast.walk(tree):
            if isinstance(node,ast.keyword) and node.arg=='serial_owner' and isinstance(node.value,ast.Constant) and node.value.value is False:
                raise ValueError('Production entry disables UART owner: '+str(path))
    changed={str(p.relative_to(destination)):sha(p) for p in destination.rglob('*.py')
             if not (source/p.relative_to(destination)).exists() or sha(p)!=sha(source/p.relative_to(destination))}
    manifest=dict(kind='single_uart_owner_candidate',hardware_access=False,deployed=False,
        default_enabled=True,revalidation_priority_window='one existing 20ms period; never renewed for the same untransmitted target',
        source=str(source.resolve()),baseline=BASE,changed=changed,
        invariant='Original 100ms freshness, MCU watchdog, range/tilt and v2 causal pairing remain mandatory')
    (destination/'serial-owner-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('destination',type=Path)
    a=p.parse_args();print(json.dumps(prepare(a.source,a.destination),indent=2))
