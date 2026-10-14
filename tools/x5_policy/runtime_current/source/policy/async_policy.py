"""Control-loop side of independent inference; no motor authority or blocking wait."""
import numpy as np
from policy_worker import PolicyWorkerError


class AsyncPolicyMixin:
    def attach_policy_worker(self, worker):
        # Only startup/offline construction may change execution architecture.
        with self._lock:
            if self._connected or self._mode != 'off':
                raise ValueError('Attach inference worker before opening hardware')
            self._policy_worker = worker
            self._policy_job = None

    def _policy_context_locked(self):
        pulse = getattr(self, '_forward', None)
        return (self._generation, self._mode, self._command_revision,
                None if pulse is None else pulse.epoch)

    def _cancel_policy_job(self):
        job, self._policy_job = self._policy_job, None
        if job is not None:
            self._policy_worker.discard(job['request_id'])

    def _submit_policy_frame(self, frame):
        obs = self._prepare_policy_observation(frame['selected'], frame['st'],
            frame['previous'], frame['command'], frame['elapsed'], head_command=frame['head_command'])
        request_id = self._policy_worker.submit(frame['selected'], obs)
        if request_id is not None:
            # Detached evidence of the tensor actually submitted, not a later
            # reconstruction from display state. This does not feed control.
            self._policy_job = dict(frame, request_id=request_id,
                submitted_observation=obs.copy(), submission_monotonic=self._clock())

    def _async_policy_step(self, frame, limit):
        job = self._policy_job
        if job is not None:
            obsolete = job['context'] != frame['context']
            expired = self._clock() - job['observation_received'] >= limit
            if obsolete or expired:
                if expired:
                    self._late_policy_frames += 1
                    self._last_late_policy_frame = dict(generation=job['generation'],
                        sample_us=job['observation_sample'],
                        observation_age_ms=(self._clock()-job['observation_received'])*1000,
                        recorded_monotonic=self._clock(), target_sent=False)
                self._cancel_policy_job()
                job = None
        if job is None:
            self._submit_policy_frame(frame)
            return None
        ready = self._policy_worker.take(job['request_id'])
        if ready is None:
            return None
        self._policy_job = None
        raw_action, inference_ms = ready
        target, action, smooth, _ = self._decode_policy_action(job['selected'],
            raw_action, job['previous'], job['smooth'], job['elapsed'], inference_ms)
        # Native CandidateTarget carries the pulse identity through the existing
        # execution-time check. Never stamp an old action with a new pulse epoch.
        if np.any(job['command']) and hasattr(target, 'diagnostics'):
            target.forward_pulse_epoch = job['context'][3]
        return job, target, action, smooth, inference_ms

    def _run_policy_inference(self, selected, observation):
        if self._policy_worker is not None:
            return self._policy_worker.run(selected, observation)
        import time
        began = time.perf_counter()
        session = self.sessions[selected]
        action = np.asarray(session.run(None, {session.get_inputs()[0].name: observation})[0])
        return action, (time.perf_counter()-began)*1000
