"""Controller side of the optional single-owner UART pipeline.

Only the control thread commits acknowledged action recurrence. The UART worker
cannot infer a policy, renew a page lease, generate TARGETs, or own motion mode.
"""
import math
import threading
import time
from serial_owner import SerialOwner, OwnerError, TargetCancelled, SnapshotAdvanced


class SerialOwnerControllerMixin:
    def _io_initialize(self, enabled):
        if type(enabled) is not bool: raise ValueError('serial_owner must be boolean')
        self._serial_owner_enabled=enabled
        self._io_owner=None
        self._io_pending=None
        self._io_snapshot_serial=None
        self._io_wake=threading.Event()
        if enabled:
            original=self._factory
            def factory(port):
                owner=SerialOwner(original,port,clock=self._clock,wake=self._io_wake,
                                  timing_callback=self._record_timing)
                self._io_owner=owner
                return owner
            self._factory=factory

    def _io_cancel_targets_locked(self):
        if not getattr(self,'_serial_owner_enabled',False): return
        if self._io_owner is not None:self._io_owner.cancel_targets()
        pending=self._io_pending
        self._io_pending=None
        if pending is not None:
            self._append_trace(pending['trace']+(self._clock(),'TARGET invalidated; no history committed'))

    def _io_accept_cache_locked(self):
        from production_motion import decode_state, STATE_PACKET, VOLTAGE_EXTENSION, PAIRED_EXTENSION, MotionError
        owner=self._io_owner
        if owner is None:raise MotionError('UART owner is unavailable')
        try:snapshot=owner.snapshot()
        except OwnerError as error:raise MotionError(str(error)) from error
        if snapshot is None or snapshot.serial==self._io_snapshot_serial:return
        if len(snapshot.payload)!=STATE_PACKET.size+VOLTAGE_EXTENSION.size+PAIRED_EXTENSION.size:
            raise MotionError('协商后的实时状态长度不匹配')
        state=decode_state(snapshot.payload)
        state['_host_received_at']=snapshot.received_at
        self._state_request_sequence=snapshot.sequence
        self._io_snapshot_serial=snapshot.serial
        self._accept_state_locked(state)

    def _validate_async_target_locked(self,payload):
        """Explicit wrapper hook, including the current-pose axis restriction."""

    def _io_submit_target_locked(self, payload, *, context, target, execution_target,
                                 action, smooth, inference_ms, latest_frame,
                                 observation_received, retry_of=None):
        from production_motion import FRESH_SECONDS, MotionError, EPISODE_SECONDS
        if self._io_pending is not None:raise MotionError('TARGET acknowledgement is still pending')
        self._validate_async_target_locked(payload)
        now=self._clock()
        deadline=min(observation_received+FRESH_SECONDS,
                     now+FRESH_SECONDS-self._age_locked(),
                     now+FRESH_SECONDS-self._imu_age_locked())
        tracking=getattr(self,'_tracking',None)
        if self._mode!='hold' or tracking and tracking.get('phase')=='running':
            deadline=min(deadline,self._heartbeat+self._heartbeat_timeout_locked())
        if tracking and tracking.get('phase')=='running':
            from tracking_probe import DURATION_S as tracking_duration
            deadline=min(deadline,tracking['started']+tracking_duration)
        if self._mode=='walk' and self._trial_began is not None:
            from attended_trial_limits import ZERO_WINDOW_S
            deadline=min(deadline,self._trial_began+ZERO_WINDOW_S)
        if any(latest_frame.get('command',())) and self._forward.started is not None:
            from bounded_forward import DURATION_S
            deadline=min(deadline,self._forward.started+DURATION_S)
        if self._mode in EPISODE_SECONDS and self._episode_started is not None:
            deadline=min(deadline,self._episode_started+EPISODE_SECONDS[self._mode])
            if self._mode=='getup' and now-self._episode_started<1.:
                deadline=min(deadline,self._episode_started+1.)
        if not math.isfinite(deadline) or deadline<=now:
            raise MotionError('TARGET feedback expired before enqueue')
        import struct
        trace=self._trace_sample('target-attempt',tuple(struct.unpack('<I15H',payload)[1:]))
        try:ticket=self._io_owner.submit_target(payload,deadline,self._io_snapshot_serial,retry_of=retry_of)
        except OwnerError as error:raise MotionError(str(error)) from error
        self._io_pending=dict(ticket=ticket,context=context,trace=trace,
            target=list(target),execution_target=list(execution_target),
            action=None if action is None else action.copy(),smooth=smooth.copy(),
            inference_ms=inference_ms,latest_frame=latest_frame,
            observation_received=observation_received, payload=bytes(payload))

    def _io_drain_ack(self):
        from production_motion import MotionError, FRESH_SECONDS
        if not self._serial_owner_enabled:return
        with self._lock:
            pending=self._io_pending
            if pending is None:return
            ticket=self._io_owner.take_target(pending['ticket'])
            if ticket is None:return
            self._io_pending=None
            if pending['context']!=self._policy_context_locked():return
            self._append_trace(pending['trace']+(ticket.ended,
                str(ticket.error) if ticket.error is not None else None))
            if ticket.error is not None:
                if (isinstance(ticket.error,SnapshotAdvanced) and not ticket.transmitted
                        and self._clock()<ticket.deadline):
                    self._io_accept_cache_locked()
                    if (self._clock()-pending['observation_received'] < FRESH_SECONDS and
                            pending['context']==self._policy_context_locked()):
                        self._require_feedback_locked()
                        if self._heartbeat_expired_locked():return
                        self._motion_gate_locked()
                        # Same untransmitted candidate and original observation
                        # deadline, checked against the now-current feedback.
                        self._raw_targets_locked(pending['execution_target'])
                        self._io_submit_target_locked(pending['payload'],context=pending['context'],
                            target=pending['target'],execution_target=pending['execution_target'],
                            action=pending['action'],smooth=pending['smooth'],
                            inference_ms=pending['inference_ms'],latest_frame=pending['latest_frame'],
                            observation_received=pending['observation_received'],retry_of=ticket)
                    return
                if isinstance(ticket.error,TargetCancelled):return
                raise MotionError('TARGET acknowledgement failed: '+str(ticket.error)) from ticket.error
            # Actual ACK time, never enqueue time or delayed control consumption.
            self._last_executed_target=pending['execution_target']
            self._target_times.append(ticket.ended)
            self._forward.note_successful_target(pending['context'][0],ticket.began,
                                                 ticket.ended,pending['trace'][14])
            if (self._mode=='walk' and self._walk_warmup_generation==self._generation
                    and not any(pending['trace'][14]) and not any(self._command)):
                self._record_successful_zero_target_locked(ticket.ended)
            action=pending['action']
            if action is not None:
                self._previous_action[:]=action;self._smooth_action[:]=pending['smooth']
            self._last_action=pending['target'];self._last_inference_ms=pending['inference_ms']
            self._error=None
            # The inference input is the newest genuinely received state. This
            # event work can run between 20ms control ticks, after ACK only.
            self._io_accept_cache_locked()
            if (self._policy_worker is not None and action is not None and
                    pending['context']==self._policy_context_locked() and
                    self._age_locked()<FRESH_SECONDS and self._imu_age_locked()<FRESH_SECONDS):
                from production_motion import _copy_policy_state
                frame=dict(pending['latest_frame'])
                frame.update(st=_copy_policy_state(self._filtered),
                    previous=action.copy(),smooth=pending['smooth'].copy(),
                    observation_received=min(self._sample_received,self._clock()-self._imu_age_locked()),
                    observation_sample=self._last_sample)
                self._submit_policy_frame(frame)

    def _io_wait_until(self,deadline):
        """ACK wakes recur/inference processing; no extra control TARGET tick."""
        if not self._serial_owner_enabled:
            self._closed.wait(max(.001,deadline-self._clock()));return
        while not self._closed.is_set():
            self._io_wake.clear()
            try:self._io_drain_ack()
            except Exception as error:
                self._io_event_error=error
                return
            remaining=deadline-self._clock()
            if remaining<=0:return
            self._io_wake.wait(remaining)

    def _io_cycle_state_locked(self):
        if self._serial_owner_enabled:
            self._io_accept_cache_locked()
            return self._state
        state=self._read_state();self._accept_state_locked(state);return state
