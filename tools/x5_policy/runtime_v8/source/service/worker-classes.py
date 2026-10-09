class ReviewedTransport(motion.SerialTransport):

    def open(self):
        global serial_open_attempts
        ram_health()
        (node, _) = serial_proof(EXPECTED)
        if _holders(node.st_rdev, time.monotonic() + 0.2):
            raise ProbeError('UART already has an owner')
        serial_open_attempts += 1
        opened_transports.append(self)
        super().open()
        try:
            serial_proof(EXPECTED, self.serial.fileno())
            fcntl.flock(self.serial.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(self.serial.fileno(), termios.TIOCEXCL)
            owners = [x for x in _holders(node.st_rdev, time.monotonic() + 0.2) if x['pid'] != os.getpid()]
            if owners:
                raise ProbeError('Another UART owner appeared')
            time.sleep(0.75)
            ram_health()
            if EXPECTED.get('discover_serial') is True:
                request_counts[str(motion.HELLO)] = request_counts.get(str(motion.HELLO), 0) + 1
                if super().request(motion.HELLO) != b'XGODUCK_MOTION_1':
                    raise ProbeError('Selected USB serial device does not run the ESP32 motion protocol')
        except BaseException:
            self.close()
            raise

    def request(self, kind, payload=b''):
        if kind not in (motion.HELLO, motion.STATE, motion.CONFIG, motion.ARM, motion.TARGET, motion.STOP, motion.VOLTAGE_STATE, motion.TELEMETRY_CAPS, motion.MOTION_EVIDENCE_CAPS):
            raise ProbeError('Unapproved motion request')
        if health_error and kind not in (motion.STOP, motion.STATE, motion.VOLTAGE_STATE):
            raise ProbeError(health_error)
        proof_began = time.monotonic()
        hot = kind in (motion.STATE, motion.VOLTAGE_STATE, motion.TARGET, motion.STOP)
        check = globals().get('serial_request_proof', serial_proof) if hot else serial_proof
        check(EXPECTED, self.serial.fileno())
        proof_ms = (time.monotonic() - proof_began) * 1000
        request_counts[str(kind)] = request_counts.get(str(kind), 0) + 1
        try:
            return super().request(kind, payload)
        finally:
            timings = getattr(self, 'request_timings', None)
            if timings and timings[-1]['kind'] == kind:
                timings[-1]['identity_proof_ms'] = round(proof_ms, 3)
                timings[-1]['identity_proof_scope'] = 'pinned-open-uart' if check is not serial_proof else 'full-usb-identity'

class OneShotController(motion.MotionController):

    def stop(self, source=None):
        with self._lock:
            self._requested_stop_source = source
            try:
                return super().stop()
            finally:
                self._requested_stop_source = None

    def _stop_locked(self, reason=None):
        before = super().status() if self._mode != 'off' else None
        timing = getattr(self._transport, 'timing_diagnostics', None)
        transport_diagnostic = timing() if before is not None and callable(timing) else None
        super()._stop_locked(reason)
        if before is not None:
            previous = getattr(self, '_last_stop_diagnostic', None)
            self._last_stop_diagnostic = dict(sequence=(previous or {}).get('sequence', 0) + 1, reason=reason, source=getattr(self, '_requested_stop_source', None) if reason is None else None, recorded_monotonic=self._clock(), before=before, transport=transport_diagnostic)

    def _service_stop_locked(self):
        if self._pending_stop and self._connected:
            self._request(motion.STOP)
            state = self._read_state()
            self._accept_state_locked(state)
            if state['online_mask'] != motion.ALL_MASK or state['fault_mask'] or state['armed'] or state['cookie'] or state['torque_mask']:
                message = '停止已发送，但全部舵机扭矩关闭尚未确认'
                if self._error:
                    message = self._error if message in self._error else self._error + '；' + message
                diagnostic = getattr(self, '_last_stop_diagnostic', None)
                if diagnostic is not None and 'verification_failure' not in diagnostic:
                    self._last_stop_diagnostic = dict(diagnostic, verification_failure=dict(reason=message, recorded_monotonic=self._clock(), after=super().status()))
                raise motion.MotionError(message)
            self._pending_stop = False

    def _run(self):
        failure = None
        try:
            self._connect_once()
            while not self._closed.is_set() and (not exit_event.is_set()):
                began = self._clock()
                if health_error:
                    raise motion.MotionError(health_error)
                self._cycle()
                self._closed.wait(max(0.001, motion.PERIOD - (self._clock() - began)))
        except BaseException as error:
            failure = str(error) or type(error).__name__
            self._terminal_error = failure
            with self._lock:
                self._stop_locked(failure)
                try:
                    self._service_stop_locked()
                except BaseException:
                    pass
            self._disconnect(error)
            exit_event.set()
        finally:
            if self._closed.is_set() or exit_event.is_set():
                with self._lock:
                    self._stop_locked(failure)
                    try:
                        self._service_stop_locked()
                    except BaseException:
                        pass
                self._disconnect(failure or '运控连接已结束')

    def status(self):
        from status_snapshot import capture_display_view
        waiting = time.perf_counter()
        with self._lock:
            acquired, cpu = time.perf_counter(), time.thread_time()
            view = capture_display_view(self)
            view._display_request_counts = dict(request_counts)
            released, cpu_end = time.perf_counter(), time.thread_time()
        # All NumPy, gate explanations, metadata and diagnostic formatting use
        # the detached view. START/TARGET continue to check the live controller.
        result = view._format_display_status()
        formatted = time.perf_counter()
        row = (waiting, acquired, released, cpu_end-cpu,
               motion.threading.get_ident(), view._generation, view._mode)
        with self._lock:
            history = (getattr(self, '_status_lock_timings', ()) + (row,))[-8:]
            self._status_lock_timings = history
        result['status_lock_diagnostics'] = dict(protocol=1,
            columns=('wait_began_monotonic','acquired_monotonic','ended_monotonic',
                     'held_cpu_seconds','thread_id','generation','mode'),
            recent=history, formatting_outside_control_lock=True,
            last_format_wall_ms=(formatted-released)*1000)
        return result

    def _format_display_status(self):
        with self._lock:
            result = super().status()
            try:
                self._motion_gate_locked('hold' if self._mode == 'off' else self._mode)
                reason = None
            except Exception as error:
                reason = str(error)
            result['start_blocked_reason'] = reason
            result['model_start_blocked_reasons'] = {}
            for name in motion.MODEL_MODES:
                try:
                    self._model_gate_locked(name)
                    reason = None
                except Exception as error:
                    reason = str(error)
                result['model_start_blocked_reasons'][name] = reason
            result['release_confirmed'] = bool(self._connected and self._state and (self._state['online_mask'] == motion.ALL_MASK) and (not self._state['fault_mask']) and (not self._state['armed']) and (not self._state['torque_mask']) and (not self._state['cookie']) and (not self._pending_stop))
            result['imu_ok'] = bool(self._state and self._state['imu_ok'])
            result['request_counts'] = dict(self._display_request_counts)
            diagnostic = getattr(self, '_last_stop_diagnostic', None)
        compact = globals().get('native_compact_stop_diagnostic')
        if callable(compact):
            result['last_stop_diagnostic'] = compact(diagnostic)
            if result['last_stop_diagnostic'] is not None:
                result['last_stop_diagnostic']['details_deferred_until_UART_closed'] = True
        else:
            result['last_stop_diagnostic'] = motion.copy.deepcopy(diagnostic)
        return result
