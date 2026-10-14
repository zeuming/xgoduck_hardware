"""Bounded STOP confirmation. Never enables torque or writes position targets."""
import time


class ReleaseUnconfirmed(RuntimeError):
    def __init__(self, report):
        self.report = report
        state = report.get('last_state') or {}
        missing = state.get('torque_unknown_ids', [])
        detail = ('；尚未获得扭矩反馈：ID ' + ', '.join(map(str, missing))) if missing else ''
        if report.get('last_error'):
            detail += '；' + report['last_error']
        super().__init__('停止已发送，释放扭矩仍未确认' + detail + '；请保持支撑，必要时断开舵机电池')


def confirm_release(send_stop, read_state, motor_ids, *, accept_state=None,
                    clock=time.monotonic, sleep=time.sleep, budget=1.0, fresh_seconds=.15):
    """Send STOP once; accept two progressing OFF samples, never an ACK alone.

    The 1 s confirmation budget covers two 250 ms torque refresh periods plus
    their interleaved read batches. It is NOT a motion freshness extension:
    every accepted response must arrive inside fresh_seconds. No ARM, TARGET,
    CONFIG, automatic STOP retry, or sensor timestamp mutation is performed.
    """
    report = dict(stop_sent=False, stop_acknowledged=False, release_confirmed=False,
                  samples=0, distinct_off_samples=0, last_state=None, last_error=None)
    start = clock()
    try:
        report['stop_sent'] = True
        send_stop()
        report['stop_acknowledged'] = True
    except Exception as error:
        # A lost ACK is not proof that STOP was not delivered. Only fresh
        # subsequent measured OFF state may confirm release; do not resend.
        report['stop_error'] = str(error)
    deadline = clock() + budget
    previous = None
    all_mask = (1 << len(motor_ids)) - 1
    while clock() < deadline:
        began = clock()
        try:
            state = read_state()
            elapsed = clock() - began
            if accept_state is not None:
                accept_state(state)
            # SOURCE_STATE reports torque readback independently of position
            # validity. Legacy STATE has no separate mask: its online bit is
            # still required as proof that that servo was actually read.
            known = state.get('torque_known_mask', state['online_mask'])
            report['samples'] += 1
            report['last_state'] = dict(sample_us=state['sample_us'],
                missing_ids=[sid for i, sid in enumerate(motor_ids) if not state['online_mask'] & (1 << i)],
                torque_unknown_ids=[sid for i, sid in enumerate(motor_ids) if not known & (1 << i)],
                torque_known_mask=known,
                fault_mask=state['fault_mask'], torque_mask=state['torque_mask'],
                armed=state['armed'], cookie=state['cookie'])
            healthy = (elapsed < fresh_seconds and clock() <= deadline and
                known == all_mask and not state['fault_mask'] and
                not state['torque_mask'] and not state['armed'] and not state['cookie'])
            sample = state['sample_us']
            delta = ((sample - previous) & 0xffffffff) if previous is not None else None
            if not healthy or delta is not None and delta >= 0x80000000:
                previous = None
                report['distinct_off_samples'] = 0
            elif previous is None:
                previous = sample
                report['distinct_off_samples'] = 1
            elif delta:
                report['distinct_off_samples'] += 1
                previous = sample
            if report['distinct_off_samples'] >= 2:
                report.update(release_confirmed=True, elapsed_ms=round((clock()-start)*1000, 3))
                return report
        except Exception as error:
            report['last_error'] = str(error)
            previous = None
            report['distinct_off_samples'] = 0
        sleep(min(.02, max(0., deadline - clock())))
    report['elapsed_ms'] = round((clock()-start)*1000, 3)
    raise ReleaseUnconfirmed(report)
