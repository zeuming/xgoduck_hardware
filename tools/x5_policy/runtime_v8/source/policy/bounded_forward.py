"""One explicit forward pulse; pure state/math, with no device or clock I/O."""
import math

PEAK_M_S = .2
DURATION_S = 2.
RAMP_S = .5
WARMUP_S = 5.
RECOVERY_S = 3.


class ForwardTargetExpired(Exception):
    """A same-pulse inference crossed its normal deadline; never send it."""
    def __init__(self, generation, epoch):
        super().__init__('试步推理超过时限，过期目标未执行')
        self.generation = generation
        self.epoch = epoch


def finite_time(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('试步单调时钟无效')
    return float(value)


class ForwardPulse:
    def __init__(self):
        self.epoch = 0
        self.reset()

    def reset(self):
        self.epoch += 1
        self.started = self.generation = None
        self.phase = 'idle'
        self.used = False
        self.positive_targets = 0
        self.zero_targets_after_positive = 0
        self.peak_sent_command = 0.
        self.first_positive_at = self.last_positive_at = self.first_zero_after_positive_at = None

    def note_successful_target(self, generation, sample_at, ended_at, command):
        """Display evidence only, called after a successful TARGET reply.

        It cannot renew feedback, authorize motion or alter the pulse clock.
        A zero-command TARGET confirms dispatch, not physical standing.
        """
        if not self.used or generation != self.generation:
            return
        if command[0] > 0:
            self.positive_targets += 1
            self.peak_sent_command = max(self.peak_sent_command, float(command[0]))
            if self.first_positive_at is None:
                self.first_positive_at = sample_at
            self.last_positive_at = ended_at
        elif self.positive_targets:
            self.zero_targets_after_positive += 1
            if self.first_zero_after_positive_at is None:
                self.first_zero_after_positive_at = ended_at

    def begin(self, now, generation):
        now = finite_time(now)
        if type(generation) is not int or generation < 0:
            raise ValueError('试步状态编号无效')
        if self.used:
            raise ValueError('本轮试步已使用，不能重复触发')
        self.started, self.generation = now, generation
        self.used, self.phase = True, 'running'
        self.epoch += 1

    def command_at(self, now, generation):
        now = finite_time(now)
        if self.phase not in ('running', 'returned-to-zero'):
            return (0., 0., 0.)
        if generation != self.generation:
            raise ValueError('试步状态已变化')
        elapsed = now - self.started
        if elapsed < 0:
            raise ValueError('试步单调时钟倒退')
        if now >= self.started + DURATION_S:
            self.phase = 'returned-to-zero'
            return (0., 0., 0.)
        fraction = max(0., min(1., elapsed / RAMP_S, (DURATION_S - elapsed) / RAMP_S))
        return (PEAK_M_S * fraction, 0., 0.)

    def cancel(self):
        if self.used:
            self.phase = 'cancelled'
            self.epoch += 1

    def check_inflight(self, now, generation, epoch):
        now = finite_time(now)
        if (not self.used or epoch != self.epoch or generation != self.generation
                or self.phase not in ('running', 'returned-to-zero')):
            raise ValueError('试步推理已取消，过期目标未执行')
        if now < self.started:
            raise ValueError('试步单调时钟倒退')
        if now >= self.started + DURATION_S:
            raise ForwardTargetExpired(generation, epoch)
