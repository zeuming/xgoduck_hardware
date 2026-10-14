"""Ideal plant test fixture: full v2 packets, no serial device or I/O."""
import struct
import motion
from causal_feedback import PAIRED_EXTENSION
from test_motion import FakeTransport


class PairedFakeTransport(FakeTransport):
    def __init__(self,port):
        super().__init__(port)
        self.paired_goals=list(self.positions)
        self.goal_sequence=0
        self.sample_sequence=0
        self.previous_completed_sample=None
        self.pair_valid=True
        self.goal_override=None
        self.protocol_version=2
        self.stop_count=0

    def negotiate_voltage(self):
        self.requests.append((motion.TELEMETRY_CAPS,b''))
        return True

    def negotiate_motion_evidence(self):
        self.requests.append((motion.MOTION_EVIDENCE_CAPS,b'\x02'))
        return self.protocol_version

    def request(self,kind,payload=b''):
        if kind == motion.VOLTAGE_STATE:
            if payload != b'\x02':
                raise AssertionError('Candidate must explicitly select v2')
            base=super().request(motion.STATE)
            self.requests[-1]=(kind,payload)
            if self.sample_us != self.previous_completed_sample:
                self.sample_sequence=(self.sample_sequence+1)&0xffffffff
                self.previous_completed_sample=self.sample_us
            voltage=motion.VOLTAGE_EXTENSION.pack(1,0x7fff,0x7fff,*([74]*15),*([20000]*15))
            start=(self.sample_us-1000)&0xffffffff
            evidence=PAIRED_EXTENSION.pack(2,int(self.pair_valid and self.armed and
                self.online==0x7fff and not self.fault),self.sample_sequence,start,
                self.cookie,(start-1000)&0xffffffff,self.goal_sequence,
                *(self.goal_override if self.goal_override is not None else self.paired_goals))
            return base+voltage+evidence
        answer=super().request(kind,payload)
        if kind in (motion.ARM,motion.TARGET):
            self.paired_goals=list(self.positions)
            self.goal_sequence=(self.goal_sequence+1)&0xffffffff
        if kind==motion.STOP:self.stop_count+=1
        return answer
