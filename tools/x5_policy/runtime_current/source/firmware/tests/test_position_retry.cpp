#define main existingFirmwareTests
#include "test_firmware.cpp"
#undef main

unsigned reads[35] = {}, silence[35] = {};
uint8_t partialId = 0, expireOnRetryId = 0;
void retryBus(const uint8_t* p, size_t n) {
  if (p[4] == 2 && p[5] == 56) {
    ++reads[p[2]];
    if (reads[p[2]] <= silence[p[2]]) return;
    if (p[2] == partialId) { Serial1.input.push_back(255); return; }
    if (p[2] == expireOnRetryId && reads[p[2]] == 2)
      mockMicros += motion::kWatchdogUs;
  }
  busWrite(p,n);
}
void retryFixture(bool armed = true) {
  mockMicros=1000000;
  lock.armed=armed;lock.cookie=armed?123:0;lock.configured=true;lock.lastError=motion::Ok;
  lock.lastCommandUs=mockMicros-10000;
  maintenanceMode=preparingArm=verifyingTorque=disablePending=torqueVerifyPending=false;
  feedback.imuOk=true;feedback.imuUs=feedback.sampleUs=mockMicros;
  presentMask=torqueKnown=feedback.onlineMask=motion::kAllMotors;
  feedback.torqueMask=armed?motion::kAllMotors:0;
  presentFaults=torqueFaults=feedback.faultMask=0;
  for(size_t i=0;i<15;++i)torqueReadUs[i]=mockMicros;
  servoSampleSequence=0;servoErrors=0;positionRetryDiagnostic=PositionRetryDiagnostic{};
  safetyDiagnostic=SafetyDiagnostic{};
  Serial.input.clear();Serial.output.clear();Serial1.input.clear();Serial1.output.clear();
  Serial1.scheduledInput.clear();Serial1.writeResultOverride=-1;Serial1.onWrite=retryBus;
  absentServo=corruptChecksumServo=wrongReplyIdServo=uartErrorServo=invalidPositionServo=0;
  absentPositionServo=outsideRangePositionServo=0;partialId=expireOnRetryId=0;
  errorOnlyReply=false;i2cReady=false;writes.clear();
  for(size_t i=0;i<35;++i){reads[i]=silence[i]=0;mockServoFault[i]=0;mockTorque[i]=armed;}
}
unsigned readTotal(){unsigned n=0;for(unsigned v:reads)n+=v;return n;}
void recoveredSilence(uint8_t id) {
  retryFixture();silence[id]=1;
  const uint32_t lease=lock.lastCommandUs;
  sampleServos();
  assert(lock.armed&&lock.cookie==123&&lock.lastCommandUs==lease);
  assert(readTotal()==16&&reads[id]==2&&writes.empty());
  assert(positionRetryDiagnostic.attempts==1&&positionRetryDiagnostic.recovered==1&&positionRetryDiagnostic.failed==0);
  assert(positionRetryDiagnostic.id==id&&positionRetryDiagnostic.firstOutcome==motion::NoValidReplyByDeadline);
  assert(positionRetryDiagnostic.retryOutcome==motion::ValidNormalReply);
  assert(feedback.onlineMask==motion::kAllMotors&&servoSampleSequence==1);
}
void noRetryAfterHardError(unsigned kind) {
  retryFixture();const uint8_t id=30;
  if(kind==0)mockServoFault[id]=1;
  if(kind==1)corruptChecksumServo=id;
  if(kind==2)wrongReplyIdServo=id;
  if(kind==3)uartErrorServo=id;
  if(kind==4)invalidPositionServo=id;
  if(kind==5)partialId=id;
  if(kind==6)Serial1.writeResultOverride=0;
  sampleServos();
  assert(positionRetryDiagnostic.attempts==0&&!lock.armed);
  assert(reads[id]==1);
}
void noRetryWithInsufficientBudget(unsigned kind) {
  retryFixture();silence[10]=1;
  // First deadline consumes 1.5 ms, leaving less than a complete second read.
  if(kind==0)feedback.sampleUs-=motion::kStateFreshUs-2900;
  if(kind==1)feedback.imuUs-=motion::kStateFreshUs-2900;
  if(kind==2)lock.lastCommandUs=mockMicros-(motion::kWatchdogUs-2900);
  sampleServos();assert(positionRetryDiagnostic.attempts==0&&!lock.armed&&reads[10]==1);
}
void diagnosticIsCached() {
  retryFixture();silence[30]=1;sampleServos();
  const size_t uart=Serial1.output.size();
  const uint32_t sample=feedback.sampleUs,imu=feedback.imuUs,lease=lock.lastCommandUs;
  motion::Frame q;q.type=motion::PositionRetryDetails;q.sequence=55;q.length=0;
  sequence.reset();assert(sequence.consume(20));
  command(q);assert(sequence.consume(21));auto r=lastResponse();assert(r.type==0x9a&&r.length==28&&r.sequence==55);
  assert(r.payload[0]==1&&r.payload[1]==30&&motion::u32(r.payload+4)==1&&motion::u32(r.payload+8)==1);
  assert(Serial1.output.size()==uart&&feedback.sampleUs==sample&&feedback.imuUs==imu&&lock.lastCommandUs==lease&&lock.armed);
  q.length=1;command(q);r=lastResponse();assert(r.payload[1]==motion::BadPacket);
  assert(lock.lastCommandUs==lease&&Serial1.output.size()==uart);
  q.type=motion::State;q.length=0;command(q);assert(lastResponse().length==106);
  q.type=motion::VoltageState;command(q);assert(lastResponse().length==186);
  q.length=1;q.payload[0]=1;command(q);assert(lastResponse().length==255);
}
int main() {
  for(uint8_t id:expectedIds)recoveredSilence(id);
  retryFixture();silence[30]=2;sampleServos();
  assert(!lock.armed&&reads[30]==2&&positionRetryDiagnostic.failed==1);
  assert(safetyDiagnostic.cause==OfflineStop&&safetyDiagnostic.readId==30);
  assert(writes.size()==1&&writes[0]=="torque0");
  retryFixture();silence[10]=silence[11]=1;sampleServos();
  assert(!lock.armed&&reads[10]==2&&reads[11]==1&&readTotal()==16);
  assert(positionRetryDiagnostic.attempts==1);
  retryFixture(false);silence[30]=1;sampleServos();assert(positionRetryDiagnostic.attempts==0);
  retryFixture();preparingArm=true;silence[30]=1;sampleServos();assert(positionRetryDiagnostic.attempts==0);
  retryFixture();verifyingTorque=true;silence[30]=1;sampleServos();assert(positionRetryDiagnostic.attempts==0);
  for(unsigned i=0;i<7;++i)noRetryAfterHardError(i);
  for(unsigned i=0;i<3;++i)noRetryWithInsufficientBudget(i);
  retryFixture();silence[10]=1;expireOnRetryId=10;sampleServos();
  assert(!lock.armed&&positionRetryDiagnostic.attempts==1&&positionRetryDiagnostic.failed==1);
  assert(writes.size()==1&&writes[0]=="torque0");
  retryFixture();silence[10]=1;
  const uint32_t offset=UINT32_MAX-mockMicros-2000;
  mockMicros+=offset;feedback.sampleUs+=offset;feedback.imuUs+=offset;lock.lastCommandUs+=offset;
  for(size_t i=0;i<15;++i)torqueReadUs[i]+=offset;
  sampleServos();assert(lock.armed&&positionRetryDiagnostic.recovered==1);
  retryFixture();silence[30]=1;positionRetryDiagnostic.attempts=UINT32_MAX;sampleServos();
  assert(positionRetryDiagnostic.attempts==UINT32_MAX);
  retryFixture();silence[30]=1;invalidPositionServo=30;sampleServos();
  assert(!lock.armed&&positionRetryDiagnostic.failed==1&&positionRetryDiagnostic.retryOutcome==motion::InvalidRegisterReply);
  retryFixture();silence[10]=1;
  uint8_t stop[32];const size_t stopBytes=motion::encode(motion::Stop,88,nullptr,0,stop);
  Serial.input.insert(Serial.input.end(),stop,stop+stopBytes);
  sampleServos();assert(!lock.armed&&lock.cookie==0&&reads[10]==2);
  for(const auto& w:writes)assert(w=="torque0");
  diagnosticIsCached();
  puts("PASS bounded position retry: 15 IDs, one retry per cycle, repeated loss, hard faults, short budgets, watchdog, wrap and cached diagnostics");
}
