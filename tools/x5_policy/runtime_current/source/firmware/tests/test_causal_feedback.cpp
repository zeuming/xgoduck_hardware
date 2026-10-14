#define main existingFirmwareTests
#include "test_firmware.cpp"
#undef main
#include <cstdlib>

size_t causalPositionReads = 0;
enum Injection { None, GoalDuringScan, MaintenanceDuringScan, NewerScanDuringScan };
Injection injection = None;
void causalBus(const uint8_t* p, size_t n) {
  busWrite(p,n);
  if (p[4] != 2 || p[5] != 56) return;
  ++causalPositionReads;
  if (p[2] != 12 || injection == None) return;
  const Injection selected = injection; injection = None;
  if (selected == GoalDuringScan) {
    uint16_t goals[15]; for (size_t i=0;i<15;++i) goals[i]=2111;
    assert(writeGoals(goals));
  } else if (selected == MaintenanceDuringScan) {
    maintenanceMode = true; invalidateMotionFeedback();
  } else {
    // Exercise the outer scan's existing generation guard independently of
    // ARM: a newer complete snapshot must not be overwritten on unwind.
    ++servoSampleSequence;
    for(size_t i=0;i<15;++i)feedback.positions[i]=2222;
    feedback.sampleUs=mockMicros;
    sampleBoundGoals.sampleSequence=servoSampleSequence;
    sampleBoundGoals.sampleStartUs=mockMicros-100;
    sampleBoundGoals.goals[0]=2222;
    sampleBoundGoals.valid=true;
  }
}

void causalFixture(bool armed=true, uint32_t start=1000000) {
  mockMicros=start;sequence.reset();parser.reset();
  lock=motion::Interlock{};lock.configured=true;lock.armed=armed;
  lock.cookie=armed?123:0;lock.lastCommandUs=start-10000;
  const auto config=configuration(1);assert(lock.config.decode(config.payload,config.length));
  maintenanceMode=preparingArm=verifyingTorque=disablePending=torqueVerifyPending=false;
  feedback=motion::Feedback{};feedback.imuOk=true;feedback.sampleUs=feedback.imuUs=start;
  presentMask=torqueKnown=feedback.onlineMask=motion::kAllMotors;
  feedback.torqueMask=armed?motion::kAllMotors:0;
  presentFaults=torqueFaults=0;servoSampleSequence=0;servoErrors=0;
  for(size_t i=0;i<15;++i){torqueReadUs[i]=start;feedback.positions[i]=2047;}
  positionRetryDiagnostic=PositionRetryDiagnostic{};safetyDiagnostic=SafetyDiagnostic{};
  Serial.input.clear();Serial.output.clear();Serial1.input.clear();Serial1.output.clear();
  Serial1.scheduledInput.clear();Serial1.writeResultOverride=-1;Serial1.onWrite=causalBus;
  absentServo=corruptChecksumServo=wrongReplyIdServo=uartErrorServo=invalidPositionServo=0;
  absentPositionServo=outsideRangePositionServo=ignoresTorqueServo=ignoresGainsServo=absentGainsServo=0;
  invalidTorqueValueServo=absentEnabledTorqueServo=invalidEnabledTorqueServo=0;
  errorOnlyReply=delayGainsUntilStale=delayedGainWriteAck=corruptGainWriteAck=false;
  delayedTorqueWriteEcho=corruptTorqueWriteEcho=receiveErrorOnTorqueOff=queueArmInPositionCycle=false;
  i2cReady=false;writes.clear();gainReads.clear();normalPosition=2047;
  writtenGoalsSequence=0;writtenGoalsSeen=false;lastWrittenGoalsUs=0;causalArmEpoch=armed?1:0;
  sampleBoundGoals=SampleBoundGoals{};memset(lastWrittenGoals,0,sizeof(lastWrittenGoals));
  for(size_t i=0;i<35;++i){mockServoFault[i]=0;mockTorque[i]=armed;mockKp[i]=mockKd[i]=0;}
  causalPositionReads=0;injection=None;voltageSeen=voltageValid=0;
}

void transmit(uint16_t value=2047) {
  uint16_t goals[15];for(size_t i=0;i<15;++i)goals[i]=value;
  assert(writeGoals(goals));
}

motion::Frame query(uint8_t version=2) {
  motion::Frame q;q.type=motion::VoltageState;q.sequence=77;
  q.length=version?1:0;q.payload[0]=version;
  command(q);return lastResponse();
}

void testPairingAndLegacy() {
  causalFixture();transmit();
  const uint32_t written=lastWrittenGoalsUs,txSequence=writtenGoalsSequence,lease=lock.lastCommandUs;
  const size_t writesBefore=writes.size();sampleServos();
  assert(causalPositionReads==15&&writes.size()==writesBefore&&lock.lastCommandUs==lease);
  assert(sampleBoundGoals.valid&&sampleBoundGoals.sampleSequence==1);
  const uint32_t sample=feedback.sampleUs,start=sampleBoundGoals.sampleStartUs;
  transmit(2111); // New cache is AFTER the scan. The sample-bound cache stays old.
  const size_t busBytes=Serial1.output.size(),reads=causalPositionReads,imuReads=imuBusReads;
  const auto v2=query();
  assert(v2.type==0x93&&v2.length==238&&v2.payload[186]==2&&v2.payload[187]==1);
  assert(motion::u32(v2.payload)==sample&&motion::u32(v2.payload+188)==1);
  assert(motion::u32(v2.payload+192)==start&&motion::u32(v2.payload+196)==123);
  assert(motion::u32(v2.payload+200)==written&&motion::u32(v2.payload+204)==txSequence);
  for(size_t i=0;i<15;++i)assert(motion::u16(v2.payload+208+2*i)==2047);
  const auto v1=query(1);
  assert(v1.length==255&&v1.payload[186]==1);
  assert(motion::u32(v1.payload+187)==lastWrittenGoalsUs);
  for(size_t i=0;i<15;++i)assert(motion::u16(v1.payload+195+2*i)==2111);
  assert(query(0).length==186);
  motion::Frame q;q.type=motion::State;q.sequence=90;command(q);assert(lastResponse().length==106);
  q.type=motion::MotionEvidenceCaps;command(q);auto caps=lastResponse();
  assert(std::string((char*)caps.payload,caps.length)=="XGODUCK_MOTION_EVIDENCE_1");
  q.length=1;q.payload[0]=2;command(q);caps=lastResponse();
  assert(std::string((char*)caps.payload,caps.length)=="XGODUCK_MOTION_EVIDENCE_2");
  assert(Serial1.output.size()==busBytes&&causalPositionReads==reads&&imuBusReads==imuReads);
  assert(lock.lastCommandUs==lease&&feedback.sampleUs==sample&&lock.armed);
  // Read-only negotiation does not consume the state-changing sequence guard.
  assert(sequence.consume(1));
  const char* path=std::getenv("XGODUCK_CAUSAL_FIXTURE");
  if(path){FILE* f=fopen(path,"wb");assert(f);assert(fwrite(v2.payload,1,v2.length,f)==v2.length);assert(fclose(f)==0);}
}

void testMissingFailedAndFutureGoal() {
  causalFixture();sampleServos();assert(!sampleBoundGoals.valid&&query().payload[187]==0);
  causalFixture();Serial1.writeResultOverride=0;uint16_t goals[15]={};
  assert(!writeGoals(goals));Serial1.writeResultOverride=-1;
  assert(!writtenGoalsSeen&&writtenGoalsSequence==0);
  sampleServos();assert(query().payload[187]==0);
  causalFixture();transmit();lastWrittenGoalsUs=mockMicros+1000;
  sampleServos();assert(!sampleBoundGoals.valid);
}

void testGoalChangesAndScanFailures() {
  causalFixture();transmit();injection=GoalDuringScan;sampleServos();
  assert(lock.armed&&causalPositionReads==15&&!sampleBoundGoals.valid&&query().payload[187]==0);
  causalFixture();transmit();absentPositionServo=12;sampleServos();
  assert(!sampleBoundGoals.valid&&!lock.armed&&query().payload[187]==0);
  causalFixture();transmit();sampleServos();const uint32_t before=servoSampleSequence;
  injection=MaintenanceDuringScan;sampleServos();
  assert(servoSampleSequence==before&&sampleBoundGoals.sampleSequence==before);
  assert(query().payload[187]==0);
  causalFixture();transmit();sampleServos();injection=NewerScanDuringScan;sampleServos();
  assert(servoSampleSequence==2&&sampleBoundGoals.sampleSequence==2&&sampleBoundGoals.valid);
  assert(sampleBoundGoals.goals[0]==2222);
  for(size_t i=0;i<15;++i)assert(feedback.positions[i]==2222);
}

void testStopAndNestedRearm() {
  causalFixture();transmit();sampleServos();assert(query().payload[187]==1);
  const uint32_t epoch=causalArmEpoch;
  const SampleBoundGoals oldPair=sampleBoundGoals;
  motion::Frame stop;stop.type=motion::Stop;command(stop);lastResponse();
  assert(!lock.armed&&query().payload[187]==0);
  // Reuse the same cookie deliberately: the internal ARM epoch must differ.
  sampleServos();command(armCommand(2));auto response=lastResponse();
  assert(response.payload[1]==motion::Ok&&lock.armed&&lock.cookie==123&&causalArmEpoch==epoch+1);
  // Existing refreshTorque() takes a fresh armed scan after torque readback.
  assert(query().payload[187]==1&&sampleBoundGoals.armEpoch==epoch+1);
  const SampleBoundGoals freshPair=sampleBoundGoals;
  sampleBoundGoals=oldPair;sampleBoundGoals.sampleSequence=servoSampleSequence;
  assert(query().payload[187]==0); // Same cookie/sequence cannot hide old ARM epoch.
  sampleBoundGoals=freshPair;assert(query().payload[187]==1);
  causalFixture(false);queueArmInPositionCycle=true;sampleServos();response=lastResponse();
  assert(response.payload[1]==motion::Ok&&lock.armed&&servoSampleSequence==2);
  assert(sampleBoundGoals.sampleSequence==2&&sampleBoundGoals.valid);
  for(size_t i=0;i<15;++i)assert(feedback.positions[i]==2200);
  sampleServos();assert(sampleBoundGoals.sampleSequence==3&&query().payload[187]==1);
}

void testWrapAndWatchdog() {
  causalFixture(true,UINT32_MAX-2000);transmit();const uint32_t goal=lastWrittenGoalsUs;
  mockMicros+=3000;sampleServos();
  assert(goal>0xffff0000u&&sampleBoundGoals.sampleStartUs<10000&&query().payload[187]==1);
  causalFixture();writtenGoalsSequence=UINT32_MAX;transmit();sampleServos();
  assert(writtenGoalsSequence==0&&writtenGoalsSeen&&query().payload[187]==1);
  causalFixture();transmit();lastWrittenGoalsUs=0;sampleServos();
  assert(writtenGoalsSeen&&sampleBoundGoals.valid); // A real TX may land at micros()==0.
  const uint32_t lease=lock.lastCommandUs;mockMicros=lease+motion::kWatchdogUs+1;
  query();assert(!lock.armed&&lock.lastCommandUs==lease&&writes.back()=="torque0");
}

int main() {
  testPairingAndLegacy();testMissingFailedAndFutureGoal();testGoalChangesAndScanFailures();
  testStopAndNestedRearm();testWrapAndWatchdog();
  puts("PASS causal v2 snapshot: 238-byte contract, legacy v1, no extra I/O/lease, nested ARM, partial scan, stop/rearm, maintenance, goal changes, failed TX, wrap, watchdog");
}
