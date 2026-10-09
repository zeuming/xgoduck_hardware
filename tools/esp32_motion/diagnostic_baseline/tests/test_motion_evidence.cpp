#define main existingFirmwareTests
#include "test_firmware.cpp"
#undef main
#include <cstdlib>

size_t sampleReadTransactions = 0;
void loadBus(const uint8_t* p, size_t n) {
  busWrite(p,n);
  if (p[4] == 2 && p[5] == 56 && p[6] == 6) {
    ++sampleReadTransactions;
    if (p[2] == absentServo) return;
    const size_t end=Serial1.input.size();
    assert(end>=12);
    const uint8_t lo=p[2],hi=0x80;
    Serial1.input[end-3]=lo;Serial1.input[end-2]=hi;
    Serial1.input[end-1]=uint8_t(Serial1.input[end-1]-lo-hi);
  }
}
void fixture() {
  mockMicros=1000000; lock.armed=false;lock.cookie=0;lock.lastCommandUs=900000;
  lock.configured=true;lock.lastError=motion::Ok;
  maintenanceMode=preparingArm=verifyingTorque=disablePending=torqueVerifyPending=false;
  feedback.imuOk=true;feedback.imuUs=feedback.sampleUs=mockMicros;
  presentMask=torqueKnown=feedback.onlineMask=motion::kAllMotors;
  presentFaults=torqueFaults=feedback.faultMask=feedback.torqueMask=0;
  voltageSeen=voltageValid=0;
  for(size_t i=0;i<15;++i)torqueReadUs[i]=mockMicros;
  Serial.output.clear();Serial.input.clear();Serial1.input.clear();Serial1.scheduledInput.clear();
  Serial1.writeResultOverride=-1;Serial1.onWrite=loadBus;
  absentServo=0;sampleReadTransactions=0;
  i2cReady=false;
}
void issueEvidence(bool armed, bool emitFixture=false) {
  fixture();lock.armed=armed;lock.cookie=armed?123:0;
  feedback.torqueMask=armed?motion::kAllMotors:0;
  lock.lastCommandUs=mockMicros-10000;
  feedback.sampleUs=feedback.imuUs=mockMicros;feedback.imuOk=true;
  servoSampleSequence=17;lastWrittenGoalsUs=mockMicros-1000;
  for(size_t i=0;i<15;++i){lastWrittenGoals[i]=uint16_t(1900+i);presentLoadRaw[i]=uint16_t(0x8000+motion::kMotorIds[i]);}
  const size_t bus=Serial1.output.size(),imu=imuBusReads;
  const uint32_t lastCommand=lock.lastCommandUs,sample=feedback.sampleUs,imuUs=feedback.imuUs;
  motion::Frame q;q.type=motion::VoltageState;q.sequence=77;q.length=1;q.payload[0]=1;
  command(q);const auto response=lastResponse();
  assert(response.type==0x93&&response.length==255&&response.sequence==77);
  assert(response.payload[186]==1&&motion::u32(response.payload+187)==lastWrittenGoalsUs);
  assert(motion::u32(response.payload+191)==17);
  for(size_t i=0;i<15;++i){assert(motion::u16(response.payload+195+2*i)==1900+i);assert(motion::u16(response.payload+225+2*i)==0x8000+motion::kMotorIds[i]);}
  assert(lock.armed==armed&&lock.lastCommandUs==lastCommand);
  assert(feedback.sampleUs==sample&&feedback.imuUs==imuUs);
  assert(Serial1.output.size()==bus&&imuBusReads==imu&&sampleReadTransactions==0);
  if(emitFixture){const char* path=std::getenv("XGODUCK_MOTION_EVIDENCE_FIXTURE");assert(path);FILE* out=fopen(path,"wb");assert(out);assert(fwrite(response.payload,1,response.length,out)==response.length);assert(fclose(out)==0);}
}
int main(){
  fixture();sampleServos();assert(sampleReadTransactions==15);
  for(size_t i=0;i<15;++i)assert(presentLoadRaw[i]==uint16_t(0x8000+motion::kMotorIds[i]));
  absentServo=21;sampleReadTransactions=0;sampleServos();assert(sampleReadTransactions==15);
  assert(!(feedback.onlineMask&(1u<<6))&&presentLoadRaw[6]==0);absentServo=0;
  fixture();uint16_t goals[15];for(size_t i=0;i<15;++i)goals[i]=uint16_t(2000+i);
  assert(writeGoals(goals));assert(memcmp(goals,lastWrittenGoals,sizeof(goals))==0);
  const uint32_t written=lastWrittenGoalsUs;Serial1.writeResultOverride=0;goals[0]=2010;
  assert(!writeGoals(goals));assert(lastWrittenGoals[0]==2000&&lastWrittenGoalsUs==written);
  issueEvidence(false,true);issueEvidence(true);
  fixture();motion::Frame q;q.type=motion::State;q.sequence=90;command(q);assert(lastResponse().length==106);
  q.type=motion::VoltageState;command(q);assert(lastResponse().length==186);
  q.type=motion::MotionEvidenceCaps;command(q);auto cap=lastResponse();assert(cap.type==0x99&&std::string((char*)cap.payload,cap.length)=="XGODUCK_MOTION_EVIDENCE_1");
  q.type=motion::VoltageState;q.length=1;q.payload[0]=2;command(q);assert(lastResponse().payload[1]==motion::BadPacket);
  fixture();lock.armed=true;lock.cookie=123;feedback.torqueMask=motion::kAllMotors;lock.lastCommandUs=mockMicros-motion::kWatchdogUs-1;
  q.type=motion::VoltageState;q.length=1;q.payload[0]=1;command(q);auto stopped=lastResponse();assert(stopped.type==0x93&&stopped.payload[14]==0&&!lock.armed);
  puts("PASS optional live MCU cache telemetry, no added servo reads/writes, legacy compatibility and watchdog");
}
