#define main oldLegacyMain
#include "test_firmware.cpp"
#undef main
#include <algorithm>
static uint32_t replyDelay=0;
static size_t groupedRequests=0, singleRequests=0;
static uint16_t missing=0;
static bool corruptGroup=false, reverseGroup=false;
void sourceBus(const uint8_t* packet,size_t count) {
 if(packet[4]!=0x82){if(packet[4]==2)++singleRequests;busWrite(packet,count);return;}
 ++groupedRequests;
 uint8_t gold[23]={255,255,254,19,130,56,6,10,11,12,13,14,20,21,22,23,24,30,31,32,33,34,0};
 uint8_t sum=0;for(size_t i=2;i<22;++i)sum+=gold[i];gold[22]=uint8_t(~sum);
 assert(count==23&&!memcmp(gold,packet,count));
 std::vector<uint8_t> replies;
 for(size_t slot=0;slot<15;++slot){size_t i=reverseGroup?14-slot:slot;
   if(missing&(1u<<i))continue;
   const uint8_t id=expectedIds[i];
   std::vector<uint8_t> r={255,255,id,8,mockServoFault[id],uint8_t(normalPosition),uint8_t(normalPosition>>8),100,128,0,0};
   uint8_t check=0;for(size_t j=2;j<r.size();++j)check+=r[j];r.push_back(uint8_t(~check));
   if(corruptGroup && i==0)++r.back();
   replies.insert(replies.end(),r.begin(),r.end());
 }
 if(replyDelay){for(size_t i=0;i<replies.size();++i)Serial1.scheduledInput.emplace_back(mockMicros+replyDelay+uint32_t(i),replies[i]);}
 else Serial1.input.insert(Serial1.input.end(),replies.begin(),replies.end());
}
void negotiate(){motion::Frame f;f.type=motion::SourceRuntime;command(f);assert(lastResponse().payload[1]==0);}
void stopNow(){motion::Frame f;f.type=motion::Stop;command(f);assert(lastResponse().payload[1]==0);assert(!lock.armed&&!feedback.torqueMask&&torqueKnown==motion::kAllMotors);}
void armNow(uint32_t seq){sampleImu();sampleServos();command(armCommand(seq));assert(lastResponse().payload[1]==0);assert(lock.armed);}

#ifndef SOURCE_PARITY_NO_MAIN
int main(){
 Serial1.onWrite=sourceBus;Wire.registers[0x0f]=0x6b;Wire.registers[0x1e]=3;
 setup();mockMicros+=110000;readyFeedback();command(configuration(1));assert(lastResponse().payload[1]==0);
 size_t bytes=Serial1.output.size();negotiate();assert(lock.sourceRuntime&&bytes==Serial1.output.size());
 const size_t singles=singleRequests;sampleServos();assert(groupedRequests==1&&singleRequests==singles&&feedback.onlineMask==motion::kAllMotors);
 reverseGroup=true;sampleServos();assert(feedback.onlineMask==motion::kAllMotors);
 replyDelay=5000;sampleServos();assert(feedback.onlineMask==motion::kAllMotors);replyDelay=0;
 armNow(2);assert(torqueKnown==0&&feedback.torqueMask==0); // unknown, NOT fabricated ON.
 assert(singleRequests==singles&&gainReads.empty());
 uint32_t readBefore=positionReadUs[0];normalPosition=2100;missing=1;sampleServos();
 assert(lock.armed&&feedback.onlineMask==0x7ffe&&feedback.positions[0]==2047&&feedback.positions[1]==2100&&positionReadUs[0]==readBefore);
 missing=motion::kAllMotors;sampleServos();assert(lock.armed&&feedback.onlineMask==0&&feedback.positions[1]==2100);
 missing=0;corruptGroup=true;sampleServos();assert(lock.armed&&feedback.onlineMask==0x7ffe);corruptGroup=false;
 motion::Frame stateFrame;stateFrame.type=motion::SourceState;command(stateFrame);auto status=lastResponse();
 assert(status.type==0xa0&&status.length==170&&status.payload[108]==1&&motion::u16(status.payload+106)==0);
 motion::Frame target;target.type=motion::SourceTarget;target.sequence=3;target.length=35;motion::put32(target.payload,123);target.payload[34]=1;
 for(size_t i=0;i<15;++i)motion::put16(target.payload+4+2*i,2200);
 command(target);assert(Serial.output.empty()&&lock.armed&&lock.desired[0]==2200);
 lock.slew(micros());assert(lock.goals[0]==2200); // no 600-count slew.
 stopNow();const size_t afterStop=writes.size();command(target);assert(!lock.armed&&writes.size()>=afterStop);assert(Serial.output.empty());
 sampleImu();sampleServos();armNow(4);target.sequence=5;motion::put16(target.payload+4,999);command(target);assert(!lock.armed&&lock.lastError==motion::Range&&Serial.output.empty());stopNow();
 armNow(6);mockServoFault[11]=4;sampleServos();assert(lock.armed&&feedback.faultMask==2);mockServoFault[11]=0;stopNow();
 armNow(7);mockMicros+=250001;sampleImu();checkSafety();flushDisable();assert(!lock.armed);stopNow();
 armNow(8);feedback.imuUs=micros()-100001;checkSafety();flushDisable();assert(!lock.armed);stopNow();
 // OFF proof expiry schedules an actual background read on a requested OFF state.
 mockMicros+=500001;sampleImu();sampleServos();stateFrame.sequence=50;command(stateFrame);status=lastResponse();
 assert(motion::u16(status.payload+106)==0 && sourceOffVerifyPending);
 loop();stateFrame.sequence=51;command(stateFrame);status=lastResponse();
 assert(motion::u16(status.payload+106)==motion::kAllMotors);
 // A queued observational OFF scan must not leak into a later ARM.
 mockMicros+=500001;sampleImu();sampleServos();command(stateFrame);lastResponse();
 assert(sourceOffVerifyPending && !torqueVerifyPending);
 // Entire loop during valid source operation has no torque/voltage reads.
 armNow(9);size_t oldSingles=singleRequests;for(int i=0;i<10;++i)loop();assert(lock.armed&&singleRequests==oldSingles);stopNow();
 // HELLO can reset source negotiation only when OFF; never arms.
 motion::Frame hello;hello.type=motion::Hello;command(hello);lastResponse();assert(!lock.sourceRuntime&&!lock.armed);
 puts("PASS grouped packet/order/delay/partial/corrupt, source ARM no extra reads, asynchronous target, STOP wins, range/device/watchdog/IMU, OFF proof, zero extra running polls");
}

#endif
