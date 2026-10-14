
#include "sync_test_helpers.h"
static bool injectDrained=false, breakGoalReply=false;
static const uint8_t capturedBytes[]={0x20,0xff,0,0xf0,0x10,0xfe,0,0,0xff};
static void tracedBus(const uint8_t* p,size_t n) {
  if(p[4]==0x83){
    syncBus(p,n);
    if(injectDrained){Serial1.input.insert(Serial1.input.end(),std::begin(capturedBytes),std::end(capturedBytes));Serial1.readCostUs=20;}
    return;
  }
  int beforeCorrupt=corruptId;
  if(breakGoalReply&&syncCalls&&p[4]==2&&p[5]==42)corruptId=p[2];
  const auto queued=Serial1.scheduledInput.size();
  syncBus(p,n);corruptId=beforeCorrupt;
  if(delayedWriteAck)for(size_t i=queued;i<Serial1.scheduledInput.size();++i)
    Serial1.scheduledInput[i].first+=3300; // Declare replies at4ms, not measured physical delay.
}
static void scene(){Serial1.readCostUs=0;syncScene();Serial1.onWrite=tracedBus;injectDrained=breakGoalReply=false;}
int main(){
  // Old caller behavior is preserved; the new optional path does not alter it.
  scene();Serial1.readCostUs=20;
  Serial1.input.insert(Serial1.input.end(),std::begin(capturedBytes),std::end(capturedBytes));
  auto before=transactions.size();auto legacy=readServo(10,42,2);
  assert(legacy.outcome==motion::BusNotQuiet&&transactions.size()==before&&!lastServoReadTrace.requestWritten);

  // Exact prior captured pre-drain bytes plus CPU cost no longer block READ.
  scene();Serial1.readCostUs=20;
  Serial1.input.insert(Serial1.input.end(),std::begin(capturedBytes),std::end(capturedBytes));
  before=transactions.size();auto source=readServo(10,42,2,true);
  assert(source.found&&source.outcome==motion::ValidNormalReply&&transactions.size()==before+1);
  assert(lastServoReadTrace.drainedCount==9&&lastServoReadTrace.requestWritten&&lastServoReadTrace.transmittedUs);

  // Actual 15-current-position diagnostic; write once, all15goal readbacks, OFF.
  scene();injectDrained=true;goalAutoEnables=true;
  expectSyncAck(syncRequest(),motion::Ok);assert(syncCalls==1&&lastDiagReply.payload[7]==15);offAndNoOtherWrites();

  // A real mismatch after TX must fail and preserve goal plus OFF evidence.
  scene();injectDrained=true;ignoreSync=true;goalAutoEnables=true;
  expectSyncAck(syncRequest(),motion::CenterGoalRead);assert(syncCalls==1);
  assert(centerFailureTrace.reply.found&&centerFailureTrace.requestWritten&&lastDiagReply.payload[227]==1);
  assert(motion::u16(lastDiagReply.payload+8)==32767&&motion::u16(lastDiagReply.payload+10)==0);
  assert(motion::u32(lastDiagReply.payload+238)==centerFailureTrace.transmittedUs);offAndNoOtherWrites();

  // 4ms responses pass existing upstream6ms, not a new timeout constant.
  scene();delayedWriteAck=true;goalAutoEnables=true;
  expectSyncAck(syncRequest(),motion::Ok);assert(syncCalls==1&&lastDiagReply.payload[7]==15);offAndNoOtherWrites();

  // Invalid checksum after a write stays a failure; no blind retry or fallback.
  scene();breakGoalReply=true;goalAutoEnables=true;
  expectSyncAck(syncRequest(),motion::CenterGoalRead);
  assert(centerFailureTrace.reply.invalidReplySeen&&centerFailureTrace.requestWritten&&syncCalls==1);offAndNoOtherWrites();

  // Silence and endless junk both remain bounded with no additional writes.
  scene();Serial1.onWrite=[](const uint8_t*p,size_t n){transactions.emplace_back(p,p+n);};
  auto began=mockMicros;before=transactions.size();auto silent=readServo(10,42,2,true);
  assert(silent.outcome==motion::NoValidReplyByDeadline&&transactions.size()==before+1&&mockMicros-began<6100);
  for(unsigned n=0;n<1000;++n)Serial1.scheduledInput.emplace_back(mockMicros+n*10,0x20);
  began=mockMicros;before=transactions.size();auto noisy=readServo(10,42,2,true);
  assert(!noisy.found&&noisy.invalidReplySeen&&transactions.size()==before+1&&mockMicros-began<6200);
  puts("PASS legacy preserved, queued bytes, all15 single write and OFF, mismatch, 4ms reply, bad checksum, silence, continuous noise");
}
