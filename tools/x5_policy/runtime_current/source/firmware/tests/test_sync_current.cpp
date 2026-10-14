#define main legacy_maintenance_test_main
#include "test_maintenance.cpp"
#undef main
static unsigned syncCalls=0;
static bool ignoreSync=false, shortSync=false;
static int mismatchId=-1;
static std::vector<uint16_t> captured;
void syncBus(const uint8_t* p,size_t n) {
 if(p[4]!=0x83){Serial1.writeResultOverride=-1;maintenanceBus(p,n);return;}
 ++syncCalls; transactions.emplace_back(p,p+n);
 uint8_t sum=0;for(size_t j=2;j<n;++j)sum+=p[j];assert(sum==255);
 assert(p[2]==254&&p[5]==42&&p[6]==6&&(n==15||n==113));
 captured.clear();
 for(size_t j=7;j<n-1;j+=7){
  auto id=p[j];auto goal=motion::u16(p+j+1);captured.push_back(goal);
  assert(testServos[id].present&&goal==testServos[id].position);
  assert(!p[j+3]&&!p[j+4]&&!p[j+5]&&!p[j+6]);
  if(!ignoreSync&&id!=mismatchId)testServos[id].goal=goal;
  if(goalAutoEnables)testServos[id].torque=true;
 }
 if(shortSync)Serial1.writeResultOverride=int(n-1);
}
void syncScene(){resetScene();Serial1.onWrite=syncBus;syncCalls=0;ignoreSync=shortSync=false;mismatchId=-1;
 for(size_t i=0;i<15;++i){auto& s=testServos[motion::kMotorIds[i]];s.present=true;s.position=2000+i*10;s.goal=99;}
 enter();transactions.clear();
}
motion::Frame syncRequest(bool all=true){auto f=maintenanceRequest(motion::MaintenanceCenterCheck,all?255:13);f.length=66;f.payload[5]=all?2:1;
 for(size_t i=0;i<15;++i){motion::put16(f.payload+6+4*i,1900);motion::put16(f.payload+8+4*i,2300);}return f;}
void offAndNoOtherWrites(){for(auto id:motion::kMotorIds)assert(!testServos[id].torque);
 for(auto& p:transactions) if(p[4]==3)assert(p[5]==40&&p[6]==0);
 assert(!lock.armed);
}
static uint32_t lastDiagSequence=0,lastDiagCookie=0;
static motion::Frame lastDiagReply;
void expectSyncAck(const motion::Frame& f,motion::Error err){
 if(lastDiagCookie!=currentCookie){lastDiagSequence=0;lastDiagCookie=currentCookie;}
 const bool dispatch=f.type==motion::MaintenanceCenterCheck&&f.length==66 &&
   (f.payload[4]<=253 || (f.payload[4]==255&&f.payload[5]==2)) &&
   f.sequence>lastDiagSequence;
 if(!dispatch){expectAck(f,err);return;}
 lastDiagSequence=f.sequence;command(f);auto r=result();lastDiagReply=r;assert(r.type==128&&r.length==250&&r.payload[0]==16&&r.payload[1]==err&&r.payload[2]==3);
 if(err==motion::Ok)assert(r.payload[4]==0&&r.payload[5]==1&&r.payload[6]==1&&motion::u16(r.payload+8)==32767&&motion::u16(r.payload+10)==0);
}
int main(){
 syncScene();auto f=syncRequest();expectSyncAck(f,motion::Ok);assert(syncCalls==1&&captured.size()==15);offAndNoOtherWrites();
 expectSyncAck(f,motion::BadPacket);assert(syncCalls==1); // consumed sequence, no replay.
 syncScene();goalAutoEnables=true;expectSyncAck(syncRequest(),motion::Ok);assert(syncCalls==1);offAndNoOtherWrites();
 syncScene();goalAutoEnables=true;expectSyncAck(syncRequest(false),motion::Ok);assert(syncCalls==1&&captured.size()==1);offAndNoOtherWrites();
 syncScene();ignoreSync=true;expectSyncAck(syncRequest(),motion::CenterGoalRead);assert(syncCalls==1&&centerFailureTraceValid&&centerFailureTrace.id==10);offAndNoOtherWrites();
 syncScene();mismatchId=13;goalAutoEnables=true;expectSyncAck(syncRequest(),motion::CenterGoalRead);assert(centerFailureTrace.id==13);offAndNoOtherWrites();
 syncScene();shortSync=true;goalAutoEnables=true;expectSyncAck(syncRequest(),motion::CenterGoalWrite);assert(syncCalls==1);offAndNoOtherWrites();
 syncScene();testServos[24].torque=true;expectSyncAck(syncRequest(),motion::CenterSnapshot);assert(syncCalls==0); // refuses before any write.
 syncScene();f=syncRequest();motion::put16(f.payload+6,2100);expectSyncAck(f,motion::Range);assert(syncCalls==0);offAndNoOtherWrites();
 syncScene();noReadAfterRename=10;expectSyncAck(syncRequest(),motion::CenterSnapshot);assert(syncCalls==0);offAndNoOtherWrites();
 syncScene();f=syncRequest(false);f.payload[5]=7;expectSyncAck(f,motion::BadPacket);assert(syncCalls==0);offAndNoOtherWrites();
 syncScene();f=syncRequest();f.payload[4]=13;expectSyncAck(f,motion::BadPacket);assert(syncCalls==0);offAndNoOtherWrites();
 syncScene();goalAutoEnables=true;ignoreRelease=true;expectSyncAck(syncRequest(),motion::BadState);assert(syncCalls==1&&disablePending&&maintenance.state==3); // OFF failure cannot be success.
 syncScene();f=syncRequest();f.length=65;expectSyncAck(f,motion::BadPacket);assert(syncCalls==0);offAndNoOtherWrites();
 syncScene();maintenance.faultSeen=true;expectSyncAck(syncRequest(),motion::BadState);assert(syncCalls==0);offAndNoOtherWrites();
 syncScene();ignoreSync=true;goalAutoEnables=true;ignoreRelease=true;
 expectSyncAck(syncRequest(),motion::BadState);auto detail=lastDiagReply;
 assert(detail.payload[4]==motion::CenterGoalRead&&detail.payload[14]==1&&detail.payload[18]==10);
 assert(motion::u16(detail.payload+10)==32767&&detail.payload[7]==0); // preserve both failures.
 printf("DETAIL_FIXTURE=");for(size_t j=0;j<detail.length;++j)printf("%02x",detail.payload[j]);puts("");
 puts("PASS 15 diagnostic scenarios: all/single, auto-holding, ignored/partial/short writes, real OFF failures, bounds/state/format, replay; no new gains/enable/target commands");
}
