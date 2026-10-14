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
