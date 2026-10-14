#define SOURCE_PARITY_NO_MAIN
#include "test_source_firmware.cpp"
static uint8_t rawVoltage=74;
void voltageBus(const uint8_t* packet,size_t count) {
 if(packet[4]!=0x82 || packet[6]==6){sourceBus(packet,count);return;}
 assert(count==23 && packet[5]==56 && packet[6]==7);++groupedRequests;
 for(size_t i=0;i<15;++i){
  if(missing&(1u<<i))continue;
  std::vector<uint8_t> r={255,255,expectedIds[i],9,mockServoFault[expectedIds[i]],255,7,100,128,0,0,rawVoltage};
  uint8_t sum=0;for(size_t j=2;j<r.size();++j)sum+=r[j];r.push_back(uint8_t(~sum));
  if(corruptGroup&&i==0)++r.back();
  Serial1.input.insert(Serial1.input.end(),r.begin(),r.end());
 }
}
int main(){
 Serial1.onWrite=voltageBus;Wire.registers[0x0f]=0x6b;Wire.registers[0x1e]=3;
 setup();mockMicros+=110000;readyFeedback();command(configuration(1));lastResponse();negotiate();
 motion::Frame request;request.type=motion::SourceState;request.length=1;request.payload[0]=1;
 command(request);auto first=lastResponse();assert(first.length==205&&first.payload[109]==3);
 auto singles=singleRequests;sampleServos();assert(singleRequests==singles&&voltageValid==32767&&feedback.onlineMask==32767);
 command(request);auto s=lastResponse();assert(s.length==205&&s.payload[170]==1&&motion::u16(s.payload+171)==32767&&motion::u16(s.payload+173)==32767);
 for(int i=0;i<15;++i){assert(s.payload[175+i]==74);assert(s.payload[190+i]<=1);}
 const auto lastRead=voltageReadUs[0];missing=1;sampleServos();assert(voltageReadUs[0]==lastRead&&!(voltageValid&1)&&voltageSeen==32767);missing=0;
 corruptGroup=true;sampleServos();assert(!(voltageValid&1));corruptGroup=false;
 rawVoltage=0;sampleServos();assert(voltageValid==0&&voltageRaw[0]==74);rawVoltage=39;sampleServos();assert(voltageRaw[0]==39);
 mockMicros+=1000001;command(request);s=lastResponse();assert(motion::u16(s.payload+173)==0);
 rawVoltage=74;sampleServos();armNow(2);sampleServos();auto oldSingles=singleRequests;
 command(request);s=lastResponse();assert(s.length==205&&lock.armed&&singleRequests==oldSingles);
 for(int i=0;i<10;++i)loop();assert(lock.armed&&singleRequests==oldSingles);
 lock.lastCommandUs=micros()-250001;sampleImu();checkSafety();flushDisable();command(request);s=lastResponse();
 assert(s.length==249&&s.payload[109]==4&&s.payload[214]==1&&s.payload[173]==TargetWatchdogStop);
 request.length=0;command(request);s=lastResponse();assert(s.length==214&&s.payload[109]==2);
 puts("PASS live voltage grouped parser, missing/corrupt/zero/low/stale, running zero extra reads, retained stop and legacy packets");
}
