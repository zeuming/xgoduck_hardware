// Include the actual firmware and memory-only serial fixture.
#define main originalFirmwareSuite
#include "test_firmware.cpp"
#undef main
int main() {
 Serial1.onWrite=busWrite;Wire.registers[0x0f]=0x6b;Wire.registers[0x1e]=3;
 setup();mockMicros+=110000;readyFeedback();
 motion::Frame caps;caps.type=motion::OfficialRuntimeCaps;
 const auto bytes=Serial1.output.size();const auto commandTime=lock.lastCommandUs;
 command(caps);auto reply=lastResponse();
 assert(reply.length==2&&reply.payload[0]==27&&reply.payload[1]==motion::Ok);
 assert(!lock.armed&&Serial1.output.size()==bytes&&lock.lastCommandUs==commandTime);
 caps.length=1;command(caps);assert(lastResponse().payload[1]==motion::BadPacket);
 for(auto id:motion::kMotorIds)assert(runKpForId(id)==(id==34?10:6));
 assert(motion::kStateFreshUs==100000);
 auto f=feedback;f.imuUs=micros()-99999;assert(f.fresh(micros()));
 f.imuUs=micros()-100001;assert(!f.fresh(micros()));
 puts("PASS Type27 read-only/no lease renewal, exact official KP6/mouth10, 100ms freshness");
}
