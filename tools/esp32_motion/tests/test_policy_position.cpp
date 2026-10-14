// Actual command handlers and bus encoder, with fake UART only.
#define main originalFirmwareSuite
#include "test_firmware.cpp"
#undef main

motion::Frame goal(uint32_t seq, uint16_t value, bool direct=true) {
  motion::Frame f; f.type=motion::Target; f.sequence=seq;
  f.length=direct ? 35 : 34; motion::put32(f.payload,123);
  for (size_t i=0;i<15;++i) motion::put16(f.payload+4+2*i,value);
  f.payload[34]=motion::kPolicyPositionProfile;
  return f;
}

int main() {
  Serial1.onWrite=busWrite; Wire.registers[0x0f]=0x6b; Wire.registers[0x1e]=3;
  setup(); mockMicros+=110000; readyFeedback();
  motion::Frame caps; caps.type=motion::PolicyPositionCaps;
  const auto busBytes=Serial1.output.size(); const auto clockBefore=lock.lastCommandUs;
  command(caps); auto reply=lastResponse();
  assert(reply.type==0x80 && reply.length==2 && reply.payload[0]==24 && reply.payload[1]==0);
  assert(!lock.armed && !lock.policyPosition && Serial1.output.size()==busBytes && lock.lastCommandUs==clockBefore);
  caps.length=1; command(caps); assert(lastResponse().payload[1]==motion::BadPacket);
  command(configuration(1)); assert(lastResponse().payload[1]==motion::Ok);
  command(armCommand(2)); assert(lastResponse().payload[1]==motion::Ok);
  assert(lock.armed && !lock.policyPosition && lock.goals[0]==2047);
  auto f=goal(3,2400); command(f); assert(lastResponse().payload[1]==motion::Ok);
  mockMicros+=10000; lock.slew(micros());
  for (size_t i=0;i<15;++i) assert(lock.goals[i]==2400 && lock.continuousGoals[i]==2400);
  Serial1.output.clear(); assert(writeGoals(lock.goals));
  // SCS goal packets are byte-identical to upstream: all IDs, goals, zero
  // duration/speed, valid checksum. No gain/EEPROM writes are involved.
  const auto &packet=Serial1.output;
  assert(packet.size()==113 && packet[2]==0xfe && packet[4]==0x83 && packet[5]==42 && packet[6]==6);
  unsigned sum=0; for(size_t i=2;i<packet.size();++i) sum+=packet[i];
  assert((sum&255)==255);
  for(size_t i=0;i<15;++i) {
    size_t p=7+7*i; assert(packet[p]==motion::kMotorIds[i]);
    assert(motion::u16(packet.data()+p+1)==2400);
    assert(motion::u16(packet.data()+p+3)==0 && motion::u16(packet.data()+p+5)==0);
  }
  // Unknown profile, wrong cookie, one bad joint, and replay update neither
  // the accepted target nor its watchdog timestamp.
  auto acceptedAt=lock.lastCommandUs;
  f=goal(4,2100); f.payload[34]=2; command(f); assert(lastResponse().payload[1]==motion::BadPacket);
  f=goal(5,2100); motion::put32(f.payload,456); command(f); assert(lastResponse().payload[1]==motion::BadState);
  f=goal(6,2100); motion::put16(f.payload+32,3001); command(f); assert(lastResponse().payload[1]==motion::Range);
  f=goal(6,2100); command(f); assert(lastResponse().payload[1]==motion::BadPacket);
  assert(lock.lastCommandUs==acceptedAt && lock.desired[0]==2400 && lock.policyPosition);
  // Returning to the legacy pose path begins at the last applied goal, not
  // a stale pre-walk interpolation accumulator.
  f=goal(7,2000,false); command(f); assert(lastResponse().payload[1]==motion::Ok);
  assert(!lock.policyPosition); mockMicros+=10000; lock.slew(micros());
  assert(lock.goals[0]>=2393 && lock.goals[0]<=2394);
  f=goal(8,2300); command(f); assert(lastResponse().payload[1]==motion::Ok);
  // Freshness stop and cookie-bound first-stop diagnostics also apply to35B.
  feedback.sampleUs=micros()-motion::kStateFreshUs-1;
  checkSafety(); assert(!lock.armed && !lock.policyPosition && lock.cookie==0);
  f=goal(9,2300); command(f); reply=lastResponse();
  assert(reply.length==40 && reply.payload[1]==motion::BadState);
  motion::Frame stop; stop.type=motion::Stop; command(stop); lastResponse(); readyFeedback();
  command(armCommand(10)); assert(lastResponse().payload[1]==motion::Ok && !lock.policyPosition);
  f=goal(11,2300); command(f); assert(lastResponse().payload[1]==motion::Ok);
  lock.lastCommandUs=micros()-motion::kWatchdogUs-1;
  checkSafety(); assert(!lock.armed && !lock.policyPosition);
  command(stop); lastResponse();
  puts("PASS explicit policy goals, upstream bus bytes, legacy return, capability, range, cookie, replay, stale feedback, watchdog, STOP and new ARM");
}
