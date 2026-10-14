// Actual ESP32 handlers with a fake bus. No UART, board or servo access.
#define SOURCE_PARITY_NO_MAIN
#include "test_source_firmware.cpp"
#include <fstream>

static motion::Frame stateRequest;
static unsigned cases = 0;
static void stateReply(unsigned cause, unsigned error) {
  // Establish the pre-existing OFF proof before comparing traffic. The
  // first OFF SOURCE_STATE can already perform its ordinary 15 readbacks.
  refreshTorque();
  const size_t reads = singleRequests;
  command(stateRequest);
  auto response = lastResponse();
  assert(response.type == 0xa0 && response.length == 214);
  assert(response.payload[108] == 1 && response.payload[109] == 2);
  assert(response.payload[170] == motion::Target && response.payload[171] == motion::BadState);
  assert(response.payload[172] == 1 && response.payload[173] == cause);
  assert(response.payload[174] == error && motion::u32(response.payload+210) == 123);
  assert(singleRequests == reads); // already fresh OFF readback, no new poll.
  ++cases;
}

int main(int argc, char** argv) {
  Serial1.onWrite=sourceBus; Wire.registers[0x0f]=0x6b; Wire.registers[0x1e]=3;
  setup(); mockMicros+=110000; readyFeedback(); command(configuration(1)); lastResponse();
  negotiate(); stateRequest.type=motion::SourceState;
  armNow(2); command(stateRequest); auto running=lastResponse();
  assert(running.length==170 && running.payload[109]==1); ++cases;
  // SOURCE mode does not stop for an incomplete position scan. The first
  // cause MUST be target watchdog, not the unrelated stale position/mask.
  feedback.onlineMask=0; presentMask=0; feedback.sampleUs=micros()-300000;
  lock.lastCommandUs=micros()-250001; feedback.imuUs=micros(); feedback.imuOk=true;
  checkSafety();
  if(argc==2 && !strcmp(argv[1],"--characterize")) {
    printf("{\"armed\":%s,\"cause\":%u,\"error\":%u,\"target_age_us\":%u,\"imu_age_us\":%u,\"position_age_us\":%u,\"online_mask\":%u}\n",
      lock.armed?"true":"false",safetyDiagnostic.cause,safetyDiagnostic.error,
      safetyDiagnostic.targetAge,safetyDiagnostic.imuAge,safetyDiagnostic.positionAge,safetyDiagnostic.online);
    return 0;
  }
  assert(safetyDiagnostic.cause==TargetWatchdogStop);
  flushDisable(); stateReply(TargetWatchdogStop,motion::Stale);
  auto retained=safetyDiagnostic; stopNow(); stateReply(TargetWatchdogStop,motion::Stale);
  assert(safetyDiagnostic.atUs==retained.atUs && safetyDiagnostic.targetAge==retained.targetAge);
  if(argc==2) {
    command(stateRequest); auto response=lastResponse();
    std::ofstream out(argv[1],std::ios::binary);
    out.write(reinterpret_cast<const char*>(response.payload),response.length);
  }
  missing=0; armNow(3); assert(!safetyDiagnostic.seen);
  command(stateRequest); assert(lastResponse().length==170); ++cases;
  feedback.imuOk=false; checkSafety(); flushDisable(); stateReply(ImuInvalidStop,motion::Stale); stopNow();
  armNow(4); presentMask=0; feedback.onlineMask=0; feedback.imuUs=micros()-100001;
  checkSafety(); flushDisable(); stateReply(ImuStaleStop,motion::Stale); stopNow();
  armNow(5); presentFaults=1; checkSafety(); flushDisable(); stateReply(DeviceFaultStop,motion::BadState);
  presentFaults=0; stopNow();
  armNow(6); motion::Frame target; target.type=motion::SourceTarget; target.sequence=7;
  target.length=35; motion::put32(target.payload,123); target.payload[34]=1;
  for(size_t i=0;i<15;++i) motion::put16(target.payload+4+i*2,2200);
  motion::put16(target.payload+4,999); command(target);
  assert(Serial.output.empty()); stateReply(InternalStop,motion::Range); stopNow();
  armNow(8); requestDisarm(motion::BadState,GoalWriteStop); flushDisable();
  stateReply(GoalWriteStop,motion::BadState); stopNow();
  armNow(9); const auto beforeSingles=singleRequests;
  for(int i=0;i<20;++i) {command(stateRequest);assert(lastResponse().length==170);}
  assert(singleRequests==beforeSingles); ++cases;
  stopNow(); command(stateRequest);assert(lastResponse().length==170); ++cases;
  // Legacy profile really DOES gate the complete position mask. Match its
  // existing cause selection too; the diagnostic change cannot alter gates.
  lock.sourceRuntime=false; lock.armed=true; lock.cookie=123; safetyDiagnostic=SafetyDiagnostic{};
  presentMask=0; torqueKnown=motion::kAllMotors; feedback.imuOk=true;
  feedback.imuUs=feedback.sampleUs=lock.lastCommandUs=micros();
  checkSafety();assert(!lock.armed && safetyDiagnostic.cause==OfflineStop);++cases;
  printf("PASS %u source-stop cases; original running STATE bytes and safety limits preserved\n",cases);
}
