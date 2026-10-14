// Uses the real installed/candidate handlers and their mock bus, never UART.
#define main inherited_test_main
#include TEST_FIRMWARE_FILE
#undef main
#include <fstream>

unsigned scenario = 0;
bool injected = false;
void scenarioWrite(const uint8_t* packet, size_t count) {
  const bool selected = count == 8 && packet[4] == 2 && packet[2] == 10 &&
    packet[5] == 40 && mockTorque[10] && !injected;
  if (!selected) { busWrite(packet, count); return; }
  injected = true;
  if (scenario == 0) return; // Pure silence.
  if (scenario == 1) { // Echo without a servo reply.
    Serial1.input.insert(Serial1.input.end(),packet,packet+count); return;
  }
  if (scenario == 2 || scenario == 3) {
    if (scenario == 2) corruptChecksumServo = 10; else wrongReplyIdServo = 10;
    busWrite(packet,count); corruptChecksumServo = wrongReplyIdServo = 0; return;
  }
  if (scenario == 4) { uartError(1); return; }
  if (scenario == 5) { // CPU unavailable while complete reply is queued.
    busWrite(packet,count); mockMicros += 2000; return;
  }
  if (scenario == 6 || scenario == 7 || scenario == 9) {
    busWrite(packet,count);
    std::vector<uint8_t> bytes(Serial1.input.begin(),Serial1.input.end());
    Serial1.input.clear();
    const uint32_t delay = scenario == 7 ? 1800 : scenario == 9 ? 500 : 0;
    for (size_t i=0;i<bytes.size();++i) {
      if (scenario == 6 && i<3) Serial1.input.push_back(bytes[i]);
      else Serial1.scheduledInput.emplace_back(mockMicros +
        (scenario == 6 ? 1800 : delay) + uint32_t(i*10),bytes[i]);
    }
    return;
  }
  if (scenario == 8) {
    for (unsigned i=0;i<70;++i) Serial1.input.push_back(0x42);
    return;
  }
  busWrite(packet,count); // Healthy / wrap case.
}

void resetCase(unsigned number) {
  scenario=number; injected=false;
  mockMicros=number==10 ? UINT32_MAX-18000 : 100000;
  Serial.input.clear(); Serial.output.clear(); Serial1.input.clear();
  Serial1.scheduledInput.clear(); Serial1.output.clear(); Serial1.onWrite=scenarioWrite;
  lock=motion::Interlock{}; feedback=motion::Feedback{}; sequence.reset();
  presentMask=presentFaults=torqueFaults=torqueKnown=0;
  memset(torqueReadUs,0,sizeof(torqueReadUs));
  memset(mockTorque,0,sizeof(mockTorque)); memset(mockServoFault,0,sizeof(mockServoFault));
  disablePending=torqueVerifyPending=verifyingTorque=maintenanceMode=preparingArm=false;
  servoErrors=0; servoSampleSequence=0; writes.clear(); gainReads.clear();
  Wire.online=true; Wire.busReady=true; Wire.deriveDataReady=false; Wire.onRead=nullptr;
  Wire.registers[0x1e]=3; i2cReady=imuConfigured=true; imuSettling=false; imuAddress=0x6a;
  readyFeedback(); assert(feedback.fresh(micros()) && feedback.imuOk);
  command(configuration(1)); assert(lastResponse().payload[1]==motion::Ok);
}

int main(int argc,char** argv) {
  assert(argc==2);
  std::ofstream out(argv[1],std::ios::binary);
  for (unsigned number=0;number<11;++number) {
    resetCase(number);
    const motion::Frame request=armCommand(2);
    command(request); const motion::Frame response=lastResponse();
    assert(injected);
    size_t enables=0, goals=0;
    for (const auto& w:writes) { enables+=w=="torque1"; goals+=w=="goals"; }
    assert(enables==1 && goals==1); // Never retries ARM, enable, or goals.
    if (number<9) {
      assert(!lock.armed && !lock.cookie && !feedback.torqueMask);
      assert(response.payload[1]!=motion::Ok && response.payload[3]==9 && response.payload[4]==10);
      assert(writes.back()=="torque0");
#ifdef TEST_EXTENDED
      assert(response.length==kArmReadTraceAckSize && response.payload[2]==2);
      assert(motion::u32(response.payload+62)==2 && motion::u32(response.payload+66)==123);
      assert(armFailedReadSeen && armFailedReadTrace.id==10 && armFailedReadTrace.address==40);
      const ServoReadTrace retained=armFailedReadTrace;
      motion::Frame stop; stop.type=motion::Stop;
      command(stop); lastResponse(); readyFeedback();
      assert(memcmp(&retained,&armFailedReadTrace,sizeof(retained))==0);
      // A later valid explicit ARM clears the failed evidence.
      const unsigned saved=scenario; scenario=11;
      command(armCommand(3)); assert(lastResponse().payload[1]==motion::Ok && !armFailedReadSeen);
      command(stop); lastResponse(); scenario=saved;
#endif
    } else {
      assert(lock.armed && response.length==2 && response.payload[1]==motion::Ok);
      if (number==10) assert(mockMicros<100000); // Actually crossed micros wrap.
    }
    const uint16_t n=response.length;
    out.write(reinterpret_cast<const char*>(&n),2);
    out.write(reinterpret_cast<const char*>(response.payload),n);
    // Only the initial ARM sequence, before preservation tests add operations.
    // Compare packets separately with a second identical initial-state run.
    resetCase(number); command(armCommand(2)); lastResponse();
    const uint32_t busBytes=uint32_t(Serial1.output.size());
    out.write(reinterpret_cast<const char*>(&busBytes),4);
    out.write(reinterpret_cast<const char*>(Serial1.output.data()),busBytes);
    motion::Frame stop; stop.type=motion::Stop; command(stop); lastResponse();
  }
  puts("PASS 11 exact-handler cases: silence, echo, checksum, foreign ID, UART error, queued bytes after CPU delay, partial, late, truncation, delayed success, wrap; one enable and no retries");
}
