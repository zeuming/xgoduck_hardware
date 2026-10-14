#pragma once
#include "motion_protocol.h"

// Upstream scs_bus.h readAllRegisters(56,6), duck_config.h SERVO_REPLY_US.
// Pure packet/parser; hardware time and I/O are owned by main.cpp.
namespace source_cycle {
constexpr uint32_t replyUs = 6000;
inline size_t request(uint8_t* out) {
  out[0]=out[1]=0xff; out[2]=0xfe; out[3]=19; out[4]=0x82;
  out[5]=56; out[6]=6;
  uint8_t sum=0;
  for(size_t i=0;i<15;++i) out[7+i]=motion::kMotorIds[i];
  for(size_t i=2;i<22;++i) sum+=out[i];
  out[22]=uint8_t(~sum); return 23;
}
struct Scan {
  uint16_t mask=0, faults=0;
  uint16_t positions[15]={}, loads[15]={}; int16_t speeds[15]={};
  uint32_t receivedUs[15]={};
  uint8_t packet[12]={}; size_t used=0;
  void feed(uint8_t byte, uint32_t now) {
    if(used==sizeof(packet)) discard();
    packet[used++]=byte;
    while(used) {
      if(packet[0]!=0xff) {discard();continue;}
      if(used<2) return;
      if(packet[1]!=0xff) {discard();continue;}
      if(used<4) return;
      if(packet[3]!=8) {discard();continue;}
      if(used<12) return;
      uint8_t sum=0; for(size_t i=2;i<12;++i) sum+=packet[i];
      size_t j=0; while(j<15 && motion::kMotorIds[j]!=packet[2]) ++j;
      if(sum!=0xff || j==15) {discard();continue;}
      const uint16_t bit=uint16_t(1u<<j);
      if(packet[4]) faults|=bit;
      else if(motion::u16(packet+5)<=4095) {
        mask|=bit; positions[j]=motion::u16(packet+5);
        speeds[j]=motion::speedSignMagnitude(motion::u16(packet+7));
        loads[j]=motion::u16(packet+9); receivedUs[j]=now;
      }
      used=0;
    }
  }
 private:
  void discard(){--used;memmove(packet,packet+1,used);}
};
}
