"""Compare original and installed-source error handling with memory-only UART.

Injected bit values are protocol stimuli, not identified physical fault types.
No original/installed production source is modified by this test.
"""
import hashlib
import json
from pathlib import Path
import subprocess

import argparse
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--firmware', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
a=p.parse_args()
ROOT = Path(__file__).resolve().parents[2]
OUT = a.output
OUT.mkdir(exist_ok=False)
UP = ROOT/'output/xgoduck_runtime_upstream/sketch'
FW = a.firmware.resolve()
config = (UP/'duck_config.h').read_text().replace('#pragma once', '').replace('#include <Arduino.h>', '')
parser = (UP/'scs_bus.h').read_text().replace('#pragma once', '').replace('#include <string.h>', '').replace('#include "duck_config.h"', '')
code = '''#define SOURCE_PARITY_NO_MAIN
#include "test_source_firmware.cpp"
namespace upstream {
struct k_mutex {};
constexpr int K_FOREVER=0;
void k_mutex_lock(k_mutex*, int) {}
void k_mutex_unlock(k_mutex*) {}
struct Sink {
 std::vector<uint8_t> input, output;
 size_t available(){return input.size();}
 int read(){assert(!input.empty());int b=input.front();input.erase(input.begin());return b;}
 void write(const uint8_t* p,size_t n){output.insert(output.end(),p,p+n);}
 void flush(){}
} Serial1;
''' + config + '\nMotor motors[NUM_MOTORS];\n' + parser + '''
k_mutex motorsMu;
}
std::vector<uint8_t> reply(uint8_t error, uint16_t position, bool corrupt=false) {
 std::vector<uint8_t> r={255,255,10,8,error,uint8_t(position),uint8_t(position>>8),0,0,0,0};
 uint8_t sum=0;for(size_t j=2;j<r.size();++j)sum+=r[j];r.push_back(uint8_t(~sum));
 if(corrupt)r.back()^=1;
 return r;
}
std::vector<uint16_t> writtenGoals;
void captureBus(const uint8_t* packet,size_t count) {
 if(packet[4]==0x83) {
   assert(count==113);writtenGoals.clear();
   for(size_t i=0;i<15;++i)writtenGoals.push_back(motion::u16(packet+8+i*7));
 }
 sourceBus(packet,count);
}
int main(){
 for(size_t i=0;i<15;++i){upstream::motors[i].id=upstream::MOTOR_IDS[i];upstream::motors[i].sign=1;}
 Serial1.onWrite=sourceBus;Wire.registers[0x0f]=0x6b;Wire.registers[0x1e]=3;
 setup();mockMicros+=110000;readyFeedback();command(configuration(1));lastResponse();negotiate();
 uint32_t seq=2;
 for(unsigned bit=0;bit<8;++bit){
   upstream::Serial1.input=reply(0,2047);upstream::parseServoRx();
   assert(upstream::motors[0].fbPos==2047);
   upstream::servoRxMask=0;upstream::Serial1.output.clear();
   upstream::Serial1.input=reply(uint8_t(1u<<bit),2100);upstream::parseServoRx();
   assert(upstream::servoRxMask==0&&upstream::motors[0].fbPos==2047&&upstream::Serial1.output.empty());
   armNow(seq++);const size_t before=writes.size();
   mockServoFault[10]=uint8_t(1u<<bit);sampleServos();
   assert(lock.armed&&!safetyDiagnostic.seen&&feedback.faultMask==1);
   assert(!(feedback.onlineMask&1)&&feedback.positions[0]==2047);
   assert(std::count(writes.begin()+before,writes.end(),"torque0")==0);
   mockServoFault[10]=0;stopNow();
   upstream::Serial1.input=reply(0,2100);upstream::parseServoRx();
   assert(upstream::servoRxMask==1&&upstream::motors[0].fbPos==2100);
 }
 armNow(seq++);const size_t before=writes.size();sampleServos();
 assert(lock.armed&&std::count(writes.begin()+before,writes.end(),"torque0")==0);
 corruptGroup=true;sampleServos();corruptGroup=false;
 assert(lock.armed&&std::count(writes.begin()+before,writes.end(),"torque0")==0);
 stopNow();
 // A visible servo error must not mask the real reason for a later watchdog.
 armNow(seq++);presentFaults=1;feedback.imuUs=micros()-100001;
 checkSafety();flushDisable();assert(!lock.armed&&safetyDiagnostic.cause==ImuStaleStop);
 presentFaults=0;stopNow();
 armNow(seq++);presentFaults=1;lock.lastCommandUs=micros()-250001;
 sampleImu();checkSafety();flushDisable();assert(!lock.armed&&safetyDiagnostic.cause==TargetWatchdogStop);
 presentFaults=0;stopNow();
 // A fault before a NEW ARM still rejects without enabling any motor.
 sampleImu();mockServoFault[10]=1;sampleServos();const size_t preArm=writes.size();
 command(armCommand(seq++));assert(lastResponse().payload[1]==motion::BadState&&!lock.armed);
 assert(std::count(writes.begin()+preArm,writes.end(),"torque1")==0);
 mockServoFault[10]=0;stopNow();
 // Every saved-range excursion, including ID32, can enter the source path.
 // Observe actual encoded bus goals, not just the internal desired array.
 Serial1.onWrite=captureBus;normalPosition=2047;
 for(size_t joint=0;joint<15;++joint)for(int side=0;side<2;++side) {
   motion::Frame cfg=configuration(seq++);
   motion::put16(cfg.payload+joint*2,side?1500:2500);
   motion::put16(cfg.payload+45+joint*2,side?1000:2100);
   motion::put16(cfg.payload+75+joint*2,side?2000:3000);
   command(cfg);assert(lastResponse().payload[1]==motion::Ok);
   sampleImu();sampleServos();const size_t before=writes.size();
   command(armCommand(seq++));assert(lastResponse().payload[1]==motion::Ok&&lock.armed);
   assert(writtenGoals.size()==15&&feedback.positions[joint]==2047);
   for(size_t i=0;i<15;++i)assert(writtenGoals[i]==(i==joint?(side?1999:2101):2047));
   assert(lock.config.contains(writtenGoals.data()));
   assert(std::find(writes.begin()+before,writes.end(),"goals")<std::find(writes.begin()+before,writes.end(),"torque1"));
   stopNow();
 }
 command(configuration(seq++));assert(lastResponse().payload[1]==motion::Ok);
 // Encoder values outside the wire domain are not a saved-range excursion.
 sampleImu();sampleServos();feedback.positions[12]=4096;
 const size_t invalidBefore=writes.size();command(armCommand(seq++));
 assert(lastResponse().payload[1]==motion::Range&&!lock.armed);
 assert(std::count(writes.begin()+invalidBefore,writes.end(),"torque1")==0);
 sampleServos();stopNow();
 // Legacy maintenance retains its original gate.
 lock.sourceRuntime=false;lock.armed=true;lock.cookie=123;presentFaults=1;
 safetyDiagnostic=SafetyDiagnostic{};checkSafety();flushDisable();
 assert(!lock.armed&&safetyDiagnostic.cause==DeviceFaultStop);
 puts("PASS: 8 isolated nonzero status bits; 30 source ARM saved-range excursions with encoded initial-goal clamp before enable; invalid wire positions rejected; startup fault, legacy, IMU, target watchdog and STOP controls pass.");
}
'''
source=OUT/'status-policy.cpp'; source.write_text(code)
binary=OUT/'status-policy'
command=['c++','-std=c++11','-Wall','-Wextra','-Werror','-Wno-unused-const-variable','-Wno-unused-function','-fsanitize=address,undefined','-g','-I',str(FW/'tests/mocks'),'-I',str(FW/'tests'),str(source),'-o',str(binary)]
build=subprocess.run(command,capture_output=True,text=True)
(OUT/'build.log').write_text(build.stdout+build.stderr)
build.check_returncode()
run=subprocess.run([str(binary)],capture_output=True,text=True)
(OUT/'run.log').write_text(run.stdout+run.stderr)
run.check_returncode()
(OUT/'result.json').write_text(json.dumps(dict(passed=True,hardware_access=False,
    installed_changes=False,upstream_commit=subprocess.check_output(['git','-C',str(UP.parent),'rev-parse','HEAD'],text=True).strip(),
    output=run.stdout,source_sha256={str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in [UP/'scs_bus.h', UP/'sketch.ino', FW/'firmware/main.cpp', FW/'firmware/motion_protocol.h',FW/'firmware/source_cycle.h']},
    limits=['Memory-only byte injection; not reproduction of physical resistance or a identified overload byte.','Original parser tested directly; full upstream loop/host stop conditions reviewed in source, not executed here.']),indent=2)+'\n')
print(run.stdout,end='')
