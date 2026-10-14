"""Compile actual upstream functions beside the candidate, using memory-only UART."""
import argparse,hashlib,json,subprocess
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--candidate',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--read-before-gains',action='store_true');a=p.parse_args()
root=Path(__file__).resolve().parents[2];up=root/'output/xgoduck_runtime_upstream/sketch'
src=(up/'scs_bus.h').read_text()
def function(name):
 start=src.index('static void '+name+'(');end=src.index('{',start);level=1;end+=1
 while level:
  level+=(src[end]=='{')-(src[end]=='}');end+=1
 return src[start:end]
functions=('scsWriteBuf','scsWriteByte','setTorqueEnable','setTempKpKd','setRunKpKd','readAllRegisters','readAllState')
header=(up/'duck_config.h').read_text().replace('#pragma once','').replace('#include <Arduino.h>','')
code='''#define SOURCE_PARITY_NO_MAIN
#include "test_source_firmware.cpp"
namespace upstream {
struct Sink {std::vector<uint8_t> output; void write(const uint8_t* p,size_t n){output.insert(output.end(),p,p+n);}void flush(){}} Serial1;
'''+header+'\nMotor motors[NUM_MOTORS];\n'+'\n'.join(function(n) for n in functions)+'''\n}
int main(){
 for(size_t i=0;i<15;++i)upstream::motors[i].id=upstream::MOTOR_IDS[i];
 upstream::readAllState();uint8_t packet[23];source_cycle::request(packet);
 assert(upstream::Serial1.output.size()==23&&!memcmp(packet,upstream::Serial1.output.data(),23));
 upstream::Serial1.output.clear();upstream::setRunKpKd();auto gains=upstream::Serial1.output;
 upstream::Serial1.output.clear();upstream::setTorqueEnable(true);auto enable=upstream::Serial1.output;
 Serial1.onWrite=sourceBus;Wire.registers[0x0f]=0x6b;Wire.registers[0x1e]=3;
 setup();mockMicros+=110000;readyFeedback();command(configuration(1));lastResponse();negotiate();sampleServos();
 Serial1.output.clear();command(armCommand(2));assert(lastResponse().payload[1]==0);
 assert(Serial1.output.size()==gains.size()+23+113+enable.size());
 assert(std::equal(gains.begin(),gains.end(),Serial1.output.begin()));
 assert(std::equal(enable.begin(),enable.end(),Serial1.output.end()-enable.size()));
 assert(source_cycle::replyUs==upstream::SERVO_REPLY_US);
 puts("PASS actual upstream functions vs actual candidate: sync-read packet, gain writes, torque enable, 6ms budget; no extra ARM transactions");
}
'''
if a.read_before_gains:
 code=code.replace('assert(std::equal(gains.begin(),gains.end(),Serial1.output.begin()));','assert(!memcmp(packet,Serial1.output.data(),23));\n assert(std::equal(gains.begin(),gains.end(),Serial1.output.begin()+23));')
a.output.mkdir(parents=True,exist_ok=True);source=a.output/'upstream-packet-oracle.cpp';source.write_text(code)
binary=a.output/'upstream-packet-oracle'
subprocess.run(['c++','-std=c++11','-Wall','-Wextra','-Werror','-Wno-unused-const-variable','-fsanitize=address,undefined','-g','-I',str(a.candidate/'firmware/tests/mocks'),'-I',str(a.candidate/'firmware/tests'),str(source),'-o',str(binary)],check=True)
r=subprocess.run([str(binary)],capture_output=True,text=True,check=True);print(r.stdout)
(a.output/'result.json').write_text(json.dumps(dict(passed=True,output=r.stdout,upstream_sha256={f:hashlib.sha256((up/f).read_bytes()).hexdigest() for f in ('duck_config.h','scs_bus.h')},hardware_access=False),indent=2)+'\n')
