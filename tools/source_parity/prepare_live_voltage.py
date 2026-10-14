"""Prepare source-sync voltage telemetry; no device access or deployment."""
import argparse,shutil
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--baseline',type=Path,required=True);p.add_argument('--firmware',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
shutil.copytree(a.baseline,a.output,ignore=shutil.ignore_patterns('__pycache__','.pio'))
shutil.copytree(a.firmware,a.output/'firmware',ignore=shutil.ignore_patterns('__pycache__','.pio'))
def edit(name,old,new):
 f=a.output/name;s=f.read_text();assert s.count(old)==1,(name,old[:80]);f.write_text(s.replace(old,new))
h='firmware/firmware/source_cycle.h'
edit(h,'inline size_t request(uint8_t* out) {','inline size_t request(uint8_t* out, bool voltage = false) {')
edit(h,'out[5]=56; out[6]=6;','out[5]=56; out[6]=voltage ? 7 : 6;')
edit(h,'  uint8_t packet[12]={}; size_t used=0;','  uint8_t voltages[15]={};\n  bool voltage=false;\n  explicit Scan(bool withVoltage=false): voltage(withVoltage) {}\n  uint8_t packet[13]={}; size_t used=0;')
edit(h,'      if(packet[3]!=8) {discard();continue;}\n      if(used<12) return;\n      uint8_t sum=0; for(size_t i=2;i<12;++i) sum+=packet[i];','      const size_t total=voltage ? 13 : 12;\n      if(packet[3]!=(voltage ? 9 : 8)) {discard();continue;}\n      if(used<total) return;\n      uint8_t sum=0; for(size_t i=2;i<total;++i) sum+=packet[i];')
edit(h,'        loads[j]=motion::u16(packet+9); receivedUs[j]=now;','        loads[j]=motion::u16(packet+9); receivedUs[j]=now;\n        if(voltage) voltages[j]=packet[11];')
f='firmware/firmware/main.cpp'
edit(f,'size_t nextVoltage = 0;','bool sourceVoltageEnabled = false;\nsize_t nextVoltage = 0;')
edit(f,'    lock.sourceRuntime = true; updateMasks(micros());','    lock.sourceRuntime = true; sourceVoltageEnabled = false; updateMasks(micros());')
edit(f,'    if (frame.length || !lock.sourceRuntime) { ack(frame, motion::BadState); return; }','    const bool withVoltage = frame.length == 1 && frame.payload[0] == 1;\n    if ((frame.length && !withVoltage) || !lock.sourceRuntime) { ack(frame, motion::BadState); return; }\n    if (withVoltage) sourceVoltageEnabled = true;')
edit(f,'    uint8_t data[214] = {}; fillState(data);','    uint8_t data[249] = {}; fillState(data);')
edit(f,'    // Normal running replies remain the exact 170-byte source packet.','    size_t size = 170;\n    // Legacy replies retain the exact 170/214-byte source packet.')
edit(f,'      respond(0xa0,frame.sequence,data,sizeof(data));\n    } else respond(0xa0,frame.sequence,data,170);','      size = 214;\n    }\n    if (withVoltage) {\n      // Compact extension: version, seen/valid masks, 15 raw bytes, and\n      // 15 ages rounded UP to 100ms (255 means at least 25.5s old).\n      // Keeps the first-stop record and stays within the 256-byte envelope.\n      data[109] += 2;\n      uint8_t* v = data + size; v[0] = 1;\n      motion::put16(v+1, voltageSeen); uint16_t valid=voltageValid;\n      for(size_t i=0;i<15;++i) {\n        const uint32_t age=uint32_t(now-voltageReadUs[i]);\n        if(age>motion::kVoltageFreshUs || maintenanceMode) valid &= ~uint16_t(1u<<i);\n        v[5+i]=voltageRaw[i];\n        v[20+i]=age>=25500000 ? 255 : uint8_t((age+99999)/100000);\n      }\n      motion::put16(v+3,valid); size += 35;\n    }\n    respond(0xa0,frame.sequence,data,size);')
edit(f,'    source_cycle::Scan scan;','    source_cycle::Scan scan(sourceVoltageEnabled);')
edit(f,'    uint8_t packet[23]; source_cycle::request(packet);','    uint8_t packet[23]; source_cycle::request(packet,sourceVoltageEnabled);')
edit(f,'    receive(); // Include bytes present at the deadline, matching upstream.','    receive(); // Include bytes present at the deadline, matching upstream.\n    if(sourceVoltageEnabled) {\n      voltageValid=0;\n      for(size_t i=0;i<15;++i) if((scan.mask & (1u<<i)) && scan.voltages[i]) {\n        voltageRaw[i]=scan.voltages[i]; voltageReadUs[i]=scan.receivedUs[i];\n        voltageSeen |= uint16_t(1u<<i); voltageValid |= uint16_t(1u<<i);\n      }\n    }')
m='policy/motion.py'
edit(m,"        payload = self._request(SOURCE_STATE)\n        legacy = len(payload) == 170 and payload[108:110] == b'\\x01\\x01'\n        stopped = len(payload) == 214 and payload[108:110] == b'\\x01\\x02'",
"        payload = self._request(SOURCE_STATE, b'\\x01')\n        voltage = len(payload) in (205,249) and payload[108:110] in (b'\\x01\\x03',b'\\x01\\x04')\n        if voltage:\n            base_size = 214 if payload[109] == 4 else 170\n            if len(payload) != base_size+35: raise MotionError('同步电压状态长度不匹配')\n            extension, payload = payload[base_size:], payload[:base_size]\n            if extension[0] != 1: raise MotionError('同步电压版本不匹配')\n            payload = bytearray(payload); payload[109] -= 2; payload = bytes(payload)\n        legacy = len(payload) == 170 and payload[108:110] == b'\\x01\\x01'\n        stopped = len(payload) == 214 and payload[108:110] == b'\\x01\\x02'")
edit(m,"        state = decode_state(payload[:106])","        state = decode_state(payload[:106])\n        if voltage:\n            seen, valid = struct.unpack_from('<HH',extension,1)\n            if seen & ~ALL_MASK or valid & ~seen: raise MotionError('同步电压有效位无效')\n            raw = list(extension[5:20])\n            if any(valid & (1<<i) and raw[i] == 0 for i in range(15)): raise MotionError('同步电压有效值为零')\n            state['voltage'] = dict(seen_mask=seen,valid_mask=valid,raw=raw,\n                ages_us=[v*100000 for v in extension[20:35]])")
edit(m,"        result['servo_voltage_polling']=dict(enabled=False,reason='正常运控不额外轮询电压')","        result['servo_voltage_polling']=dict(enabled=bool(result.get('voltage_supported')),\n            method='grouped_registers_56_62', additional_servo_requests=0, age_resolution_ms=100)")
s='service/worker-classes.py'
edit(s,'if kind in (POLICY_POSITION_CAPS, OFFICIAL_RUNTIME_CAPS, SOURCE_RUNTIME, SOURCE_STATE) and payload != b\'\':',"if (kind in (POLICY_POSITION_CAPS, OFFICIAL_RUNTIME_CAPS, SOURCE_RUNTIME) and payload != b''\n                or kind == SOURCE_STATE and payload not in (b'', b'\\x01')):")
for f in a.output.rglob('*.py'):compile(f.read_text(),str(f),'exec')
