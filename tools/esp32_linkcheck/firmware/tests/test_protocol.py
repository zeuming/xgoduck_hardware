"""Native protocol-only tests. No serial/I2C access or ESP toolchain needed."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


SOURCE = r'''
#include "servo_readonly.h"
#include "workbench_xfer.h"
#include <cassert>
#include <string>
#include <vector>

using namespace linkcheck;

std::vector<uint8_t> frame(uint8_t id, uint8_t error, std::initializer_list<uint8_t> data) {
  std::vector<uint8_t> bytes = {255,255,id,static_cast<uint8_t>(data.size()+2),error};
  bytes.insert(bytes.end(), data.begin(), data.end());
  uint8_t sum=0;
  for (size_t i=2;i<bytes.size();++i) sum+=bytes[i];
  bytes.push_back(static_cast<uint8_t>(~sum));
  return bytes;
}

bool feed(ReplyParser& parser, const std::vector<uint8_t>& data) {
  bool found=false;
  for (uint8_t byte:data) found=parser.feed(byte)||found;
  return found;
}

bool allowed(const std::vector<uint8_t>& packet) {
  return validateXferPacket(packet.data(),packet.size())==nullptr;
}

std::string toHex(const std::vector<uint8_t>& bytes) {
  const char* hex="0123456789abcdef";
  std::string result;
  for (uint8_t byte:bytes) { result+=hex[byte>>4]; result+=hex[byte&15]; }
  return result;
}

int main() {
  // Every address/length/ID outside the fixed read-only allowlist is rejected.
  uint8_t request[8] = {};
  for (uint16_t id : {uint16_t(0), uint16_t(1), uint16_t(253)}) {
    for (unsigned address=0;address<256;++address) {
      for (unsigned size=0;size<256;++size) {
        const bool allowed=(address==5&&size==1)||(address==56&&size==6)||(address==40&&size==1);
        const auto count=makeRead(id,address,size,request);
        assert((count!=0)==allowed);
        if (allowed) {
          assert(count==8 && request[2]==id && request[4]==2);
          uint8_t sum=0;
          for (size_t i=2;i<8;++i) sum+=request[i];
          assert(sum==255);
        }
      }
    }
  }
  assert(makeRead(254,5,1,request)==0);
  assert(makeRead(255,5,1,request)==0);
  assert(makeRead(65535,5,1,request)==0);
  makeRead(10,5,1,request);

  // Echo cannot count as a servo; a later fragmented reply still works.
  ReplyParser echoed(request,1);
  assert(!feed(echoed,std::vector<uint8_t>(request,request+8)));
  assert(echoed.stats.echo==1 && !echoed.reply.found);
  auto ok=frame(10,0,{10});
  for(size_t i=0;i<ok.size()-1;++i) assert(!echoed.feed(ok[i]));
  assert(echoed.feed(ok.back()));
  assert(echoed.reply.responseId==10 && echoed.reply.data[0]==10);

  // Replies from another ID, corrupt checksums, and zero-data successful
  // replies are rejected. The parser recovers on the next valid frame.
  ReplyParser malformed(request,1);
  assert(!feed(malformed,frame(11,0,{11})));
  assert(malformed.stats.wrongId==1);
  auto corrupt=ok; corrupt.back()^=1;
  assert(!feed(malformed,corrupt));
  assert(malformed.stats.badChecksum==1);
  assert(!feed(malformed,frame(10,0,{})));
  assert(feed(malformed,ok));

  // Full and error-only fault responses mean found, not offline.
  ReplyParser fault(request,1);
  assert(feed(fault,frame(10,1,{10})));
  assert(fault.reply.found && fault.reply.error==1 && fault.reply.size==1);
  ReplyParser shortFault(request,1);
  assert(feed(shortFault,frame(10,32,{})));
  assert(shortFault.reply.found && shortFault.reply.error==32 && shortFault.reply.size==0);

  // Noise and an impossible length cannot permanently hide a later reply.
  ReplyParser noise(request,1);
  assert(!feed(noise,{7,255,255,10,255,1,2,3}));
  assert(feed(noise,ok));
  assert(noise.stats.badLength>0 && noise.stats.noise>0);

  // Position reads six bytes so its response cannot be identical to the
  // eight-byte READ echo, even with error=2 and position bytes [56,6].
  assert(makeRead(253,56,2,request)==0);
  makeRead(253,56,6,request);
  ReplyParser position(request,6);
  assert(!feed(position,std::vector<uint8_t>(request,request+8)));
  assert(feed(position,frame(253,0,{255,15,0,0,0,0})));
  assert(position.reply.size==6 && position.reply.data[0]==255 && position.reply.data[1]==15);
  ReplyParser positionFault(request,6);
  assert(!feed(positionFault,std::vector<uint8_t>(request,request+8)));
  assert(feed(positionFault,frame(253,2,{56,6,0,0,0,0})));
  assert(positionFault.reply.found && positionFault.reply.error==2 && positionFault.reply.size==6);
  // An ID-register mismatch is left intact for the caller to reject.
  makeRead(10,5,1,request);
  ReplyParser mismatched(request,1);
  assert(feed(mismatched,frame(10,0,{11})));
  assert(mismatched.reply.responseId==10 && mismatched.reply.data[0]==11);

  // XFER allows exactly the workbench's required reads, including reading
  // its initial target, and rejects all other addresses/counts/opcodes.
  for (unsigned address=0;address<256;++address) {
    for (unsigned count=0;count<256;++count) {
      const bool expected=((address==5||address==40||address==55)&&count==1)||
                          (address==42&&count==2)||(address==56&&count==6);
      assert(allowed(frame(10,2,{static_cast<uint8_t>(address),static_cast<uint8_t>(count)}))==expected);
    }
  }
  // Single-byte writes must have an exact allowed value, address and length.
  for (unsigned address=0;address<256;++address) {
    for (unsigned value=0;value<256;++value) {
      const bool expected=(address==5&&value<=253)||((address==40||address==55)&&value<=1);
      assert(allowed(frame(10,3,{static_cast<uint8_t>(address),static_cast<uint8_t>(value)}))==expected);
    }
  }
  for (unsigned op=0;op<256;++op) {
    if (op!=2 && op!=3) assert(!allowed(frame(10,op,{5,1})));
  }
  for (unsigned id=0;id<256;++id) {
    assert(allowed(frame(id,2,{5,1}))==(id<=253));
    assert(allowed(frame(id,3,{40,0}))==(id<=253));
  }
  for (uint16_t position: {uint16_t(0),uint16_t(4095),uint16_t(4096),uint16_t(65535)}) {
    assert(allowed(frame(10,3,{42,static_cast<uint8_t>(position),static_cast<uint8_t>(position>>8),0,0,0,0}))==(position<=4095));
  }
  for (size_t index=3;index<7;++index) {
    std::vector<uint8_t> goal=frame(10,3,{42,0,0,0,0,0,0});
    goal[5+index]=1;
    // Repair checksum so failure must be the payload allowlist, not checksum.
    uint8_t sum=0; for(size_t i=2;i<goal.size()-1;++i) sum+=goal[i];
    goal.back()=static_cast<uint8_t>(~sum);
    assert(!allowed(goal));
  }
  auto valid=frame(10,2,{5,1});
  auto invalid=valid; invalid[0]=0; assert(!allowed(invalid));
  invalid=valid; invalid[3]++; assert(!allowed(invalid));
  invalid=valid; invalid.back()^=1; assert(!allowed(invalid));
  invalid=valid; invalid.push_back(0); assert(!allowed(invalid));
  assert(!allowed(frame(10,3,{5,1,0})));
  assert(!allowed(frame(10,3,{42,0,0,0,0,0})));

  // Strict correlation syntax, canonical hex session, decimal uint32 bounds.
  XferRequest transfer;
  const std::string suffix=" "+toHex(valid);
  assert(parseXfer(("XFER AABBCCDD00112233 4294967295"+suffix).c_str(),transfer)==nullptr);
  assert(strcmp(transfer.session,"aabbccdd00112233")==0 && transfer.seq==UINT32_MAX);
  assert(transfer.size==8 && transfer.hasSession && transfer.hasSeq);
  assert(parseXfer(("XFER 0000000000000000 0"+suffix).c_str(),transfer)==nullptr);
  for (const char* bad : {"-1","+1","1a","4294967296"," 1","","00000000000"}) {
    assert(parseXfer((std::string("XFER 0000000000000000 ")+bad+suffix).c_str(),transfer)!=nullptr);
  }
  for (const char* badSession : {"0","00000000000000000","000000000000000g"}) {
    assert(parseXfer((std::string("XFER ")+badSession+" 1"+suffix).c_str(),transfer)!=nullptr);
  }
  assert(parseXfer("XFER 0000000000000000 1 ff",transfer)!=nullptr);
  assert(parseXfer("XFER 0000000000000000 1 ffgf",transfer)!=nullptr);
  assert(parseXfer("XFER 0000000000000000 1 f",transfer)!=nullptr);
  assert(parseXfer(("XFER 0000000000000000 1"+suffix+" ").c_str(),transfer)!=nullptr);
  assert(parseXfer(("XFER 0000000000000000 1"+suffix+"00").c_str(),transfer)!=nullptr);

  // A sequence is consumed before UART, regardless of the later outcome.
  // Switching sessions cannot make an old session's request replayable.
  XferReplayGuard guard;
  assert(guard.consume(0,0)==nullptr);
  assert(guard.consume(0,0)!=nullptr);
  assert(guard.consume(0,10)==nullptr);
  assert(guard.consume(0,9)!=nullptr);
  assert(guard.consume(1,0)==nullptr);
  assert(guard.consume(0,10)!=nullptr);
  assert(guard.consume(0,11)==nullptr);
  assert(guard.consume(0,UINT32_MAX)==nullptr);
  assert(guard.consume(0,0)!=nullptr); // wraparound is not allowed
  for (uint64_t session=2;session<kXferSessionCapacity;++session) assert(guard.consume(session,0)==nullptr);
  assert(guard.used()==kXferSessionCapacity);
  assert(guard.consume(kXferSessionCapacity,0)!=nullptr);
  assert(guard.consume(1,1)==nullptr); // existing sessions survive capacity
  assert(guard.consume(1,0)!=nullptr);
}
'''


class ProtocolTest(unittest.TestCase):
    def test_diagnostic_and_transfer_protocol(self):
        compiler = shutil.which("clang++") or shutil.which("g++")
        if not compiler:
            self.skipTest("A native C++ compiler is unavailable")
        firmware = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory(prefix=".native-test-", dir=firmware) as directory:
            executable = str(Path(directory) / "protocol-test")
            subprocess.run(
                [compiler, "-x", "c++", "-std=c++11", "-Wall", "-Wextra", "-Werror",
                 "-fsanitize=address,undefined", "-I", str(firmware), "-o", executable, "-"],
                input=SOURCE, text=True, check=True,
            )
            subprocess.run([executable], check=True)


if __name__ == "__main__":
    unittest.main()
