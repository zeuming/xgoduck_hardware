import assert from 'node:assert/strict';
import vm from 'node:vm';
import {Worker} from 'node:worker_threads';
import {followHeartbeatWorker} from './joint_follow.js';

function fixture(reply=()=>({ok:true,status:200,json:async()=>({heartbeat_accepted:true,generation:6})})){
  let now=0;
  const timers=[],posts=[],messages=[],self={postMessage:m=>messages.push(m)};
  vm.runInNewContext(`(${followHeartbeatWorker.toString()})();`,{self,performance:{now:()=>now},AbortController,
    setTimeout(fn,ms){timers.push({fn,ms});return timers.length;},clearTimeout(){},
    fetch:async(url,options)=>{posts.push({url,options,at:now});return reply();}});
  self.onmessage({data:{type:'start',url:'http://fixture/api/motion',token:'test',displayGapMs:1500,
    body:{action:'heartbeat',page_generation:7,generation:6,control_id:'fixture',background:false}}});
  return {self,posts,messages,async tick(ms){now+=ms;timers.filter(t=>t.ms===50).at(-1)?.fn();for(let i=0;i<20;i++)await Promise.resolve();}};
}

let f=fixture();await f.tick(50);
assert.equal(JSON.parse(f.posts[0].options.body).action,'heartbeat');
assert.equal(f.posts[0].options.credentials,'same-origin');
assert.equal(f.posts[0].options.headers['X-CSRF-Token'],'test');
assert.equal(f.posts[0].url,'http://fixture/api/motion');
// An isolated heartbeat may bridge a slow render, but cannot live forever
// without current same-session observations from the page.
for(let i=0;i<20;i++)await f.tick(50);
assert.equal(f.messages.length,0);assert.ok(f.posts.length>15);
await f.tick(500);assert.match(f.messages[0].message,/画面反馈持续中断/);
const stopped=f.posts.length;await f.tick(50);assert.equal(f.posts.length,stopped);

f=fixture();
for(let i=0;i<60;i++){if(i%4===0)f.self.onmessage({data:{type:'frame'}});await f.tick(50);}
assert.equal(f.messages.length,0,'fresh page observations sustain only the existing heartbeat');
assert.ok(f.posts.every(p=>JSON.parse(p.options.body).action==='heartbeat'),'worker cannot ARM or send a target');

f=fixture(()=>({ok:false,status:409,json:async()=>({error:'当前操作尚未完成；请求未排队、未重发'})}));
await f.tick(50);assert.equal(f.messages.length,0);
await f.tick(650);assert.match(f.messages[0].message,/心跳未得到确认/);

let delayedReplies=0;
f=fixture(()=>++delayedReplies===1?{ok:false,status:409,json:async()=>({error:'心跳回复延迟；正在核对原请求，未重发'})}
  :{ok:true,status:200,json:async()=>({heartbeat_accepted:true,generation:6})});
await f.tick(50);assert.equal(f.messages.length,0,'a delayed IPC reply is bounded, not a terminal rejection');
await f.tick(350);assert.equal(f.messages.length,0,'a subsequent confirmed heartbeat can sustain the same valid session');

f=fixture(()=>({ok:true,status:200,json:async()=>({heartbeat_accepted:true,generation:8})}));
await f.tick(50);assert.match(f.messages[0].message,/不匹配/);

// Reproduce the actual difference from the DOM-stub acceptance: the page
// thread is blocked for 600 ms, while the dedicated transport must keep
// dispatching independently. No hardware is used here.
const source=`const {parentPort}=require('node:worker_threads');
let count=0;global.self={postMessage:m=>parentPort.postMessage(m)};
global.fetch=async()=>{count++;return {ok:true,status:200,json:async()=>({heartbeat_accepted:true,generation:6})};};
(${followHeartbeatWorker.toString()})();
self.onmessage({data:{type:'start',url:'http://fixture/api/motion',token:'test',displayGapMs:1500,body:{action:'heartbeat',generation:6}}});
parentPort.postMessage({type:'ready'});setTimeout(()=>parentPort.postMessage({type:'count',count}),700);`;
const worker=new Worker(source,{eval:true});
try{
  await new Promise((resolve,reject)=>{worker.once('message',resolve);worker.once('error',reject);});
  const result=new Promise((resolve,reject)=>{worker.once('message',resolve);worker.once('error',reject);});
  const until=performance.now()+600;while(performance.now()<until){}
  assert.ok((await result).count>=8,'a slow page frame must not starve native heartbeat delivery');
}finally{await worker.terminate();}
console.log('Passed isolated heartbeat: page-stall delivery, bounded missing frames, contention deadline and generation rejection.');
