import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';
import { STLLoader } from './vendor/STLLoader.js';
import { createRig, applyPose as pose } from './kinematics.js';
import { CalibrationPanel } from './calibration.js';
import { JointFollowPanel } from './joint_follow.js';

const $ = id => document.getElementById(id);
const clamp = (n, a, b) => Math.max(a, Math.min(b, n));
let meta, robot, ghost, replays = [], simulation, dataset, mode = 'live';
let playing = false, busy = false, time = 0, group = 'left', selected = 3, current;
let livePolling=false, liveData=null, liveSequence=null, liveStarted=0, liveBusy=false, liveLastReply=0;
let manual, jointInfo = [], rowNodes = [], lastBase = new THREE.Vector3(), view = 'iso';
let jointFollow, jointViewGeneration=null;
let calibration, demoKind=null, demoIndex=3;
const groupIndices={left:[0,1,2,3,4],right:[9,10,11,12,13],head:[5,6,7,8,14]};
let jointTags=[];
const scene = new THREE.Scene();
const jointMarker=new THREE.Mesh(new THREE.SphereGeometry(.006,16,12),new THREE.MeshBasicMaterial({color:0x8fe0c5,depthTest:false,transparent:true,opacity:.9}));
jointMarker.renderOrder=10;jointMarker.visible=false;scene.add(jointMarker);
const jointAxis=new THREE.Line(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:0x8fe0c5,depthTest:false,transparent:true,opacity:.95}));
jointAxis.geometry.setAttribute('position',new THREE.BufferAttribute(new Float32Array(6),3));
jointAxis.renderOrder=10;jointAxis.visible=false;scene.add(jointAxis);
scene.fog = new THREE.FogExp2(0x172027, .75);
const camera = new THREE.PerspectiveCamera(36, 1, .005, 30);
camera.up.set(0, 0, 1);
const renderer = new THREE.WebGLRenderer({antialias:true, alpha:true});
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.25;
$('viewport').append(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.minDistance = .22;
controls.maxDistance = 3;
controls.maxPolarAngle = Math.PI * .49;
const ambient = new THREE.HemisphereLight(0xd7efff, 0x52616a, 2.2);
ambient.up.set(0,0,1); ambient.position.set(0,0,1); scene.add(ambient);
const sun = new THREE.DirectionalLight(0xffe3c4, 3.5);
sun.position.set(.4,-.6,1.4); sun.castShadow = true;
sun.shadow.mapSize.set(2048,2048);
Object.assign(sun.shadow.camera, {left:-1.5,right:1.5,top:1.5,bottom:-1.5,near:.1,far:5});
sun.shadow.bias = -.00004;
sun.shadow.normalBias = .0005;
sun.shadow.radius = 3; scene.add(sun, sun.target);
const fill = new THREE.DirectionalLight(0x94bad5, 1.6);fill.position.set(-.5,.7,.7);scene.add(fill);
const floor = new THREE.Mesh(new THREE.PlaneGeometry(30,30), new THREE.MeshStandardMaterial({color:0x1a252c,roughness:1}));
floor.receiveShadow = true;floor.position.z = -.001;scene.add(floor);
const grid = new THREE.GridHelper(10,200,0x3e515c,0x2b3d47);
grid.rotation.x = Math.PI / 2;grid.position.z=.0001;grid.material.transparent=true;grid.material.opacity=.45;scene.add(grid);
const origin = new THREE.Mesh(new THREE.RingGeometry(.19,.1905,96),new THREE.MeshBasicMaterial({color:0x677766,transparent:true,opacity:.4,side:THREE.DoubleSide}));
origin.position.z=.0005;scene.add(origin);
const trajectory = new THREE.Line(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:0xdf9e68,transparent:true,opacity:.6}));scene.add(trajectory);
const arrows = new THREE.Group();
arrows.add(new THREE.ArrowHelper(new THREE.Vector3(1,0,0),new THREE.Vector3(-.21,-.21,.001),.075,0xd09280,.012,.006));
arrows.add(new THREE.ArrowHelper(new THREE.Vector3(0,1,0),new THREE.Vector3(-.21,-.21,.001),.075,0x92bb9c,.012,.006));scene.add(arrows);

function message(text='', error=false) {$('message').textContent=text;$('message').classList.toggle('error',error);}
async function api(url, params) {
  const abort = new AbortController();
  const timeout = url==='/api/live'?setTimeout(()=>abort.abort(),1500):null;
  try {
    const options=params?{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(params)}:{};
    const response = await fetch(url, {...options,signal:abort.signal});
    const data = await response.json();if(!response.ok) throw Error(data.error || `HTTP ${response.status}`);return data;
  } finally {if(timeout)clearTimeout(timeout);}
}
function setView(name = view) {
  view = name;
  const center = current ? new THREE.Vector3(...current.base) : new THREE.Vector3(0,0,.14);
  center.z = ['live','calibration'].includes(mode)?.22:.12;
  const offsets = {iso:[.43,-.48,.29],front:[.67,0,.1],side:[0,-.67,.1],top:[.001,0,.78]};
  const offset=new THREE.Vector3(...offsets[name]);
  if(mode==='joints'&&robot){
    const bounds=new THREE.Box3().setFromObject(robot.root),sphere=bounds.getBoundingSphere(new THREE.Sphere());
    if(Number.isFinite(sphere.radius)&&sphere.radius>0){
      center.copy(sphere.center);
      const vertical=THREE.MathUtils.degToRad(camera.fov)/2;
      const halfAngle=Math.min(vertical,Math.atan(Math.tan(vertical)*camera.aspect));
      offset.setLength(Math.max(.3,sphere.radius*1.15/Math.sin(halfAngle)));
    }
  }
  controls.target.copy(center); camera.position.copy(center).add(offset);controls.update();
  document.querySelectorAll('[data-view]').forEach(b=>b.classList.toggle('active',b.dataset.view===name));
}
function material(name, isGhost) {
  if(isGhost)return new THREE.MeshBasicMaterial({color:0xd79df7,wireframe:true,transparent:true,opacity:.12,depthWrite:false});
  let color=0xc9d2ce, metalness=.16;
  if(name==='head_roll'||name==='jaw_moving'||name.includes('ankle'))color=0xec9349;
  else if(name.includes('hip_yaw')||name==='neck_pitch'||name==='head_yaw') {color=0x344149;metalness=.35;}
  return new THREE.MeshStandardMaterial({color,metalness,roughness:.52});
}
function assemble(geometry, isGhost=false) {
  const rig=createRig(meta.tree,(spec,index)=>{
    const mesh=new THREE.Mesh(geometry[spec.name],material(spec.name,isGhost));
    mesh.scale.fromArray(meta.assets[spec.name].scale);mesh.castShadow=!isGhost;mesh.receiveShadow=!isGhost;mesh.userData.joint=index;return mesh;
  });
  jointInfo=rig.joints.map(j=>j.info);scene.add(rig.root);return rig;
}
function selectJoint(index) {
  if(mode==='calibration'&&calibration&&!calibration.select(index))return false;
  selected=index;$('chart-title').textContent=meta.labels[index]+' · 角度曲线';
  rowNodes.forEach(({row,index:i})=>row.classList.toggle('selected',i===index));
  // A CAD link can contain the next servo's housing. Highlight the output
  // shaft below, never present the whole link mesh as this servo's identity.
  robot.meshes.forEach(m=>m.material.emissive.setHex(0));
  drawChart();
  return true;
}
function rebuildJoints() {
  $('joint-list').replaceChildren();rowNodes=[];
  const indices=groupIndices[group];
  indices.forEach(index=>{
    const row=document.createElement('div');row.className='joint-row';
    const line=document.createElement('div');line.className='joint-line';
    const id=document.createElement('span');id.className='joint-id';id.textContent=meta.motor_ids[index];
    const name=document.createElement('span');name.className='joint-name';name.textContent=meta.labels[index];
    const value=document.createElement('span');value.className='joint-value';
    const delta=document.createElement('span');delta.className='joint-delta';
    line.append(id,name,value,delta);row.append(line);
    row.addEventListener('click',()=>selectJoint(index));
    row.tabIndex=0;row.setAttribute('role','button');row.setAttribute('aria-label',`观察${meta.labels[index]}`);
    row.addEventListener('keydown',e=>{if(e.target===row&&(e.key==='Enter'||e.key===' ')){e.preventDefault();selectJoint(index);}});
    let slider;
    if(mode==='joints'){
      slider=document.createElement('input');slider.type='range';slider.min=jointInfo[index].range[0];slider.max=jointInfo[index].range[1];slider.step=.1;slider.value=manual.q[index];
      slider.setAttribute('aria-label',meta.labels[index]+'角度');
      slider.addEventListener('input',()=>{selectJoint(index);if(jointFollow?.active){jointFollow.target(meta.motor_ids[index],Number(slider.value));return;}if(jointFollow?.busy)return;manual.q[index]=Number(slider.value);manual.target=null;displayFrame(manual);});row.append(slider);
    }
    rowNodes.push({row,index,value,delta,slider});$('joint-list').append(row);
  });
  selectJoint(indices.includes(selected)?selected:indices[0]);
  rebuildJointTags();
  if(current)updateJointValues(current);
  syncFollowControls();
}
function syncFollowControls(){
  if(mode!=='joints'||!jointFollow)return;
  const motion=jointFollow.snapshot?.motion,h=motion?.hardware;
  $('connection-badge').textContent=h?`● X5 已连接 · ${h.online_ids?.length||0}/15 · ${h.mode==='off'&&h.release_confirmed?'已卸力':jointFollow.active?'随动中':h.armed?'扭矩已开启':'卸力待确认'}`
    :motion?.owns_connection?'● X5 已连接 · 等待画面':'○ X5 未连接';
  rowNodes.forEach(({index,slider})=>{if(!slider)return;const j=jointFollow.joint(meta.motor_ids[index]);
    slider.disabled=jointFollow.busy||!!jointFollow.stopPending||!!jointFollow.feedbackWaiting;
    slider.min=jointFollow.active&&j?j.min_deg:jointInfo[index].range[0];
    slider.max=jointFollow.active&&j?j.max_deg:jointInfo[index].range[1];
    if(document.activeElement!==slider)slider.value=jointFollow.active&&j?j.requested_deg:manual.q[index];
  });
  if(!jointFollow.active){ghost.root.visible=false;$('run-status').textContent='仅模型预览 · 随动未启用';}
  else if(jointFollow.feedbackWaiting){$('run-status').textContent='等待画面 · 关节目标冻结';}
}
function rebuildJointTags(){
  $('joint-tags').replaceChildren();$('joint-leaders').replaceChildren();jointTags=[];
  for(const index of groupIndices[group]){
    const button=document.createElement('button');button.className='joint-tag';
    button.textContent=`ID${meta.motor_ids[index]} · ${meta.labels[index]}轴`;
    button.setAttribute('aria-label',`选择 ${meta.labels[index]} ID${meta.motor_ids[index]} 输出轴`);
    button.onclick=()=>selectJoint(index);$('joint-tags').append(button);
    const line=document.createElementNS('http://www.w3.org/2000/svg','line');
    const dot=document.createElementNS('http://www.w3.org/2000/svg','circle');dot.setAttribute('r','3');
    $('joint-leaders').append(line,dot);jointTags.push({index,button,line,dot});
  }
}
function updateJointTags(){
  const visible=!!robot&&['calibration','live','joints'].includes(mode)&&$('show-joint-ids').checked;
  $('joint-annotations').hidden=!visible;jointMarker.visible=visible;jointAxis.visible=visible;
  if(!visible)return;
  robot.root.updateMatrixWorld(true);const box=$('viewport'),width=box.clientWidth,height=box.clientHeight;
  $('joint-leaders').setAttribute('viewBox',`0 0 ${width} ${height}`);
  const points=jointTags.map(tag=>{
    const j=robot.joints[tag.index],world=j.moving.localToWorld(j.pos.clone()),p=world.clone().project(camera);
    return {...tag,x:(p.x+1)*width/2,y:(1-p.y)*height/2,inView:p.z>-1&&p.z<1&&Math.abs(p.x)<=1&&Math.abs(p.y)<=1};
  });
  // Keep labels readable and in numerical order; leader endpoints use exact
  // MJCF hinge anchors, even for shafts hidden behind a housing (ID10/20).
  const tagHeight=jointTags[0]?.button.offsetHeight||28,spacing=tagHeight+6;
  const maxTop=Math.max(8,height-(mode==='joints'?72:130)-tagHeight-(points.length-1)*spacing);
  const minTop=mode==='joints'?(width<600?40:140):190;
  const top=clamp(points.reduce((s,p)=>s+p.y,0)/points.length-points.length*spacing/2,Math.min(minTop,maxTop),maxTop);
  points.forEach(({index,button,line,dot,x,y,inView},n)=>{
    const ty=top+n*spacing,on=index===selected;
    button.style.top=`${ty}px`;button.classList.toggle('selected',on);button.setAttribute('aria-pressed',String(on));
    button.title=inView?'引线圆点是输出轴位置；被外壳遮住时仍显示':'输出轴在当前画面外，请居中查看';
    line.style.display=dot.style.display=inView?'':'none';
    for(const [k,v] of Object.entries({x1:14+button.offsetWidth,y1:ty+tagHeight/2,x2:x,y2:y}))line.setAttribute(k,String(v));
    dot.setAttribute('cx',String(x));dot.setAttribute('cy',String(y));
    line.setAttribute('class',on?'selected':'');dot.setAttribute('class',on?'selected':'');
  });
  const joint=robot.joints[selected];jointMarker.position.copy(joint.moving.localToWorld(joint.pos.clone()));
  const ends=[-.016,.016].map(d=>joint.moving.localToWorld(joint.pos.clone().addScaledVector(joint.axis,d)));
  ends.forEach((p,i)=>jointAxis.geometry.attributes.position.setXYZ(i,p.x,p.y,p.z));
  jointAxis.geometry.attributes.position.needsUpdate=true;jointAxis.geometry.computeBoundingSphere();
}
function updateJointValues(frame){
  rowNodes.forEach(({index,value,delta})=>{
    value.textContent=Number.isFinite(frame.q[index])?frame.q[index].toFixed(1)+'°':'—';
    value.title=frame.kind==='live'?'实机编码器经已保存校准换算的原始角度':'实际 / 仿真角度';
    const diff=Number.isFinite(frame.target?.[index])&&Number.isFinite(frame.q[index])?frame.target[index]-frame.q[index]:null;
    delta.textContent=diff===null?'—':(diff>=0?'+':'')+diff.toFixed(1)+'°';
    delta.title='目标减实际角度';
  });
}
function displayFrame(frame){
  current=frame;pose(robot,frame);ghost.root.visible=!!frame.target&&mode!=='calibration'&&(mode==='joints'?!!jointFollow?.active:$('ghost').checked);
  if(ghost.root.visible)pose(ghost,frame,true);
  const next=new THREE.Vector3(...frame.base);
  if($('follow').checked){const delta=next.clone().sub(lastBase);delta.z=0;camera.position.add(delta);controls.target.add(delta);}
  lastBase.copy(next);sun.position.set(next.x+.4,next.y-.6,1.4);sun.target.position.set(next.x,next.y,0);
  $('tilt').innerHTML=mode==='joints'?'—':frame.tilt.toFixed(1)+'<small> °</small>';
  $('distance').innerHTML=mode==='simulation'?((frame.base[0]-(dataset?.frames[0].base[0]||0))*100).toFixed(1)+'<small> cm</small>':'—';
  $('velocity').innerHTML=['joints','live','calibration'].includes(mode)?'—':frame.command[0].toFixed(2)+'<small> m/s</small>';
  ['left','right'].forEach(side=>$(side+'-contact').classList.toggle('contact',mode==='simulation'&&!!frame.contacts?.[side]));
  updateJointValues(frame);
}
function frameAt(t){
  const frames=dataset?.frames;if(!frames?.length)return null;
  let lo=0,hi=frames.length-1;
  while(lo<hi){const mid=Math.ceil((lo+hi)/2);if(frames[mid].t<=t)lo=mid;else hi=mid-1;}
  return frames[lo]; // Sample-and-hold preserves actual recorded samples, including missing targets.
}
function seek(t){
  if(['live','calibration'].includes(mode))return message('正在显示最新实机反馈；下方曲线保留最近 10 秒。');
  if(mode==='joints')return message('关节调试使用右侧滑块；切换到仿真或回放后可操作时间轴。');
  if(!dataset)return;
  time=clamp(t,0,dataset.duration);$('timeline').value=time;$('current-time').textContent=time.toFixed(2).padStart(5,'0');
  const frame=frameAt(time);if(frame)displayFrame(frame);drawChart();
}
function setPlaying(value){
  playing=value;$('play').textContent=value?'Ⅱ':'▶';$('play').setAttribute('aria-label',value?'暂停':'播放');
  if(['live','calibration'].includes(mode))return;
  $('run-status').textContent=mode==='joints'?'手动姿态':busy?'正在求解':value?(mode==='replay'?'记录回放中':'仿真播放中'):'已暂停';
  $('run-status').classList.toggle('paused',!value);
}
function setDataset(data){
  dataset=data;setPlaying(false);time=0;$('timeline').max=data.duration;$('duration').textContent=data.duration.toFixed(2).padStart(5,'0');
  $('evidence-note').textContent=data.note;$('markers').replaceChildren();
  data.markers.forEach(marker=>{if(marker.t>data.duration)return;const b=document.createElement('button');b.textContent=marker.label;b.style.left=clamp(marker.t/data.duration*100,0,100)+'%';b.addEventListener('click',()=>{setPlaying(false);seek(marker.t);});$('markers').append(b);});
  trajectory.geometry.dispose();trajectory.geometry=new THREE.BufferGeometry().setFromPoints(data.kind==='simulation'?data.frames.map(f=>new THREE.Vector3(f.base[0],f.base[1],.001)):[]);
  trajectory.visible=data.kind==='simulation';
  $('footer-state').textContent=data.kind==='simulation'?`${meta.physics} · 50 Hz 策略 / 200 Hz 物理`:`${data.frames.length} 帧实机记录 · ${data.truncated?'部分记录':'记录未截断'}`;
  seek(0);setView();
}
function setMode(next){
  const previous=mode;
  if(previous==='joints'&&next!=='joints')jointFollow?.leave();
  if(previous==='calibration'&&next!=='calibration')calibration?.leave();
  mode=next;jointViewGeneration=null;document.body.dataset.workspaceMode=next;setPlaying(false);message();
  if(previous==='live'&&next!=='live')api('/api/live/disconnect',{}).catch(e=>message(e.message,true));
  $('run-status').classList.remove('stale');
  document.querySelectorAll('[data-mode]').forEach(b=>{b.classList.toggle('active',b.dataset.mode===next);b.setAttribute('aria-pressed',String(b.dataset.mode===next));});
  ['live','calibration','simulation','replay','joint'].forEach(n=>$(n+'-controls').hidden=next!==(n==='joint'?'joints':n));
  document.querySelector('.joint-preview-actions').hidden=next!=='joints';
  document.querySelector('.joint-speed-control').hidden=next!=='joints';
  $('joint-list-title').textContent=next==='joints'?'关节控制':'关节观察';
  if(next!=='joints')$('joint-follow-details').close();
  const titles={calibration:['VISUAL CALIBRATION','选哪里，就核对哪里。','实机反馈 · 关节高亮 · 模型示范','关节校准','X5 · 正式校准'],live:['LIVE DIGITAL TWIN','实机动，模型跟着动。','实时编码器 · IMU 姿态 · 只读同步','实时反馈','X5 → ESP32'],simulation:['PHYSICS SIMULATION','让动作看得见。','真实关节结构 · 接触物理 · 行走策略','仿真指令','MUJOCO + ONNX'],replay:['HARDWARE REPLAY','把现场慢下来。','实测关节角 · IMU 倾斜 · 目标对照','历史记录','RECORDED DATA'],joints:['JOINT EXPLORER','每个关节，单独看。','模型预览 · 可选真机随动 · 实测与目标对照','姿态编辑','KINEMATICS']};
  ['mode-eyebrow','view-title','view-subtitle','panel-title','engine'].forEach((id,i)=>$(id).textContent=titles[next][i]);
  $('record-label').textContent={calibration:'只读关节反馈 · 最近 10 秒',live:'实时反馈 · 最近 10 秒',simulation:'物理仿真',replay:'实机记录 · 固定机身位置',joints:'手动姿态'}[next];
  $('distance-source').textContent=next==='simulation'?'相对初始位置':'无世界坐标数据';
  $('tilt-source').textContent=['replay','live'].includes(next)?'实测 IMU 重力方向':next==='simulation'?'仿真计算':'姿态编辑不计算倾角';
  $('contact-source').textContent=next==='simulation'?'MuJoCo 碰撞检测':'没有接触传感数据';
  if(next==='live'){
    dataset={kind:'live',frames:[],duration:10,markers:[]};liveSequence=null;liveStarted=performance.now();
    trajectory.visible=false;ghost.root.visible=false;$('markers').replaceChildren();
    $('evidence-note').textContent='等待实时反馈。接收距今是本机收到数据后的时间，不是完整端到端延迟。未测量机身位移与绝对偏航。';
    $('timeline').max=10;$('timeline').value=10;$('duration').textContent='LIVE';$('current-time').textContent='00.00';
    $('run-status').textContent='等待连接';$('run-status').classList.add('paused');
    $('tilt').textContent=$('distance').textContent=$('velocity').textContent='—';
    $('left-contact').classList.remove('contact');$('right-contact').classList.remove('contact');
    $('footer-state').textContent='只读反馈 · 尚未收到实机数据';pollLive();
  }
  else if(next==='calibration'){
    dataset={kind:'live',frames:[],duration:10,markers:[]};liveStarted=performance.now();trajectory.visible=false;ghost.root.visible=false;
    displayFrame({...current,base:[0,0,.22],quat:[1,0,0,0],target:null});
    $('markers').replaceChildren();$('duration').textContent='LIVE';$('current-time').textContent='00.00';
    $('tilt-source').textContent='模型机身固定，仅核对关节相对姿态';
    $('tilt').textContent=$('velocity').textContent='—';$('footer-state').textContent='三维校准 · 点击采样，预览后保存';
    $('evidence-note').textContent='亮色关节与实机编码器同步。紫色仅为模型示范；采样和保存由你逐步点击，实机不自动动作。';
    calibration.enter();setView();
  }
  else if(next==='simulation'){if(simulation)setDataset(simulation);else generate(false);}
  else if(next==='replay')loadRecord();
  else{
    dataset=null;trajectory.visible=false;
    manual={q:[...meta.home],target:null,base:[0,0,simulation?.frames[0].base[2]||.14],quat:[1,0,0,0],tilt:0,command:[0,0,0]};
    displayFrame(manual);setView();$('evidence-note').textContent='未启用真机随动时，滑块仅改变页面模型。姿态编辑不计算重力、碰撞或承重，包含 ID34 嘴部开合。';
    $('markers').replaceChildren();$('current-time').textContent='00.00';$('duration').textContent='00.00';$('timeline').value=0;
    $('footer-state').textContent='15 关节运动链 · 真机随动需显式启用';
    jointFollow?.enter();
  }
  rebuildJoints();drawChart();
}
function loadRecord(){
  const record=replays.find(r=>r.id===$('record').value);
  if(!record){dataset=null;return message('当前没有可用的实机记录。',true);}
  setDataset(record);$('record-reason').textContent='记录结束原因：'+(record.reason||'未记录');
}
async function generate(autoplay=true){
  if(busy)return message('上一段物理仿真正在计算，完成后即可再次运行。');
  busy=true;setPlaying(false);message('正在运行 ONNX 策略与接触物理…');$('generate').textContent='正在求解…';
  try{
    simulation=await api('/api/simulate',{speed:Number($('speed').value),duration:Number($('sim-duration').value)});
    if(mode==='simulation'){setDataset(simulation);message();if(simulation.reason!=='完成指定仿真时长')message(simulation.reason);if(autoplay)setPlaying(true);}
  }catch(error){message(error.message,true);}finally{busy=false;$('generate').innerHTML='运行物理仿真 <span>↗</span>';setPlaying(playing);}
}
function drawChart(){
  const canvas=$('chart'), dpr=Math.min(devicePixelRatio,2), w=canvas.clientWidth,h=canvas.clientHeight;
  if(!w)return;canvas.width=w*dpr;canvas.height=h*dpr;const ctx=canvas.getContext('2d');ctx.scale(dpr,dpr);
  const frames=dataset?.frames;
  ctx.strokeStyle='#2c3b44';ctx.lineWidth=.5;for(let i=0;i<3;i++){ctx.beginPath();ctx.moveTo(0,12+i*30);ctx.lineTo(w,12+i*30);ctx.stroke();}
  if(!frames?.length){ctx.fillStyle='#667e89';ctx.font='10px sans-serif';ctx.fillText(mode==='live'?'等待实时关节反馈':'选择仿真或回放，查看角度曲线',12,49);return;}
  const values=frames.flatMap(f=>[f.q[selected],...(f.target?[f.target[selected]]:[])]).filter(Number.isFinite);
  if(!values.length){ctx.fillStyle='#849298';ctx.fillText('本模式没有该关节的测量或仿真数据',12,49);return;}
  const start=['live','calibration'].includes(mode)?Math.max(0,time-10):0;
  const span=['live','calibration'].includes(mode)?10:dataset.duration;
  const min=Math.min(...values)-2,max=Math.max(...values)+2,x=t=>8+(t-start)/span*(w-16),y=v=>h-9-(v-min)/(max-min)*(h-20);
  for(const [key,color] of [['target','#ce8df0'],['q','#8fe0c5']]){ctx.strokeStyle=color;ctx.lineWidth=1.25;ctx.beginPath();let pen=false;frames.forEach(f=>{if(!Number.isFinite(f[key]?.[selected])){pen=false;return;}if(pen)ctx.lineTo(x(f.t),y(f[key][selected]));else ctx.moveTo(x(f.t),y(f[key][selected]));pen=true;});ctx.stroke();}
  ctx.setLineDash([3,3]);ctx.strokeStyle='#ffae67';ctx.beginPath();ctx.moveTo(x(time),0);ctx.lineTo(x(time),h);ctx.stroke();ctx.setLineDash([]);
  ctx.font='8px monospace';ctx.fillStyle='#8097a2';ctx.fillText(max.toFixed(0)+'°',3,9);ctx.fillText(min.toFixed(0)+'°',3,h-1);
}

async function pollLive(){
  if(livePolling||mode!=='live')return;
  livePolling=true;
  try{
    liveData=await api('/api/live');
    liveLastReply=performance.now();
    if(mode!=='live')return;
    const f=liveData.feedback;
    $('live-target').textContent=liveData.target;
    const recent=liveData.active&&liveData.receive_age_ms!==null&&liveData.receive_age_ms<500;
    $('live-online').textContent=recent&&f?f.online_ids.length+' / 15':'— / 15';
    $('live-age').textContent=liveData.receive_age_ms===null?'— ms':Math.round(liveData.receive_age_ms)+' ms';
    $('live-hz').textContent=(recent?liveData.rate_hz:0).toFixed(1)+' Hz';
    $('live-torque').textContent=liveData.fresh&&f?f.armed||f.torque_ids.length?'有扭矩':'扭矩关闭':'未知';
    $('connection-badge').textContent=liveData.fresh?'● 实机反馈已连接':'○ 实机反馈未就绪';
    const status=liveData.fresh?'实时同步':liveData.error?'反馈异常':liveData.active?f?'反馈不完整':'正在连接':'未连接实机';
    $('run-status').textContent=status;$('run-status').classList.toggle('stale',!liveData.fresh);$('run-status').classList.toggle('paused',!liveData.fresh);
    if(liveData.error)message(liveData.error,true);
    else if(liveData.active&&f&&!liveData.fresh){
      const absent=meta.motor_ids.filter(id=>!f.online_ids.includes(id));
      message(absent.length?'未收到关节反馈：ID '+absent.join('、')+'。请检查舵机供电；画面保留上一姿态。':!f.imu_ok?'IMU 数据或保存校准尚未就绪；画面保留上一姿态。':'反馈已过期，画面已冻结。',true);
    }else if(liveData.fresh){
      message(f.armed||f.torque_ids.length?'当前有舵机扭矩，请勿强行掰动关节。页面只读取反馈。':!Number.isFinite(f.q[14])?'ID34 嘴部反馈缺失，嘴部保留最后姿态，角度显示未知。':'');
    }
    if(liveData.fresh&&f&&f.sequence!==liveSequence){
      liveSequence=f.sequence;
      const t=(performance.now()-liveStarted)/1000;
      const first=dataset.frames.length===0;
      const frame={kind:'live',q:f.q,target:null,base:[0,0,.22],quat:f.quat,tilt:f.tilt,command:[0,0,0],t};
      displayFrame(frame);
      if(first)setView();
      dataset.frames.push(frame);dataset.frames=dataset.frames.filter(p=>p.t>=t-10);
      // Store a sliding time window while retaining real measured joint angles.
      time=t;dataset.duration=Math.max(10,t);$('current-time').textContent=t.toFixed(2);$('timeline').max=dataset.duration;$('timeline').value=t;
      $('evidence-note').textContent=`实机编码器 + 已保存校准（修订 ${f.calibration_revision}）+ IMU；串口往返 ${f.uart_rtt_ms.toFixed(1)} ms。机身位置固定、偏航未知。`;
      $('footer-state').textContent=`${liveData.target} · STATE 只读 · ${f.online_ids.length}/15 舵机在线`;drawChart();
    }
    if(!liveData.active){$('live-torque').textContent='未知';$('footer-state').textContent='实时反馈已断开 · 画面保留最后姿态';}
  }catch(error){if(mode==='live'){message('反馈连接中断：'+error.message,true);$('run-status').textContent='连接中断';$('run-status').classList.add('stale');$('live-torque').textContent='未知';}}
  finally{livePolling=false;if(mode==='live')setTimeout(pollLive,33);}
}

function bind(){
  document.querySelectorAll('[data-mode]').forEach(b=>b.addEventListener('click',()=>setMode(b.dataset.mode)));
  document.querySelectorAll('[data-view]').forEach(b=>b.addEventListener('click',()=>setView(b.dataset.view)));
  $('frame-view').addEventListener('click',()=>setView());
  document.querySelectorAll('[data-group]').forEach(b=>b.addEventListener('click',()=>{
    const next=b.dataset.group,index=groupIndices[next].includes(selected)?selected:groupIndices[next][0];
    if(!selectJoint(index))return;
    group=next;document.querySelectorAll('[data-group]').forEach(a=>a.classList.toggle('active',a.dataset.group===group));rebuildJoints();
  }));
  function speedChange(){const v=Number($('speed').value);$('speed-value').innerHTML=v.toFixed(2)+' <small>m/s</small>';document.querySelectorAll('[data-speed]').forEach(b=>b.classList.toggle('active',Number(b.dataset.speed)===v));}
  $('speed').value=.2;speedChange();
  $('speed').addEventListener('input',speedChange);
  document.querySelectorAll('[data-speed]').forEach(b=>b.addEventListener('click',()=>{$('speed').value=b.dataset.speed;speedChange();}));
  $('generate').addEventListener('click',()=>generate());
  $('live-connect').addEventListener('click',async()=>{if(liveBusy)return message('正在连接，请稍候。');liveBusy=true;message('正在连接 X5 只读反馈…');try{await api('/api/live/connect',{});liveSequence=null;liveStarted=performance.now();dataset={kind:'live',frames:[],duration:10,markers:[]};message();}catch(e){message(e.message,true);}finally{liveBusy=false;}});
  $('live-disconnect').addEventListener('click',async()=>{try{await api('/api/live/disconnect',{});message('实时反馈已断开，串口已释放。');}catch(e){message(e.message,true);}});
  $('play').addEventListener('click',()=>{if(['live','calibration'].includes(mode))return message('当前持续显示反馈，请使用右侧采集按钮。');if(mode==='joints')return message('关节调试使用右侧滑块，无时间序列。');if(!dataset)return message('请先生成仿真或选择一条记录。');if(time>=dataset.duration)seek(0);setPlaying(!playing);});
  $('reset').addEventListener('click',()=>{setPlaying(false);if(['live','calibration'].includes(mode)){setView();return;}if(mode==='joints')$('home-pose').click();else seek(0);});
  $('timeline').addEventListener('input',()=>{setPlaying(false);seek(Number($('timeline').value));});
  $('record').addEventListener('change',loadRecord);
  $('jump-forward').addEventListener('click',()=>{const marker=dataset?.markers.find(m=>m.label==='前进指令');if(!marker)return message('这条记录没有前进指令。');message();setPlaying(false);seek(marker.t);});
  $('home-pose').addEventListener('click',()=>{if(jointFollow?.active||jointFollow?.busy)return;manual.q=[...meta.home];rebuildJoints();displayFrame(manual);message();});
  $('joint-follow-inspect').addEventListener('click',()=>{$('joint-follow-details').showModal();});
  $('joint-follow-close').addEventListener('click',()=>{$('joint-follow-details').close();});
  $('ghost').addEventListener('change',()=>{if(current)displayFrame(current);});
  $('grid').addEventListener('change',()=>grid.visible=$('grid').checked);
  document.addEventListener('keydown',e=>{if(e.code==='Space'&&!['INPUT','SELECT','BUTTON'].includes(document.activeElement.tagName)){e.preventDefault();$('play').click();}});
}

const raycaster=new THREE.Raycaster(),pointer=new THREE.Vector2();let down;
renderer.domElement.addEventListener('pointerdown',e=>{down=[e.clientX,e.clientY];});
renderer.domElement.addEventListener('pointerup',e=>{
  if(!robot||!down||Math.hypot(e.clientX-down[0],e.clientY-down[1])>4)return;
  if(['calibration','live','joints'].includes(mode)){
    message('请点图上的 ID 标签或关节列表；连杆网格包含相邻舵机外壳，不能用整块外壳判断编号。');return;
  }
  const rect=renderer.domElement.getBoundingClientRect();pointer.set((e.clientX-rect.left)/rect.width*2-1,-(e.clientY-rect.top)/rect.height*2+1);
  raycaster.setFromCamera(pointer,camera);const hit=raycaster.intersectObjects(robot.meshes)[0];const i=hit?.object.userData.joint;
  if(i===undefined||!selectJoint(i))return;group=i<5?'left':i<9||i===14?'head':'right';document.querySelectorAll('[data-group]').forEach(b=>b.classList.toggle('active',b.dataset.group===group));rebuildJoints();
});
new ResizeObserver(()=>{const box=$('viewport');camera.aspect=box.clientWidth/box.clientHeight;camera.updateProjectionMatrix();renderer.setSize(box.clientWidth,box.clientHeight);if(mode==='joints'&&robot)setView();drawChart();}).observe($('viewport'));
let last=performance.now();
function animate(now){const dt=Math.min((now-last)/1000,.1);last=now;
  if(mode==='calibration'&&robot){
    if(demoKind){
      const q=demoKind==='positive'?[...current.q]:Array(meta.motor_ids.length).fill(0);if(demoKind==='positive')q[demoIndex]=(Number.isFinite(q[demoIndex])?q[demoIndex]:0)+6*(1-Math.cos(now/900));
      pose(ghost,{q,base:current?.base||[0,0,.22],quat:[1,0,0,0]});ghost.root.visible=true;
    }
    if(calibration.received&&now-calibration.received>600){calibration.render();}
  }
  if(mode==='live'&&liveData?.fresh&&now-liveLastReply>600){
    liveData.fresh=false;$('run-status').textContent='反馈过期 · 已冻结';$('run-status').classList.add('stale');
    $('live-torque').textContent='未知';$('connection-badge').textContent='○ 实机反馈已过期';
    message('未收到新的反馈，画面保留最后姿态。请检查网络或重新连接。',true);
  }
  if(playing&&dataset){seek(time+dt*Number($('rate').value));if(time>=dataset.duration){setPlaying(false);$('run-status').textContent='播放结束';}}
  controls.update();updateJointTags();renderer.render(scene,camera);requestAnimationFrame(animate);
}
requestAnimationFrame(animate);setView();

try{
  [meta,replays]=await Promise.all([api('/api/model'),api('/api/replays')]);
  const loader=new STLLoader();
  const entries=[],assets=Object.entries(meta.assets);
  // Leave gateway capacity for other viewers instead of requesting every
  // mesh at once. This only loads static geometry, never hardware feedback.
  for(let offset=0;offset<assets.length;offset+=2){
    entries.push(...await Promise.all(assets.slice(offset,offset+2).map(async([name,spec])=>[name,await loader.loadAsync('/assets/'+spec.file)])));
  }
  const geometry=Object.fromEntries(entries);robot=assemble(geometry);ghost=assemble(geometry,true);ghost.root.visible=false;
  for(const record of replays){const option=document.createElement('option');option.value=record.id;option.textContent=record.title;$('record').append(option);}
  $('model-hash').textContent='SHA256 '+meta.policy_sha256;
  current={q:[...meta.home],target:null,base:[0,0,.139439],quat:[1,0,0,0],tilt:0,command:[0,0,0]};
  calibration=new CalibrationPanel({meta,
    onSelect:index=>{if(index<0)return;group=index<5?'left':index<9||index===14?'head':'right';selectJoint(index);document.querySelectorAll('[data-group]').forEach(b=>b.classList.toggle('active',b.dataset.group===group));rebuildJoints();},
    disconnectViewer:()=>api('/api/live/disconnect',{}),
    onDemo:(kind,index)=>{demoKind=kind;demoIndex=index;$('demo-caption').hidden=!kind;$('demo-caption').textContent=kind==='zero'?'紫色：模型零姿态参考 · 实机不动作':'紫色：所选关节向正方向 +12° 示范 · 实机不动作';if(ghost){ghost.root.visible=!!kind;ghost.meshes.forEach(m=>m.material.opacity=m.userData.joint===index?.7:.13);}},
    onStatus:(text,fresh,detail)=>{if(mode==='calibration'){
      $('run-status').textContent=text;$('run-status').classList.toggle('stale',!fresh);$('connection-badge').textContent=fresh?'● 只读校准已连接':'○ 校准反馈未就绪';
      $('view-subtitle').textContent=fresh&&detail?`实机同步 · ID${detail.id} ${detail.raw} 刻度 / ${detail.angle.toFixed(2)}° · 已更新 ${detail.appliedFrames} 帧`:`未同步实机 · ${text}；画面保留参考或上次姿态`;
    }},
    onFrame:f=>{if(mode!=='calibration')return;const t=(performance.now()-liveStarted)/1000,frame={kind:'live',q:f.q,target:null,base:[0,0,.22],quat:[1,0,0,0],tilt:0,command:[0,0,0],t};displayFrame(frame);$('tilt').textContent='—';time=t;dataset.frames.push(frame);dataset.frames=dataset.frames.filter(p=>p.t>=t-10);$('current-time').textContent=t.toFixed(2);drawChart();}
  });
  jointFollow=new JointFollowPanel({ids:meta.motor_ids,labels:Object.fromEntries(meta.motor_ids.map((id,i)=>[id,meta.labels[i]])),disconnectViewer:()=>api('/api/live/disconnect',{}),onChange:syncFollowControls,
    onFrame:f=>{if(mode!=='joints')return;const first=jointViewGeneration!==f.hardware.generation;jointViewGeneration=f.hardware.generation;
      manual={q:f.q,target:f.target,base:[0,0,.22],quat:[1,0,0,0],tilt:0,command:[0,0,0]};displayFrame(manual);if(first)setView();
      $('run-status').textContent='真机随动';$('evidence-note').textContent='实心为实测关节，紫色为请求目标；机身位置固定，不表示实际承重或足底接触。';}
  });
  lastBase.fromArray(current.base);displayFrame(current);bind();rebuildJoints();$('loading').hidden=true;setView();
  setMode(location.hash==='#calibration'?'calibration':location.hash==='#joints'?'joints':'live');
  // A shareable joint link changes the visual selection only. It never starts
  // feedback, records a sample, saves calibration or sends a motor command.
  const focusId=Number(new URLSearchParams(location.search).get('joint'));
  if(mode==='calibration'&&meta.motor_ids.includes(focusId))calibration.onSelect(meta.motor_ids.indexOf(focusId));
}catch(error){$('loading').querySelector('span').textContent='加载失败：'+error.message;$('loading').querySelector('.spinner').hidden=true;message(error.message,true);console.error(error);}
