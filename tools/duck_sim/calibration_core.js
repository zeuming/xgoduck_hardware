// Pure display/intent logic. All sampling and calibration commits remain in
// the existing X5 calibration service; this module has no motor commands.
export const MOTOR_IDS=[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34];
export const JOINT_IDS=[10,11,12,13,14,30,31,32,33,20,21,22,23,24,34];
export const NEXT_ACTION={idle:'direction-start',failed:'direction-start',cancelled:'direction-start',complete:'direction-start',
  'await-positive':'direction-end','await-range-a':'range-a','await-range-b':'range-b',ready:'save'};
export const ACTION_LABELS={'direction-start':'1 · 记录起点','direction-end':'2 · 记录正方向位置',
  'range-a':'3 · 记录安全位置 A','range-b':'4 · 记录安全位置 B',save:'5 · 保存本关节'};
export function draftId(s){
  const w=s?.joint_wizard,z=s?.joint_zero;
  if(w?.id!=null&&!['idle','complete','failed','cancelled'].includes(w.stage))return w.id;
  if(z?.id!=null&&['sampling','ready'].includes(z.stage))return z.id;
  return null;
}
export function feedbackFrame(s,elapsed=0){
  const f=s?.feedback,session=s?.session,cfg=s?.calibration?.joints;
  const age=session?.received_age_ms;
  if(session?.status!=='active'||session.collection_scope!=='joints'||!f||f.joint_feedback_valid!==true||
    !Number.isFinite(age)||!Number.isFinite(elapsed)||age<0||elapsed<0||age+elapsed>=500)return null;
  if(f.armed!==false||f.torque_mask!==0||f.fault_mask!==0||f.online_mask!==32767)return null;
  if(!Array.isArray(f.positions)||f.positions.length!==15||f.positions.some(p=>!Number.isInteger(p)||p<0||p>4095))return null;
  if(!Array.isArray(cfg)||cfg.length!==15||cfg.some((j,i)=>j.id!==MOTOR_IDS[i]||!Number.isInteger(j.zero)||j.zero<0||j.zero>4095||![1,-1].includes(j.sign)))return null;
  if(!Number.isInteger(f.sample_us)||f.sample_us<0||f.sample_us>0xffffffff)return null;
  const angles=cfg.map((j,i)=>(f.positions[i]-j.zero)*360/4095*j.sign);
  return {raw:[...f.positions],q:JOINT_IDS.map(id=>angles[MOTOR_IDS.indexOf(id)]),sample:f.sample_us,
    generation:session.generation,calibration:JSON.stringify(s.calibration),age_ms:age+elapsed};
}
export function movementSummary(rows){
  if(!rows.length)return [];
  return MOTOR_IDS.map((id,i)=>{
    const values=rows.map(r=>r.raw[i]);const min=Math.min(...values),max=Math.max(...values);
    return {id,min,max,span_ticks:max-min,span_deg:(max-min)*360/4095};
  }).sort((a,b)=>b.span_ticks-a.span_ticks);
}
export function captureIntent({snapshot:s,id,action,kind='direction',supported=false,confirmed=false,elapsed=0}){
  if(!JOINT_IDS.includes(id))throw Error('请选择模型中的一个关节');
  if(!s?.native_access?.full_workbench||!Number.isInteger(s?.session?.generation))throw Error('请在 X5 工作台的「仿真工作台」中打开此功能');
  if(!supported&&action!=='cancel')throw Error('请先勾选已托稳机身；记录过程不会开启扭矩');
  const locked=draftId(s);
  if(locked!==null&&locked!==id)throw Error(`ID${locked} 有未完成采样，请先保存或取消，再换关节`);
  if(action!=='cancel'&&!feedbackFrame(s,elapsed))throw Error('需要新鲜的关节反馈、全部在线且扭矩关闭；请先连接只读校准');
  if(action!=='cancel'&&(s.joint_wizard?.busy||s.joint_zero?.busy))throw Error('正在采样，请保持静止，等本步结束');
  if(kind==='zero'){
    if(!['capture','save','cancel'].includes(action))throw Error('未知零点步骤');
    if(action!=='cancel'&&!confirmed)throw Error('请对照紫色零姿态，确认所选关节的相对姿态已摆稳');
    if(action==='save'&&(s.joint_zero?.id!==id||s.joint_zero.stage!=='ready'))throw Error('请先采样并核对零点预览');
    return {path:'/api/joint-zero',body:{generation:s.session.generation,id,action,support_confirmed:supported,zero_reference_confirmed:confirmed}};
  }
  if(action!=='cancel'){
    const w=s.joint_wizard||{},expected=w.id===id?NEXT_ACTION[w.stage]:'direction-start';
    const retry=w.id===id&&w.range_retry_allowed&&['range-a','range-b'].includes(action);
    const reuse=action==='save-direction'&&w.id===id&&w.direction_reuse_allowed===true;
    if(action!==expected&&!retry&&!reuse)throw Error('请按页面当前步骤采集，不能跳过未完成步骤');
    if(['direction-end','save','save-direction'].includes(action)&&!confirmed)throw Error('请确认刚才的实际轻摆方向与紫色正方向示范一致');
  }
  return {path:'/api/joint-calibration',body:{generation:s.session.generation,id,action,support_confirmed:supported,positive_confirmed:confirmed}};
}
