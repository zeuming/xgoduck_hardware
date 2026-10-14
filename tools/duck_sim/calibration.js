import {MOTOR_IDS,JOINT_IDS,NEXT_ACTION,ACTION_LABELS,draftId,feedbackFrame,movementSummary,captureIntent} from './calibration_core.js';
const $=id=>document.getElementById(id);
const partHints={
  10:['左腿根部 · 扭转轴','托住躯干，观察整条左腿相对躯干的扭转。'],
  11:['左腿根部 · 侧摆轴','扶住躯干，轻摆左腿。正方向是向鸭子自身左侧，也就是左腿外摆。'],
  12:['左腿根部 · 前后摆轴','扶住躯干，观察左腿相对躯干的前后摆动。'],
  13:['左腿中段 · 大腿与小腿之间','扶住左大腿，只轻摆小腿，观察两段之间的夹角。'],
  14:['舵机在左小腿下端、脚掌上方；图中标记为输出轴','ID14 是脚掌上方的舵机，脚掌是被驱动的部件。一手扶住小腿靠近脚踝处，另一手轻转脚掌，只观察脚掌相对小腿的夹角。'],
  20:['右腿根部 · 扭转轴','托住躯干，观察整条右腿相对躯干的扭转。'],
  21:['右腿根部 · 侧摆轴','扶住躯干，轻摆右腿。正方向是向鸭子自身左侧，也就是右腿内收。'],
  22:['右腿根部 · 前后摆轴','扶住躯干，观察右腿相对躯干的前后摆动。'],
  23:['右腿中段 · 大腿与小腿之间','扶住右大腿，只轻摆小腿，观察两段之间的夹角。'],
  24:['舵机在右小腿下端、脚掌上方；图中标记为输出轴','ID24 是脚掌上方的舵机，脚掌是被驱动的部件。一手扶住小腿靠近脚踝处，另一手轻转脚掌，只观察脚掌相对小腿的夹角。'],
  30:['颈部下端 · 靠近躯干','扶住躯干，观察颈部相对躯干的转动，避开线束与板卡。'],
  31:['颈部上端 · 靠近头部','扶住颈部，观察头部相对颈部的俯仰，避开线束与板卡。'],
  32:['头部 · 左右转头','扶住颈部，轻轻转头，对照示范的相对转向。'],
  33:['头部 · 左右歪头','扶住颈部，轻轻歪头，对照示范的相对转向。'],
  34:['嘴部内部舵机 · 下颌两侧同轴铰链','托住头部，轻轻转动下颌。正方向为张嘴；零点参考为轻合嘴，不要越过闭合位置。']};

export class CalibrationPanel{
  constructor({meta,onFrame,onDemo,onStatus,onSelect,disconnectViewer}){
    Object.assign(this,{meta,onFrame,onDemo,onStatus,onSelect,disconnectViewer});
    this.index=3;this.snapshot=null;this.received=0;this.active=false;this.busy=false;this.polling=false;this.epoch=0;
    this.recent=[];this.lastSample=null;this.record=null;this.demo=null;this.appliedFrames=0;this.pollError='';
    $('cal-connect').onclick=()=>this.connect();$('cal-disconnect').onclick=()=>this.end();
    $('cal-demo-positive').onclick=()=>this.showDemo('positive');$('cal-demo-zero').onclick=()=>this.showDemo('zero');$('cal-demo-stop').onclick=()=>this.showDemo(null);
    $('cal-task').onchange=()=>{if(draftId(this.snapshot)!==null||this.record?.active){$('cal-task').value=this.task||'observe';this.error('先完成或取消当前采样，再切换任务');return;}this.task=$('cal-task').value;$('cal-confirm').checked=false;this.render();};
    $('cal-next').onclick=()=>this.capture();$('cal-cancel').onclick=()=>this.capture('cancel');
    $('cal-save-direction').onclick=()=>this.capture('save-direction');$('cal-retry-a').onclick=()=>this.capture('range-a');
    $('cal-record-start').onclick=()=>this.startRecord();$('cal-record-stop').onclick=()=>this.stopRecord();$('cal-export').onclick=()=>this.exportRecord();
    this.task='observe';this.select(3);
  }
  get id(){return this.meta.motor_ids[this.index];}
  label(id){return this.meta.labels[JOINT_IDS.indexOf(id)]||'下颌';}
  error(text=''){$('cal-error').textContent=text;}
  select(index){
    const id=this.meta.motor_ids[index],locked=draftId(this.snapshot);
    if(this.selectionReady&&index===this.index)return true;
    if(this.active&&((locked!==null&&locked!==id)||this.record?.active||this.busy)){this.error('先结束本次记录，或保存 / 取消当前关节采样，再选择其他关节');return false;}
    this.index=index;this.recent=[];$('cal-confirm').checked=false;this.showDemo(null);
    this.selectionReady=true;
    $('cal-name').textContent=`${this.meta.labels[index]} · ID${id}`;
    $('cal-location').textContent=partHints[id][0];$('cal-hand-guide').textContent=partHints[id][1];this.render();return true;
  }
  enter(){this.active=true;this.poll();this.render();}
  leave(){if(this.record?.active)this.stopRecord('离开校准页，记录已结束');this.active=false;this.showDemo(null);}
  showDemo(kind){this.demo=kind;this.onDemo(kind,this.index);}
  async request(path,body){
    if(!['/api/live-status','/api/recover-feedback','/api/recheck-feedback','/api/check-connection','/api/end-feedback','/api/joint-calibration','/api/joint-zero'].includes(path))throw Error('未知校准接口');
    const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),body?8000:2000);
    try{
      const response=await fetch(path,{credentials:'same-origin',cache:'no-store',signal:abort.signal,
        ...(body?{method:'POST',headers:{'Content-Type':'application/json','X-CSRF-Token':this.snapshot?.token||''},body:JSON.stringify(body)}:{})});
      const result=await response.json();if(!response.ok||result.error)throw Error(result.error||`HTTP ${response.status}`);
      if(!result.native_access?.full_workbench)throw Error('请从 X5 实机工作台打开「仿真工作台 → 三维校准」');
      if(body)this.settleUntil=performance.now()+500;
      return result;
    }catch(e){if(e.name==='AbortError')throw Error('请求超时，结果尚未确认。请看当前步骤状态；不会自动重试或保存。');throw e;}
    finally{clearTimeout(timer);}
  }
  accept(s){
    if(this.pollError)this.error();
    this.pollError='';
    const old=this.snapshot;
    if(old&&s.session.generation!==old.session.generation){$('cal-support').checked=false;$('cal-confirm').checked=false;this.recent=[];if(this.record?.active)this.stopRecord('采集会话变化，本次记录不完整');}
    if(old&&JSON.stringify(old.calibration)!==JSON.stringify(s.calibration)){this.recent=[];if(this.record?.active)this.stopRecord('校准发生变化，本次记录不完整');}
    this.snapshot=s;this.received=performance.now();
    const locked=draftId(s);
    if(locked!==null){
      this.task=s.joint_zero?.id===locked&&['sampling','ready'].includes(s.joint_zero.stage)?'zero':'direction';
      $('cal-task').value=this.task;
      if(!this.busy&&locked!==this.id)this.onSelect(JOINT_IDS.indexOf(locked));
    }
    const frame=feedbackFrame(s);
    if(frame){
      if(frame.sample!==this.lastSample){
        this.lastSample=frame.sample;this.lastAdvance=performance.now();
        const row={...frame,t:performance.now()};this.recent.push(row);this.recent=this.recent.filter(r=>row.t-r.t<10000);
        if(this.record?.active){this.record.samples.push({sample_us:frame.sample,elapsed_s:(row.t-this.record.started)/1000,raw:frame.raw});}
        this.onFrame(frame,s.body_imu);
        this.appliedFrames++;
      }
      if(this.record?.active&&performance.now()-this.lastAdvance>1000)this.stopRecord('采样时钟停止推进，本次记录不完整');
      this.lastValid=performance.now();
    }else if(this.record?.active&&performance.now()-(this.lastValid||0)>1000)this.stopRecord('反馈中断或扭矩状态变化，本次记录不完整');
    if(this.record?.active&&performance.now()-this.record.started>=120000)this.stopRecord('达到 120 秒，记录已结束');
    this.render();
  }
  async poll(){
    if(!this.active||this.polling)return;
    if(performance.now()<(this.settleUntil||0)){setTimeout(()=>this.poll(),250);return;}
    this.polling=true;const epoch=this.epoch;
    try{const s=await this.request('/api/live-status');if(this.active&&!this.busy&&epoch===this.epoch)this.accept(s);}
    catch(e){if(this.active){this.pollError=e.message;this.error(e.message);this.onStatus('读取失败：'+e.message,false);if(this.record?.active)this.stopRecord('读取失败，本次记录不完整');}}
    finally{this.polling=false;if(this.active)setTimeout(()=>this.poll(),250);}
  }
  async operation(fn){
    if(this.busy)return this.error('正在处理上一步，请稍候');
    if(performance.now()<(this.settleUntil||0))return this.error('上一步已收到回复，正在等待状态更新；请稍候再点');
    this.busy=true;this.epoch++;this.error();
    try{await fn();}catch(e){this.error(e.message);}
    finally{this.epoch++;if(performance.now()>=(this.settleUntil||0)){try{this.accept(await this.request('/api/live-status'));}catch(e){this.error(e.message);}}this.busy=false;this.render();}
  }
  async connect(){return this.operation(async()=>{
    await this.disconnectViewer();this.accept(await this.request('/api/live-status'));
    const s=this.snapshot;
    if(s.motion?.owns_connection)throw Error('运控正在占用连接。请先托稳机身，在运控页停止并断开，再连接校准');
    if(s.session.status==='active'&&s.session.collection_scope==='joints')return;
    if(s.session.status==='active')throw Error('当前为 IMU 采集，请先结束采集');
    const path=s.session.connection_check?.allowed?'/api/check-connection':s.session.recheck_allowed?'/api/recheck-feedback':'/api/recover-feedback';
    this.accept(await this.request(path,{generation:s.session.generation,purpose:'joints'}));
  });}
  async end(){return this.operation(async()=>{
    if(this.record?.active)this.stopRecord('手动结束采集');
    this.accept(await this.request('/api/live-status'));
    if(draftId(this.snapshot)!==null)throw Error('请先保存或取消未完成采样，再结束采集');
    this.accept(await this.request('/api/end-feedback',{generation:this.snapshot.session.generation}));
  });}
  async capture(explicit){return this.operation(async()=>{
    const generation=this.snapshot?.session?.generation;
    this.accept(await this.request('/api/live-status'));
    if(this.snapshot.session.generation!==generation)throw Error('采集会话已变化，请重新确认支撑并查看当前步骤');
    const kind=this.task==='zero'?'zero':'direction',w=kind==='zero'?this.snapshot.joint_zero:this.snapshot.joint_wizard;
    const action=explicit||(kind==='zero'?(w?.id===this.id&&w.stage==='ready'?'save':'capture'):(w?.id===this.id?NEXT_ACTION[w.stage]:'direction-start'));
    const intent=captureIntent({snapshot:this.snapshot,id:this.id,action,kind,supported:$('cal-support').checked,confirmed:$('cal-confirm').checked});
    this.accept(await this.request(intent.path,intent.body));
    if(['direction-start','cancel','save','save-direction'].includes(action)){$('cal-confirm').checked=false;this.recent=[];}
    if(action==='direction-start')this.showDemo('positive');
  });}
  startRecord(){
    if(this.record?.active)return this.error('已在记录，摆好后点「结束记录」');
    if(!$('cal-support').checked)return this.error('请先确认机身已托稳');
    const f=feedbackFrame(this.snapshot,performance.now()-this.received);
    if(!f)return this.error('请先连接只读校准，等待全部在线且扭矩关闭');
    this.record={schema:1,joint_id:this.id,joint_name:this.label(this.id),started:performance.now(),started_at:new Date().toISOString(),active:true,
      calibration:structuredClone(this.snapshot.calibration),generation:f.generation,samples:[],motor_commands:0,calibration_writes:0};
    this.recent=[];this.error();this.render();
  }
  stopRecord(reason=null){
    if(!this.record?.active){if(!reason)this.error('当前没有正在进行的记录');return;}
    this.record.active=false;this.record.reason=reason||'用户结束记录';this.record.complete=!reason||['达到 120 秒，记录已结束','手动结束采集'].includes(reason);
    this.record.summary=movementSummary(this.record.samples);this.record.duration_s=(performance.now()-this.record.started)/1000;
    this.record.complete=this.record.complete&&this.record.samples.length>=5;this.render();
  }
  exportRecord(){
    if(!this.record||this.record.active)return this.error('请先结束一段记录，再导出');
    const output={...this.record,warning:'人工观察的极值不等于已核验的机械安全边界；未修改正式校准。'};delete output.started;
    const url=URL.createObjectURL(new Blob([JSON.stringify(output,null,2)],{type:'application/json'}));const a=document.createElement('a');a.href=url;a.download=`duck-joint-${this.record.joint_id}-${Date.now()}.json`;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
  }
  render(){
    const s=this.snapshot,f=this.pollError?null:feedbackFrame(s,performance.now()-this.received),j=s?.calibration?.joints.find(j=>j.id===this.id);
    $('cal-connection').textContent=f?'15 / 15 在线 · 扭矩关闭':s?.session?.message||'等待新鲜只读反馈';
    $('cal-reading').textContent=f?`${f.raw[MOTOR_IDS.indexOf(this.id)]} 刻度 · ${f.q[this.index].toFixed(2)}°`:'读数未就绪';
    if(this.active)this.onStatus(f?'只读校准 · 扭矩关闭':this.pollError?'读取失败：'+this.pollError:s?.session?.status==='active'?'反馈未就绪或已过期':'未连接只读校准',!!f,
      {id:this.id,raw:f?.raw[MOTOR_IDS.indexOf(this.id)],angle:f?.q[this.index],appliedFrames:this.appliedFrames});
    const summary=movementSummary(this.recent),moving=summary.filter(r=>r.span_ticks>=12).slice(0,3);
    $('cal-moving').textContent=moving.length?'最近变化：'+moving.map(r=>`${this.label(r.id)} ID${r.id} ${r.span_deg.toFixed(1)}°`).join(' · '):'最近 10 秒：尚无明显关节变化';
    const selected=summary.find(r=>r.id===this.id);
    if(moving.length&&selected?.span_ticks<12)$('cal-moving').textContent+='；所选关节尚未明显转动，请按亮色关节轴核对';
    $('cal-observe-panel').hidden=this.task!=='observe';$('cal-formal-panel').hidden=this.task==='observe';
    const rec=this.record;
    $('cal-record-status').textContent=rec?.active?`正在记录 · ${Math.floor((performance.now()-rec.started)/1000)} 秒 · ${rec.samples.length} 帧`:
      rec?`${rec.complete?'记录结束':'记录不完整'} · ${rec.samples.length} 帧；所选关节摆幅 ${(rec.summary?.find(r=>r.id===rec.joint_id)?.span_deg||0).toFixed(2)}°。${rec.reason}`:'未开始记录 · 等你点开始';
    const zero=this.task==='zero',w=zero?s?.joint_zero:s?.joint_wizard,own=w?.id===this.id;
    $('cal-step').textContent=own?(w.message||w.stage):zero?'先看零姿态参考，把所选关节的相对姿态摆稳，再采样预览。':'1 · 保持当前关节静止，记录起点；这一步不会重设零点。';
    const action=zero?(own&&w.stage==='ready'?'save':'capture'):(own?NEXT_ACTION[w.stage]:'direction-start');
    $('cal-next').textContent=this.busy||w?.busy?'采样 / 处理进行中…':zero?(action==='save'?'保存新零点':'采样零点 · 先预览'):ACTION_LABELS[action]||'等待本步完成';
    $('cal-confirm-label').textContent=zero?'所选关节已按紫色零姿态参考摆稳':'刚才实际轻摆方向与紫色正方向示范一致';
    $('cal-save-direction').hidden=zero||!own||!w.direction_reuse_allowed;
    $('cal-retry-a').hidden=zero||!own||!w.range_retry_allowed;
    let preview=j?`已保存：零点 ${j.zero} · 方向 ${j.sign>0?'+1':'−1'} · 范围 ${j.min??'未采集'}～${j.max??'未采集'} 刻度`:'等待正式校准';
    if(own){
      if(zero&&w.candidate_zero!=null)preview+=`\n待保存：零点 ${w.old_zero} → ${w.candidate_zero}。保存后此关节的方向和范围需重新采集。`;
      if(!zero)preview+=`\n本次：起点 ${w.start_position??'—'} · 正向 ${w.end_position??'—'}\n安全 A ${w.range_a??'—'} · B ${w.range_b??'—'}`;
      if(!zero&&w.inferred_sign!=null)preview+=`\n待保存方向 ${w.inferred_sign>0?'+1':'−1'}`;
      if(!zero&&w.ready_to_save){
        if(this.id===34){const opened=Math.abs(w.range_a-j.zero)>Math.abs(w.range_b-j.zero)?w.range_a:w.range_b;const end=opened-w.inferred_sign*12;preview+=` · 待保存范围 ${Math.min(j.zero,end)}～${Math.max(j.zero,end)}（闭合端保留零点）`;}
        else preview+=` · 缩边范围 ${Math.min(w.range_a,w.range_b)+12}～${Math.max(w.range_a,w.range_b)-12}`;
      }
      if(w.error)preview+='\n'+w.error;
    }
    $('cal-preview').textContent=preview;
    $('cal-formal-note').textContent=zero?'零姿态按相邻部件的相对姿态判断，不能只看脚底是否贴桌。保存只重设所选关节零点，并清除该关节旧方向和范围。':
      this.id===34?'嘴部 A / B 分别记录轻合与张开。后台保留原闭合零点，仅张开端缩进 12 刻度；不得越过闭合位置。方向需连同范围一起保存。':
      '每一步由你摆稳后点击，稳定采样完成再继续。A / B 保持其他关节参考姿态并留出间隙；后台两端各缩进 12 刻度并校验零点、站姿。';
  }
}
