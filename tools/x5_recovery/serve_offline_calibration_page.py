#!/usr/bin/env python3
"""Serve a local, immutable calibration preview when the X5 link is offline.

Only loopback HTTP is opened. Inputs are read once; this server has no SSH,
UART, model runtime, hardware controller or filesystem write operations.
"""
import argparse
import copy
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import re
import stat

import calibration_restore

ASSETS = {'/zero-reference.png': ('zero-reference.png', '5c0c06ec88010a4d1b77755478a04d78b077928d4ea3cf9f5e09048bd4ab994c'),
          '/positive-direction.png': ('positive-direction.png', '253d06d0306292319688e006af90853b6a513bdc6407db873f1cc205cad3f915')}
OFFLINE_SCRIPT = r'''<script>
const offlinePreviewText='X5 已停用；本页是本机只读预览。模型推理和所有采集均已关闭。';
const offlinePreviewRenderBefore=renderMotion;
renderMotion=function(){offlinePreviewRenderBefore();
  state.online=false;state.token=null;motion.owned=false;
  $('imuMountedConfirmed').checked=false;
  $('serviceDot').className='status-dot offline';$('serviceLabel').textContent='本机只读预览 · X5 已离线';
  $('deviceInfo').textContent='已保存校准可查看；设备故障排除后再恢复真机测试';
  $('hardwareConnection').textContent='无实时连接 · 本页只读';
  $('motionMode').textContent='真机服务已停止 · 当前释放状态无实时反馈';
  $('hardwareDetail').textContent=motion.status?.error||'当前显示笔记本上的已保存校准';
  $('motionReadiness').textContent=offlinePreviewText;
  $('motionGate').textContent=offlinePreviewText;
  $('imuWizardGate').textContent='可以查看下面的操作图；当前无实时 IMU 数据，不能开始采样。';
  $('motionHeartbeat').textContent='本机预览不发送硬件指令';
  for(const element of document.querySelectorAll('button,input,select'))if(!['motionTab','syntheticTab'].includes(element.id))element.disabled=true;
  $('emergencyStop').disabled=true;$('emergencyStop').textContent='真机服务已停 · 本页只读';
};
hardwareFresh=function(){return false;};imuWizardCan=function(){return false;};
markOffline=function(){state.online=false;state.token=null;updateControls();renderMotion();};
stopMotion=async function(){motion.owned=false;renderMotion();};
async function refreshOfflinePreview(){try{
  const [status,hardware]=await Promise.all([fetch('/api/status',{cache:'no-store'}).then(r=>r.json()),fetch('/api/motion/status',{cache:'no-store'}).then(r=>r.json())]);
  state.models=status.models;state.online=false;state.token=null;receiveMotion(hardware);updateControls();renderMotion();
}catch(error){markOffline();}}
loadMotionStatus=refreshOfflinePreview;
refreshOfflinePreview();renderMotion();setInterval(refreshOfflinePreview,5000);
</script>'''


def read_input(path, maximum, private=True):
    path=Path(path)
    if not path.is_absolute() or path.is_symlink():
        raise ValueError('Inputs must be absolute non-symlink regular files')
    fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW)
    try:
        before=os.fstat(fd)
        if not stat.S_ISREG(before.st_mode) or before.st_nlink!=1 or not 0<before.st_size<=maximum:
            raise ValueError('Input is not a bounded regular file')
        if private and (before.st_uid!=os.getuid() or stat.S_IMODE(before.st_mode)!=0o600):
            raise ValueError('HTML, calibration and stop report must be user-owned private 0600 files')
        data=os.read(fd,maximum+1);after=os.fstat(fd)
        keys=('st_dev','st_ino','st_size','st_mtime_ns','st_ctime_ns')
        if len(data)!=before.st_size or any(getattr(before,key)!=getattr(after,key) for key in keys):
            raise ValueError('Input changed during read')
        return data
    finally:os.close(fd)


def joint_ready(joint):
    values=[joint.get(key) for key in ('min','zero','max')]
    return (all(type(value) is int and 0<=value<=4095 for value in values)
            and values[0]<values[2] and values[0]<=values[1]<=values[2]
            and joint.get('direction_confirmed') is True and joint.get('sign') in (-1,1))


def offline_snapshot(calibration,stop_report,reason):
    calibration_restore.validate_calibration_snapshot(calibration)
    joints=[dict(copy.deepcopy(joint),ready=joint_ready(joint)) for joint in calibration['joints']]
    imu=copy.deepcopy(calibration['imu'])
    imu.update(calibrating=False,progress=0.,acc_body=None,gyro_body=None,gravity=None,pitch_deg=None,roll_deg=None)
    history=stop_report.get('serial_shutdown_proof')
    if history is not None and not isinstance(history,dict):raise ValueError('Historical stop proof must be an object')
    history=copy.deepcopy(history)
    return {'connected':False,'port':None,'protocol':1,'mode':'off','generation':0,
            'calibrated':all(j['ready'] for j in joints) and imu['calibrated'] and imu['axes_confirmed'] and calibration['mechanical_reference_confirmed'],
            'mechanical_reference_confirmed':calibration['mechanical_reference_confirmed'],'hold_ready':False,
            'motor_output':False,'armed':False,'configured':False,'online_ids':[],'fault_ids':[],'torque_ids':[],
            'age_ms':None,'imu_age_ms':None,'rawdata':[],'joints':joints,'imu':imu,
            'command':{'forward':0.,'side':0.,'yaw':0.},'last_action':None,'inference_ms':None,
            'error':reason,'pending_stop':False,'firmware_error':None,'imu_feedback_ok':False,
            'rates':{'state_hz':0.,'control_hz':0.},'heartbeat_remaining_ms':0.,
            'paused':True,'read_only':True,'pause_reason':reason,'pause_feedback':None,
            'pause_release_status':'unknown','pause_release_confirmed':False,
            'pause_worker_stopped':False,'pause_serial_closed':False,
            'offline_preview':True,'live_feedback_available':False,'data_source':'saved_calibration_snapshot',
            'historical_stop_reason':stop_report.get('reason'),'historical_stop_proof':history,
            'historical_stop_proof_is_live_feedback':False,
            'imu_wizard':{'stage':'idle','expected_action':'level','busy':False,'progress':0.,
                          'message':'查看操作图；设备故障排除后再恢复采样','error':None,'generation':0,
                          'sample_count':0,'required_samples':100,'pose':'尚无实时反馈'}}


def build_preview(args):
    source=read_input(args.new_html,256*1024)
    if (not re.fullmatch('[0-9a-f]{64}',args.expected_html_sha256)
            or hashlib.sha256(source).hexdigest()!=args.expected_html_sha256):
        raise ValueError('Frozen HTML SHA256 does not match')
    html=source.decode('utf-8','strict')
    if html.count('</body>')!=1 or 'id="imuWizardPanel"' not in html:
        raise ValueError('Frozen page is not the reviewed IMU wizard HTML')
    html=html.replace('</body>',OFFLINE_SCRIPT+'</body>')
    calibration=json.loads(read_input(args.calibration,16384),object_pairs_hook=calibration_restore._restore_json_pairs)
    stop_report=json.loads(read_input(args.stop_report,65536),object_pairs_hook=calibration_restore._restore_json_pairs)
    if not isinstance(stop_report,dict):raise ValueError('Historical stop report must be an object')
    if not isinstance(args.reason,str) or not 1<=len(args.reason)<=500:raise ValueError('Offline reason must be a short description')
    hardware=offline_snapshot(calibration,stop_report,args.reason)
    status={'online':False,'token':None,'motor_output':False,'offline_preview':True,'read_only':True,
            'models':[{'id':name,'ready':False,'error':'X5 已停用，本机只读预览不运行模型'} for name in ('walk','getup','pick','roulade')],
            'device':{'hostname':'本机只读预览'},'benchmark':{'models':{}},'error':args.reason}
    contents={'/':(html.encode('utf-8'),'text/html; charset=utf-8'),
              '/api/status':(json.dumps(status,ensure_ascii=False,allow_nan=False).encode(),'application/json; charset=utf-8'),
              '/api/motion/status':(json.dumps(hardware,ensure_ascii=False,allow_nan=False).encode(),'application/json; charset=utf-8')}
    assets=Path(__file__).resolve().parents[1]/'x5_policy'/'assets'
    for url,(name,digest) in ASSETS.items():
        data=read_input(assets/name,8*1024*1024,private=False)
        if hashlib.sha256(data).hexdigest()!=digest:raise ValueError('Pinned reference asset SHA256 changed')
        contents[url]=(data,'image/png')
    return contents


def make_handler(contents):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def reply(self,code,data,content_type='application/json; charset=utf-8'):
            if not isinstance(data,bytes):data=json.dumps(data,ensure_ascii=False).encode()
            self.send_response(code);self.send_header('Content-Type',content_type)
            self.send_header('Content-Length',str(len(data)));self.send_header('Cache-Control','no-store')
            self.send_header('X-Content-Type-Options','nosniff');self.send_header('X-Frame-Options','DENY')
            self.end_headers();self.wfile.write(data)
        def do_GET(self):
            port=self.server.server_address[1]
            if self.headers.get('Host') not in (f'127.0.0.1:{port}',f'localhost:{port}'):
                return self.reply(403,{'error':'仅允许本机访问'})
            entry=contents.get(self.path)
            if entry is None:return self.reply(404,{'error':'找不到页面'})
            return self.reply(200,*entry)
        def do_POST(self):
            return self.reply(409,{'error':'本机只读预览不接受采集、修改、推理或硬件指令','read_only':True,'motor_output':False})
    return Handler


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--new-html',required=True);parser.add_argument('--expected-html-sha256',required=True)
    parser.add_argument('--calibration',required=True);parser.add_argument('--stop-report',required=True)
    parser.add_argument('--reason',default='X5 已停用；当前显示笔记本上已保存的校准')
    parser.add_argument('--host',choices=['127.0.0.1'],default='127.0.0.1')
    parser.add_argument('--port',type=int,default=8767)
    args=parser.parse_args()
    if not 1024<=args.port<=65535:parser.error('Loopback port must be 1024..65535')
    contents=build_preview(args)
    server=ThreadingHTTPServer((args.host,args.port),make_handler(contents))
    print(json.dumps({'state':'offline_preview','url':f'http://127.0.0.1:{args.port}/','read_only':True,
                      'hardware_access':False,'model_inference':False,'input_html_sha256':args.expected_html_sha256},ensure_ascii=False),flush=True)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:server.server_close()

if __name__=='__main__':main()
