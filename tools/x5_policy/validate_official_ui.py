"""Run the actual native-page renderer in a fake DOM; no physical or network I/O."""
import argparse,json,sys,tempfile,subprocess,shutil
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args();r=a.root
sys.path[:0]=[str(r/'candidate/host/service'),str(Path(__file__).resolve().parents[1]/'x5_recovery')]
import native_workbench as w
from test_native_workbench import Runtime,SOURCE
runtime=Runtime();ids=[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34];runtime.calibration=json.loads((r/'saved-calibration.json').read_text());h=runtime.motion['hardware']
h.update(age_ms=0,imu_age_ms=0,imu_ok=True,calibrated=True,policy_metadata_protocol=1,target_validation_protocol=1,behavior_protocol=1,stand_target_deg=[0]*15,hold_ready=False,online_ids=ids,torque_ids=[],fault_ids=[],rates=dict(state_hz=50,control_hz=0),rawdata=[dict(id=s,position=2048) for s in ids],model_start_blocked_reasons={},hold_control_protocol=1,command=dict(forward=0,side=0,yaw=0),official_runtime=dict(protocol=1,recovery=dict(phase='walk')))
page=w.native_page(SOURCE.read_text(),authentication_required=False,full_workbench=True);(r/'ui-page.html').write_bytes(page)
with tempfile.TemporaryDirectory() as d:
 shutil.copyfile(SOURCE,Path(d)/'index.html')
 wb=w.NativeWorkbench(runtime,web_root=d,authentication_required=False);data=wb.snapshot();data['native_access']['full_workbench']=True
 (r/'ui-fixture.json').write_text(json.dumps(data)+'\n')
result=subprocess.run(['node',str(Path(__file__).with_name('test_official_runtime_ui.js')),str(r/'ui-page.html'),str(r/'ui-fixture.json')],capture_output=True,text=True,timeout=20)
print(result.stdout+result.stderr);(r/'ui-check-result.json').write_text(json.dumps(dict(passed=result.returncode==0,output=result.stdout+result.stderr,hardware_access=False),indent=2)+'\n');raise SystemExit(result.returncode)
