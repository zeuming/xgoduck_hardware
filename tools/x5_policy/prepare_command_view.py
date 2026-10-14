"""Build the thin command page and X5 command lifetime service offline.

Require exact baseline contexts, and preserve every unrelated package member.
"""
from pathlib import Path
import argparse
import hashlib
import json
import shutil


def prepare(source, output):
    assert not output.exists(), 'Use a new output directory; preserve evidence'
    shutil.copytree(source, output, ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    h, here = output, Path(__file__).resolve().parent
    # Always rebuild from exact installed STOP-recovery baseline, never incrementally stack wrappers.
    for name in ('native_process.py','native_workbench.py','native_gamepad.py','gamepad_panel.html'):
     shutil.copyfile(source/'service'/name,h/'service'/name)
    for name in ('native_command_runtime.py','command_view_page.py','command_view_ui.js'):
     shutil.copyfile(here/name,h/'service'/name)
    def once(s,a,b):
     assert s.count(a)==1,(a[:80],s.count(a));return s.replace(a,b,1)
    f=h/'service/native_process.py';s=f.read_text()
    s=once(s,'from native_gamepad import GamepadControl, ControlMonitor','from native_gamepad import GamepadControl, ControlMonitor\nfrom native_command_runtime import CommandRuntime')
    s=once(s,'        self.gamepad = GamepadControl(workbench, monitor=self.control_monitor)','        self.gamepad = GamepadControl(workbench, monitor=self.control_monitor)\n        self.command_runtime = CommandRuntime(workbench, self.control_monitor)')
    s=s.replace("['motion']['gamepad_control'] = self.gamepad.status()","['motion']['gamepad_control'] = self.gamepad.status()")
    s=once(s,"        live['motion']['gamepad_control'] = self.gamepad.status()", "        live['motion']['gamepad_control'] = self.gamepad.status()\n        live['motion']['command_runtime'] = self.command_runtime.snapshot()")
    s=once(s,"            full['motion']['gamepad_control'] = self.gamepad.status()", "            full['motion']['gamepad_control'] = self.gamepad.status()\n            full['motion']['command_runtime'] = self.command_runtime.snapshot()")
    s=once(s,'    def start_commands(self):\n        self.gamepad.start()', '    def start_commands(self):\n        self.command_runtime.start()\n        self.gamepad.start()')
    s=once(s,'        self.stop.set()\n        self.gamepad.close()', '        self.stop.set()\n        self.command_runtime.close()\n        self.gamepad.close()')
    f.write_text(s)
    f=h/'service/native_gamepad.py';s=f.read_text()
    s=once(s,"for name in ('browser', 'x5_local')", "for name in ('browser', 'x5_local', 'x5_runtime')")
    s=once(s,"                        self.wb.control_owner = dict(self.origin, generation=owner['generation'])", "                        # A failed input-loss recovery must not hand a still-running\n                        # action to the independent command runtime. Let the existing\n                        # controller watchdog expire; no service may renew it.\n                        self.wb.control_owner = (None if failure else\n                            dict(self.origin, generation=owner['generation']))")
    f.write_text(s)
    f=h/'service/native_workbench.py';s=f.read_text()
    s=once(s,"    page = page.replace('</script>', SOFT_BUTTON_UI + '\\n' + Path(__file__).with_name('official_runtime_ui.js').read_text() + '\\n</script>', 1)\n    return page.encode()", "    page = page.replace('</script>', SOFT_BUTTON_UI + '\\n' + Path(__file__).with_name('official_runtime_ui.js').read_text() + '\\n</script>', 1)\n    from command_view_page import command_view_page\n    return command_view_page(page.encode())")
    s=once(s,"        if action == 'stop' and body.get('stop_source') not in STOP_SOURCES:\n            raise ValueError('停止来源无效')", "        if action == 'stop' and body.get('stop_source') != 'button':\n            raise ValueError('页面切换或关闭不再停止运控；只有明确的停止命令可执行')")
    s=once(s,"            hardware = before.get('hardware') or {}\n            # Delayed publication", "            hardware = before.get('hardware') or {}\n            closed_stop = action in ('stop', 'disconnect') and not before.get('owns_connection')\n            if closed_stop:\n                receipt_action = False\n            # Delayed publication")
    s=once(s,"                elif action == 'release-torque':", "                elif action == 'release-torque' or closed_stop:")
    s=once(s,"                    result = self.runtime.release_torque()", "                    current = self.runtime.snapshot()\n                    proof = current.get('closed_proof') or {}\n                    result = (current if proof.get('release_confirmed') and proof.get('serial_closed')\n                        else self.runtime.release_torque())")
    f.write_text(s)
    f=h/'service/gamepad_panel.html';s=f.read_text()
    s=once(s,"    if(busy){error.textContent='本次操作尚未完成；未重复发送。';return;}\n", '')
    s=once(s,"    if(!snapshot?.token){error.textContent='页面状态尚未就绪，请等待连接状态更新。';return;}\n", '')
    s=once(s,"    if(action==='gamepad-enable'&&!confirmed){error.textContent='请先勾选防跌准备确认。';return;}\n", '')
    s=once(s,"    if(!Number.isInteger(generation)){error.textContent='请先连接运控并进入固定站姿。';return;}\n", '')
    s=once(s,"'X-CSRF-Token':snapshot.token", "'X-CSRF-Token':snapshot?.token || ''")
    s=s.replace("const local=monitor?.sources?.x5_local,web=monitor?.sources?.browser;", "const local=monitor?.sources?.x5_local,web=monitor?.sources?.browser,runtime=monitor?.sources?.x5_runtime;")
    s=s.replace("'控制来源：'+source+'；本次服务已接受心跳：X5 手柄 '","'控制来源：'+source+'；X5 独立运控心跳 '+(runtime?.heartbeat_accepted||0)+' 次；本次服务已接受心跳：X5 手柄 '")
    s=s.replace("snapshot?.motion?.control_owner?'网页'","snapshot?.motion?.control_owner?'X5 命令运行服务'")
    s=s.replace('这里记录网页和 X5 手柄各自发送的请求。手柄接管后，网页心跳不会续接手柄的控制权；','这里分别记录网页命令、X5 命令运行服务和手柄的请求。网页不发送运控心跳；')
    f.write_text(s)
    for f in h.rglob('*.py'):compile(f.read_bytes(),str(f),'exec')
    names = ('native_process.py', 'native_workbench.py', 'native_gamepad.py',
        'gamepad_panel.html', 'native_command_runtime.py', 'command_view_page.py', 'command_view_ui.js')
    return {'service/'+n:hashlib.sha256((h/'service'/n).read_bytes()).hexdigest() for n in names}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(prepare(args.source, args.output), indent=2))
