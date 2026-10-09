#!/usr/bin/env python3
"""Mac loopback CPU model tests; no hardware, serial connection or simulation."""
import argparse
import json
from pathlib import Path
import signal
import sys
import time


ROOT = Path(__file__).resolve().parents[2]
BUNDLE = ROOT / 'output' / 'x5_policy_bundle'
PORT = 8768

MAC_UI = r'''<style>
#motionTab,#motionPanel,#emergencyStop{display:none!important}
</style><script>
const macModelNote='Mac CPU · 合成观测推理 · 非物理仿真 · 不连接或控制真机';
function lockMacHardware(){
  motion.owned=false;motion.tab=false;
  for(const node of document.querySelectorAll('#motionPanel button,#motionPanel input,#motionPanel select,#motionTab,#emergencyStop'))node.disabled=true;
  $('motionPanel').hidden=true;$('syntheticPanel').hidden=false;
  $('motionTab').onclick=null;$('emergencyStop').onclick=null;
}
const macControlsBefore=updateControls;
updateControls=function(){macControlsBefore();lockMacHardware();
  $('serviceLabel').textContent=state.online?'Mac 本机 CPU 模型服务':'Mac 本机模型服务未连接';
  $('deviceInfo').textContent=macModelNote;
};
renderMotion=lockMacHardware;
selectLabTab=function(){lockMacHardware();};
loadMotionStatus=async function(){};
stopMotion=async function(){lockMacHardware();};
startMotion=async function(){lockMacHardware();};
mutateMotion=async function(){lockMacHardware();return false;};
heartbeatMotion=async function(){};queueMotionCommand=async function(){};
document.querySelector('.subtitle').textContent='在这台 Mac 上调整合成指令与姿态，查看四个预训练模型的输出。';
document.querySelector('.scope-main').textContent=macModelNote;
document.querySelector('.scope-sub').innerHTML='<a href="http://127.0.0.1:8767/" target="_blank" rel="noopener">查看已保存校准与操作图</a>';
lockMacHardware();updateControls();
</script>'''


def load_runtime():
    # An accidental future hardware branch must fail at import, before I/O.
    if sys.modules.get('motion') is not None or sys.modules.get('serial') is not None:
        raise RuntimeError('Hardware modules must not be loaded in the Mac model process')
    sys.modules['motion'] = None
    sys.modules['serial'] = None
    sys.path.insert(0, str(BUNDLE))
    import web_app
    import numpy as np

    class MacWorkbench(web_app.Workbench):
        def __init__(self):
            super().__init__(motion_port=None)
            self.benchmark = None  # The bundle report belongs to the old X5.
            if self.motion is not None or set(self.sessions) != set(web_app.NAMES):
                raise RuntimeError('All four reviewed models must load with hardware disabled')

        def status(self):
            result = super().status()
            result.update(execution_host='Mac', source='synthetic', physical_simulation=False,
                          hardware_disabled=True, motor_output=False)
            result['hardware'] = self.motion_status()
            return result

        def motion_status(self):
            result = super().motion_status()
            result.update(read_only=True, hardware_disabled=True,
                          error='Mac 模型测试不连接真机；已保存校准请打开 8767 页面查看。')
            return result

        def motion_action(self, *args, **kwargs):
            raise ValueError('Mac 模型测试永久禁用所有硬件操作')

        def measure(self):
            results = {}
            for name, session in self.sessions.items():
                _, observation = web_app.synthetic_observation({'model': name})
                for _ in range(50):
                    web_app.infer(session, observation)
                elapsed = []
                for _ in range(1000):
                    start = time.perf_counter_ns()
                    web_app.infer(session, observation)
                    elapsed.append((time.perf_counter_ns() - start) / 1e6)
                results[name] = dict(mean_ms=float(np.mean(elapsed)),
                                     p95_ms=float(np.percentile(elapsed, 95)),
                                     p99_ms=float(np.percentile(elapsed, 99)),
                                     max_ms=max(elapsed), iterations=1000, finite=True,
                                     provider=session.get_providers())
            self.benchmark = dict(test='synthetic_observation_cpu_inference_only',
                                  execution_host='Mac', onnxruntime=web_app.ort.__version__,
                                  models=results)
            return self.benchmark  # RAM only; never overwrite the bundle report.

    return web_app, MacWorkbench()


def make_handler(web_app, workbench):
    html = (BUNDLE / 'index.html').read_text().replace('RDK X5', 'Mac CPU').replace('X5', 'Mac')
    html = html.replace('板端推理性能', 'Mac CPU 推理性能').replace('</body>', MAC_UI + '</body>')
    html = html.encode('utf-8')
    original = web_app.make_handler(workbench)

    class Handler(original):
        def do_GET(self):
            if self.path == '/' and self.valid_host():
                return self.reply(200, html, 'text/html; charset=utf-8')
            return super().do_GET()

        def do_POST(self):
            if not self.valid_host():
                return self.reply(403, {'error': '仅允许本机访问'})
            # A permanent allowlist, independent of future base motion routes.
            if self.path not in ('/api/infer', '/api/benchmark'):
                return self.reply(409 if self.path.startswith('/api/motion') else 404,
                                  {'error': 'Mac 模型测试不接受硬件控制或校准请求',
                                   'hardware_disabled': True, 'motor_output': False})
            return super().do_POST()

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, choices=[PORT], default=PORT)
    args = parser.parse_args()
    web_app, workbench = load_runtime()
    server = web_app.ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(web_app, workbench))
    print(json.dumps({'url': f'http://127.0.0.1:{PORT}/', 'execution_host': 'Mac',
                      'models_ready': list(workbench.sessions), 'hardware_disabled': True,
                      'physical_simulation': False}, ensure_ascii=False), flush=True)
    def stop(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
