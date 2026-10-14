"""Rebuild a minimal dual-model overlay from an inspected native bundle.

No installation, UART, calibration, firmware or controller parameter changes.
The source bundle is copied; strict anchors refuse unrelated source drift.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

HERE = Path(__file__).resolve().parent


def once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('Source context changed: ' + old[:100])
    return text.replace(old, new, 1)


def prepare(source, output, training):
    if output.exists():
        raise ValueError('Use a new candidate directory')
    for folder in ('policy', 'service'):
        shutil.copytree(source/folder, output/folder, ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    changed = []
    def edit(name, transform):
        path = output/name
        path.write_text(transform(path.read_text()))
        changed.append(name)
    shutil.copyfile(HERE/'model_sets.py', output/'service/model_sets.py')
    changed.append('service/model_sets.py')
    edit('policy/infer.py', lambda s: once(once(s,
        'def session(name):\n',
        "def session(name, *, model_directory=None):\n    if name not in NAMES:\n        raise ValueError('Unknown model')\n    directory = HERE / 'models' if model_directory is None else Path(model_directory)\n"),
        "    manifest = json.loads((HERE / 'models/manifest.json').read_text())\n    path = HERE / 'models' / f'xgoduck_{name}.onnx'",
        "    manifest = json.loads((directory / 'manifest.json').read_text())\n    path = directory / f'xgoduck_{name}.onnx'"))
    edit('policy/policy_worker.py', lambda s: once(s,
        "and command[6] == str(root / 'policy/models'))",
        "and command[6] in (str(root / 'policy/models'), str(root / 'policy/model-sets/trained')))"))
    def runtime(s):
        s=once(s,'class NativeRuntime:', 'from model_sets import ModelSetMixin\n\n\nclass NativeRuntime(ModelSetMixin):')
        s=once(s,'    def connect(self):', '    def connect(self, model_set=None):')
        s=once(s,'        with self._lifecycle:\n            with self.lock:\n                reuse =',
            '        with self._lifecycle:\n            self._select_model_set(model_set)\n            with self.lock:\n                reuse =')
        s=once(s,'models_loaded=list(self.sessions),', 'models_loaded=list(self.sessions), **self.model_set_status(),')
        s=once(s,"            return self.connect()\n", "            return self.connect(model_set=body.get('model_set'))\n")
        return s
    edit('service/native_runtime.py', runtime)
    def web(s):
        s=once(s,"stop={'stop_source'}, connect=set(), disconnect=set(),", "stop={'stop_source'}, connect={'model_set'}, disconnect=set(),")
        s=once(s,"        if action == 'start':\n", "        if action == 'connect' and 'model_set' in body and body['model_set'] not in ('original', 'trained'):\n            raise ValueError('未知模型组')\n        if action == 'start':\n")
        s=once(s,'                    result = self.runtime.connect()\n', "                    result = (self.runtime.connect(model_set=body['model_set'])\n                        if 'model_set' in body else self.runtime.connect())\n")
        return s
    edit('service/native_workbench.py', web)
    def page(s):
        # Work with either installed card layout, preserving the current page.
        helper='''def model_set_controls(text):
    pattern = r'<button\\b[^>]*id="motionConnect"[^>]*>.*?</button>'
    matches = list(re.finditer(pattern, text))
    assert len(matches) == 1, 'Model loading button source changed'
    old = matches[0].group()
    original = re.sub(r'(<button\\b[^>]*>).*?(</button>)', r'\\1加载原版模型\\2', old)
    controls = original + '<button class="button" id="motionConnectTrained" type="button">加载自训练模型</button><span id="motionModelSetStatus" class="note" role="status">当前模型组：原版模型</span><span class="note">加载不启动动作；切换另一组前先停止并断开运控。</span>'
    text = text[:matches[0].start()] + controls + text[matches[0].end():]
    return text.replace('<h3>全部官方模型</h3>', '<h3>全部运控模型</h3>')


'''
        s=once(s,'def command_view_page(page):', helper+'def command_view_page(page):')
        s=once(s,'    text = model_actions_page(text)\n', '    text = model_actions_page(text)\n    text = model_set_controls(text)\n')
        return s
    edit('service/command_view_page.py', page)
    def ui(s):
        s=once(s,"['motionConnect','motionShadow'", "['motionConnect','motionConnectTrained','motionShadow'")
        s=once(s,"el('motionConnect').onclick=()=>runMotion('connect');",
            "el('motionConnect').onclick=()=>runMotion('connect',{model_set:'original'});\nel('motionConnectTrained').onclick=()=>runMotion('connect',{model_set:'trained'});")
        s=once(s,"el('motionConnect').textContent='连接并载入模型';", "el('motionConnect').textContent='加载原版模型';")
        s=once(s,"el('motionConnect').textContent=connected?'已连接 · 模型已载入':'连接并载入模型';",
            "el('motionConnect').textContent='加载原版模型';\n  el('motionConnectTrained').textContent='加载自训练模型';\n  const modelSet=m?.model_set || 'original';\n  el('motionModelSetStatus').textContent='当前模型组：'+(m?.model_set_label || (modelSet==='trained'?'自训练模型':'原版模型'))+(connected?' · 已连接':' · 未连接');\n  el('motionConnect').setAttribute('aria-pressed',String(modelSet==='original'));\n  el('motionConnectTrained').setAttribute('aria-pressed',String(modelSet==='trained'));")
        return s
    edit('service/command_view_ui.js', ui)
    selection=json.loads((training/'selected-models-v1.json').read_text())
    destination=output/'policy/model-sets/trained'
    destination.mkdir(parents=True)
    model_manifest={'models':{}}
    hashes={}
    for name, entry in selection['models'].items():
        raw=(training/entry['model']).read_bytes()
        digest=hashlib.sha256(raw).hexdigest()
        if digest!=entry['sha256']:raise ValueError('Selected model changed: '+name)
        filename='xgoduck_'+name+'.onnx'
        (destination/filename).write_bytes(raw)
        hashes[name]=digest
        model_manifest['models'][filename]={'sha256':digest,'size_bytes':len(raw)}
        changed.append('policy/model-sets/trained/'+filename)
    (destination/'manifest.json').write_text(json.dumps(model_manifest,indent=2)+'\n')
    changed.append('policy/model-sets/trained/manifest.json')
    deploy_path=output/'service/deployment.json'
    deployment=json.loads(deploy_path.read_text())
    deployment['model_sets']={
        'original':{'label':'原版模型','model_hashes':deployment['model_hashes'].copy()},
        'trained':{'label':'自训练模型','model_hashes':hashes,'training_selection':'864g-20261014','hardware_verified':False}}
    deployment['default_model_set']='original'
    deploy_path.write_text(json.dumps(deployment,ensure_ascii=False,indent=2)+'\n')
    changed.append('service/deployment.json')
    for name in changed:
        if name.endswith('.py'):compile((output/name).read_bytes(),name,'exec')
    report=dict(source=str(source),candidate=str(output),changed_files={n:hashlib.sha256((output/n).read_bytes()).hexdigest() for n in changed},
        default_model_set='original',automatic_motor_start=False,calibration_changed=False,
        original_models_changed=False,controller_parameters_changed=False,firmware_changed=False)
    (output/'model-sets-preparation.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--training',type=Path,required=True);a=p.parse_args()
    print(json.dumps(prepare(a.source,a.output,a.training),indent=2))
