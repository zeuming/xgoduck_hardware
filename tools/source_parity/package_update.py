"""Prepare pinned paired update from reviewed installers; no network/hardware."""
import argparse
import ast
import hashlib
import io
import json
from pathlib import Path
import tarfile

sha = lambda data: hashlib.sha256(data).hexdigest()


def pack(files):
    stream = io.BytesIO()
    with tarfile.open(fileobj=stream, mode='w:gz') as archive:
        for name, data in sorted(files.items()):
            info = tarfile.TarInfo(name); info.size = len(data); info.mode = 0o600
            archive.addfile(info, io.BytesIO(data))
    return stream.getvalue()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    args = parser.parse_args(); root = args.root
    repo = Path(__file__).resolve().parents[2]
    baseline = json.loads((root/'install-baseline.json').read_text())
    snapshot = json.loads((root/'baseline/snapshot.json').read_text())
    changes = json.loads(Path(__file__).with_name('changes.json').read_text())
    assert baseline['seal'] == changes['baseline_installation_sha256']
    for check in ('host-test-result', 'gamepad-test-result', 'calibration-test-result'):
        assert json.loads((root/(check+'.json')).read_text())['passed']
    assert all(v['passed'] for v in json.loads((root/'firmware-regression-result.json').read_text()))
    assert json.loads((root/'upstream-packet-comparison/result.json').read_text())['passed']
    files = {}
    for change in changes['changes']:
        data = (root/'candidate'/change['path']).read_bytes()
        assert sha(data) == change['after_sha256']
        if not change['path'].startswith('firmware/'):
            files[change['path']] = data
    stage = '/dev/shm/xgoduck-source-parity-20261012'
    old = repo/'output/duck_sim/official-execution-20261012/install-20261012'
    source = (old/'reviewed-host-install.py').read_text()
    replacements = {'STAGE': 'STAGE=Path('+repr(stage)+')',
                    'EXPECTED_SEAL': 'EXPECTED_SEAL='+repr(baseline['seal']),
                    'EXPECTED_BOOT': 'EXPECTED_BOOT='+repr(baseline['boot']),
                    'FILES': 'FILES='+repr(set(files))}
    for node in ast.parse(source).body:
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
            key = node.targets[0].id
            if key in replacements:
                source = source.replace(ast.get_source_segment(source,node), replacements[key],1)
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Assign) and isinstance(node.targets[0],ast.Name) and node.targets[0].id == 'expected_policy':
            expected = {k:v for k,v in snapshot['all_archive_members'].items()
                        if k.startswith('policy/') and k.endswith('.py')}
            source = source.replace(ast.get_source_segment(source,node),'expected_policy='+repr(expected),1)
            break
    source = source.replace("saved['revision']==25", "saved['revision']=="+str(baseline['calibration_revision']))
    source = source.replace("calibration_revision=25", "calibration_revision="+str(baseline['calibration_revision']))
    source = source.replace('2bf1352cbe9597f88f732f24ee481c8b7cf55fef041b77eb921da000a3c9251f',baseline['calibration_sha'])
    source = source.replace('Enable capability checked official policy goal execution and original HOME/EMA decode; preserve installed scheduling, calibration, stance transition and guards',
                            'Source-aligned grouped 6ms servo polling and asynchronous 2Mbps targets; no per-ID ON ARM acceptance; shared model/calibration HOME')
    compile(source,'<pinned host update>','exec')
    files['hashes.json']=(json.dumps({n:sha(v) for n,v in files.items()},indent=2)+'\n').encode()
    files['install.py']=source.encode()
    output=root/'install';output.mkdir(exist_ok=False)
    (output/'host.tar.gz').write_bytes(pack(files))
    (output/'reviewed-host-install.py').write_text(source)
    # Preserve the previously reviewed powered-OFF gate, single flash attempt,
    # backup-before-write handshake and full extent readback; only repin inputs.
    flash_old=repo/'output/duck_sim/arm-readback-20261012/install-position-first-v1'
    source=(flash_old/'reviewed-flash-source.py').read_text();constants={}
    for node in ast.parse(source).body:
        if isinstance(node,ast.Import): start=node.lineno;break
        constants[node.targets[0].id]=ast.literal_eval(node.value)
    body=''.join(source.splitlines(True)[start-1:])
    image=(root/'candidate/firmware/.pio/build/esp32s3_motion/firmware.bin').read_bytes()
    constants.update(BOOT=baseline['boot'],INSTALL_SEAL=baseline['seal'],CAL_SHA256=baseline['calibration_sha'],
                     OLD_SIZE=constants['APP_SIZE'],OLD_SHA256=constants['APP_SHA256'],
                     APP_SIZE=len(image),APP_SHA256=sha(image),REGION_SIZE=(len(image)+4095)//4096*4096)
    flash_files={}
    with tarfile.open(flash_old/'reviewed-flash-bundle.tar.gz') as archive:
        for info in archive:
            data=archive.extractfile(info).read()
            assert sha(data)==constants['FILES'][info.name]['sha256']
            if info.name!='firmware.bin':flash_files[info.name]=data
    flash_files['firmware.bin']=image;bundle=pack(flash_files)
    constants.update(SIZE=len(bundle),BUNDLE_SHA=sha(bundle),
                     FILES={n:dict(size=len(v),sha256=sha(v)) for n,v in flash_files.items()})
    source='\n'.join(k+' = '+repr(v) for k,v in constants.items())+'\n'+body
    compile(source,'<pinned firmware update>','exec')
    (output/'reviewed-flash-source.py').write_text(source)
    (output/'reviewed-flash-bundle.tar.gz').write_bytes(bundle)
    report=dict(stage=stage,baseline=baseline,host_sha256=sha((output/'host.tar.gz').read_bytes()),
                host_members=sorted(files),firmware_sha256=sha(image),firmware_size=len(image),
                flash_bundle_sha256=sha(bundle),flash_source_sha256=sha(source.encode()),
                old_firmware_sha256=constants['OLD_SHA256'],flash_extent=constants['REGION_SIZE'],
                installed=False,flashed=False,automatic_motion=False)
    (output/'preparation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
