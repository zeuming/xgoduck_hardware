"""Prepare a sealed process-isolation upgrade from an existing installation.

Preserve the deployed policy, candidate constructor, UI and calibration seed.
This only creates local artifacts; it never accesses a robot or installs them.
"""
import argparse
import hashlib
import io
import json
from pathlib import Path
import tarfile

HERE = Path(__file__).resolve().parent


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def unpack(raw):
    with tarfile.open(fileobj=io.BytesIO(raw), mode='r:gz') as archive:
        values = {}
        for item in archive.getmembers():
            path = Path(item.name)
            if (not item.isfile() or path.is_absolute() or '..' in path.parts
                    or item.name in values or item.size > 128 * 1024 * 1024):
                raise ValueError('Unexpected installation member')
            values[item.name] = archive.extractfile(item).read()
        return values


def pack(values):
    output = io.BytesIO()
    with tarfile.open(fileobj=output, mode='w:gz') as archive:
        for name, raw in sorted(values.items()):
            item = tarfile.TarInfo(name)
            item.size, item.mode, item.mtime = len(raw), 0o600, 0
            archive.addfile(item, io.BytesIO(raw))
    return output.getvalue()


def replace(source, old, new):
    if source.count(old) != 1:
        raise ValueError('Deployment source differs at: ' + old[:100])
    return source.replace(old, new, 1)


def prepare(source, destination):
    source, destination = Path(source), Path(destination)
    original = source.read_bytes()
    outer = unpack(original)
    seal = json.loads(outer['installation.json'])
    for name, sha in seal['files'].items():
        if digest(outer[name]) != sha:
            raise ValueError('Installation seal differs: ' + name)
    manifest = json.loads(outer['deployment.json'])
    if digest(outer['native-bundle.tar.gz']) != manifest['archive_sha256']:
        raise ValueError('Inner archive differs')
    members = unpack(outer['native-bundle.tar.gz'])
    for name, info in manifest['files'].items():
        if len(members[name]) != info['size_bytes'] or digest(members[name]) != info['sha256']:
            raise ValueError('Inner member differs: ' + name)
    old_members = dict(members)
    bootstrap = members['service/bootstrap.py'].decode()
    bootstrap = replace(bootstrap,
        'import native_calibration_store as calibration_storage,native_transports',
        'import native_calibration_store as calibration_storage,native_transports,native_process')
    bootstrap = replace(bootstrap,
        'native_transports.allowed_child_command(args[0],args[1])):',
        'native_transports.allowed_child_command(args[0],args[1])) and not (\n'
        '                native_process.allowed_web_command(args[0],args[1],root)):')
    bootstrap = replace(bootstrap, 'native_workbench.serve(runtime,', 'native_process.serve(runtime,')
    members['service/bootstrap.py'] = bootstrap.encode()
    server = members['service/native_workbench.py'].decode()
    server = replace(server, 'def __init__(self, address, handler, *, max_threads=12):',
        'def __init__(self, address, handler, *, max_threads=12, listener=None):')
    server = replace(server, '        super().__init__(address, handler)',
        '        super().__init__(address, handler, bind_and_activate=listener is None)\n'
        '        if listener is not None:\n'
        '            self.socket.close()\n'
        '            self.socket = listener\n'
        '            self.server_address = listener.getsockname()')
    members['service/native_workbench.py'] = server.encode()
    # Retain candidate_profile and every deployed policy gate. Restrict these
    # changes to display/log serialization; full STOP evidence is saved closed.
    runtime = members['service/native_runtime.py'].decode()
    compact_runtime = all(marker in runtime for marker in (
        "self._record('live-state.json', self.snapshot(compact=True))",
        "native_compact_stop_diagnostic=_compact_diagnostic",
        "separators=(',', ':')"))
    if not compact_runtime:
        runtime = replace(runtime,
            "json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2)",
            "json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':'))")
        runtime = replace(runtime, "self._record('live-state.json', self.snapshot())",
            "self._record('live-state.json', self.snapshot(compact=True))")
        runtime = replace(runtime,
            "opened_transports=[], health_error=None, exit_event=threading.Event())",
            "opened_transports=[], health_error=None, exit_event=threading.Event(),\n"
            "                         native_compact_stop_diagnostic=_compact_diagnostic)")
        runtime = replace(runtime, "            after = _own_controller_status(controller.status())\n",
            "            after = _own_controller_status(controller.status())\n"
            "            if self._serial_closed(controller, namespace):\n"
            "                after['last_stop_diagnostic'] = copy.deepcopy(\n"
            "                    getattr(controller, '_last_stop_diagnostic', None))\n")
    members['service/native_runtime.py'] = runtime.encode()
    worker = members['service/worker-classes.py'].decode()
    if "compact = globals().get('native_compact_stop_diagnostic')" not in worker:
        worker = replace(worker,
            "        result['last_stop_diagnostic'] = motion.copy.deepcopy(diagnostic)",
            "        compact = globals().get('native_compact_stop_diagnostic')\n"
            "        if callable(compact):\n"
            "            result['last_stop_diagnostic'] = compact(diagnostic)\n"
            "            if result['last_stop_diagnostic'] is not None:\n"
            "                result['last_stop_diagnostic']['details_deferred_until_UART_closed'] = True\n"
            "        else:\n"
            "            result['last_stop_diagnostic'] = motion.copy.deepcopy(diagnostic)")
    members['service/worker-classes.py'] = worker.encode()
    members['service/native_process.py'] = (HERE / 'native_process.py').read_bytes()
    for name in ('system_health.py', 'system_health_panel.html'):
        members['service/' + name] = (HERE / name).read_bytes()
    members['service/gamepad_web.py'] = (HERE / 'gamepad_web.py').read_bytes()
    members['service/native_gamepad.py'] = (HERE / 'native_gamepad.py').read_bytes()
    members['service/gamepad_panel.html'] = (HERE / 'gamepad_panel.html').read_bytes()
    changed = [name for name in members if members[name] != old_members.get(name)]
    for name in changed:
        if name.endswith('.py'):
            compile(members[name], '<' + name + '>', 'exec')
    if any(old_members[name] != members[name] for name in old_members if name.startswith(('policy/', 'web/'))):
        raise ValueError('Policy/calibration/UI unexpectedly changed')
    raw = pack(members)
    manifest.update(archive_sha256=digest(raw), archive_size=len(raw),
        member_bytes=sum(map(len, members.values())),
        files={name: dict(size_bytes=len(data), sha256=digest(data)) for name, data in members.items()})
    manifest['required_source_sha256'].update({name: digest(members[name]) for name in changed})
    outer['native-bundle.tar.gz'] = raw
    outer['deployment.json'] = (json.dumps(manifest, ensure_ascii=False, sort_keys=True, indent=2) + '\n').encode()
    seal['files'] = {name: digest(raw) for name, raw in outer.items() if name != 'installation.json'}
    outer['installation.json'] = (json.dumps(seal, ensure_ascii=False, sort_keys=True, indent=2) + '\n').encode()
    payload = pack(outer)
    destination.mkdir(mode=0o700, parents=True, exist_ok=False)
    archive = destination / 'installation.tar.gz'
    archive.write_bytes(payload)
    archive.chmod(0o600)
    report = dict(kind='separate_web_controller_processes', source_path=str(source.resolve()),
        source_sha256=digest(original), old_seal_sha256=digest(unpack(original)['installation.json']),
        package_sha256=digest(payload), changed_inner_members=changed,
        outer_files={name: dict(size=len(raw), sha256=digest(raw)) for name, raw in outer.items()},
        model_policy_calibration_UI_bytes_preserved=True, automatic_motion_start=False,
        firmware_changed=False, nonzero_commands_constructor_locked=(
            b'research_nonzero_command=False' in members.get('policy/motion.py', b'')))
    (destination / 'prepared-package.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source')
    parser.add_argument('destination')
    args = parser.parse_args()
    print(json.dumps(prepare(args.source, args.destination), ensure_ascii=False))
