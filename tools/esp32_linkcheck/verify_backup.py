#!/usr/bin/env python3
"""Verify a complete backup with the ESP app halted; never writes flash.

Factory applications may update NVS after esptool's default hard reset.
Keep the first snapshot, read only changed 64-KiB blocks, then independently
compare the complete reconstructed snapshot with the device's flash MD5.
Requires esptool 4.11.0 and an already confirmed 16-MiB ESP32-S3.
"""
import argparse
import hashlib
import json
import pathlib
import struct

from esptool.targets.esp32s3 import ESP32S3ROM


def partitions(data):
    result = []
    for offset in range(0x8000, 0x8C00, 32):
        magic, kind, subtype, address, size, label, flags = struct.unpack(
            '<HBBII16sI', data[offset:offset + 32])
        if magic != 0x50AA:
            break
        result.append({'name': label.split(b'\0', 1)[0].decode('ascii', 'replace'),
                       'type': kind, 'subtype': subtype,
                       'offset': address, 'size': size})
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--original', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    original = pathlib.Path(args.original).read_bytes()
    if len(original) != 16777216:
        raise ValueError('Expected a complete 16-MiB backup')
    snapshot = bytearray(original)
    table = partitions(original)
    changed = []
    rom = ESP32S3ROM(args.port, baud=115200)
    esp = rom
    try:
        rom.connect()
        esp = rom.run_stub()
        esp.change_baud(460800)
        # Same volatile ROM/stub flashchip RAM setting used by esptool CLI.
        esp.flash_set_parameters(len(original))
        coarse = 1048576
        fine = 65536
        for offset in range(0, len(original), coarse):
            expected = hashlib.md5(original[offset:offset + coarse]).hexdigest()
            if esp.flash_md5sum(offset, coarse) == expected:
                print(f'Block {offset:#010x}: matched', flush=True)
                continue
            for start in range(offset, offset + coarse, fine):
                expected = hashlib.md5(original[start:start + fine]).hexdigest()
                if esp.flash_md5sum(start, fine) == expected:
                    continue
                current = esp.read_flash(start, fine)
                snapshot[start:start + fine] = current
                differing = [start + i for i, (old, new) in
                             enumerate(zip(original[start:start + fine], current))
                             if old != new]
                owners = sorted({p['name'] for p in table if any(
                    p['offset'] <= address < p['offset'] + p['size']
                    for address in differing)})
                changed.append({'offset': start, 'size': fine,
                                'changed_bytes': len(differing),
                                'first_changed_address': differing[0],
                                'partitions': owners})
                print(f'Changed block {start:#010x}; {len(differing)} bytes; partitions={owners}', flush=True)
        expected = hashlib.md5(snapshot).hexdigest()
        actual = esp.flash_md5sum(0, len(snapshot))
        if actual != expected:
            raise ValueError('Stable full-device MD5 verification failed; no verified backup saved')
        output = pathlib.Path(args.output)
        output.write_bytes(snapshot)
        output.chmod(0o600)
        manifest = {'file': output.name, 'size_bytes': len(snapshot),
                    'sha256': hashlib.sha256(snapshot).hexdigest(),
                    'md5': expected, 'device_md5': actual,
                    'verified_against_halted_device': True,
                    'original_sha256': hashlib.sha256(original).hexdigest(),
                    'changed_blocks_after_factory_restart': changed,
                    'partition_table': table,
                    'flash_written': False}
        output.with_suffix('.json').write_text(json.dumps(manifest, indent=2) + '\n')
        print(json.dumps(manifest, indent=2), flush=True)
    finally:
        # Keep the application halted; no factory restart between this check
        # and the separately authorized diagnostic flash.
        esp._port.close()


if __name__ == '__main__':
    main()
