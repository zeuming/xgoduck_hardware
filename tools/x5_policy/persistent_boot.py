"""Boot-local guards for the installed X5 workbench. No I/O on import."""
import copy
import hashlib
import json
import re

CARD = '035344534b33324785d55cb687017c00'
MODEL = 'D-Robotics RDK X5 V1.0'
KNOWN_BOOT_WARNINGS = (
    re.compile(r'^\[\s*\d+(?:\.\d+)?\] systemd-journald\[\d+\]: File /var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system\.journal corrupted or uncleanly shut down, renaming and replacing\.$'),
    re.compile(r'^\[\s*\d+(?:\.\d+)?\] FAT-fs \(mmcblk1p1\): Volume was not properly unmounted\. Some data may be corrupt\. Please run fsck\.$'),
)

# Source-reviewed completed return, not an automatically learned fault baseline.
# Three metadata-only observations retained the full kernel history, the same
# CH341 (1a86:7523) at 1-1.4, /dev/ttyUSB0 inode 640, and no UART holders.
# Evidence: head-preview-usb-metadata-t9s8rxt1/stdout. Cause remains unknown.
REVIEWED_USB_RETURN = dict(
    boot_id='91ac84d9-a758-454d-9567-ef53dac9b29e',
    kernel_sha256='3a3ade99545a3892ad00e939e3cb2b692ee01898e3294eb4ec0feca95438352b',
    kernel_lines=934,
    fault_line='[ 8091.386253] usb 1-1.4: USB disconnect, device number 3',
    evidence_sha256='14536a0568cba311e8b220796010d960c610c1e70474c04de704168d530462f9',
)


def check_platform(metadata):
    if metadata['cid'] != CARD or metadata['model'] != MODEL:
        raise ValueError('Installed workbench belongs to a different X5/card')
    if re.fullmatch(r'[0-9a-f]{8}(?:-[0-9a-f]{4}){3}-[0-9a-f]{12}', metadata['boot_id']) is None:
        raise ValueError('Invalid current boot ID')
    fields = metadata['root_mount_line'].split()
    try:
        separator = fields.index('-', 6)
        valid = (fields[2:5] == ['179:2', '/', '/'] and fields[separator+1:separator+3] == ['ext4', '/dev/root']
                 and 'rw' in fields[5].split(',') and 'ro' not in fields[5].split(',')
                 and 'ro' not in fields[separator+3].split(','))
    except (IndexError, ValueError):
        valid = False
    if not valid or metadata['swaps'].splitlines() != ['Filename\t\t\t\tType\t\tSize\t\tUsed\t\tPriority']:
        # proc's spacing is not stable; require the exact header fields, no rows.
        swaps = metadata['swaps'].splitlines()
        if not valid or len(swaps) != 1 or swaps[0].split() != ['Filename','Type','Size','Used','Priority']:
            raise ValueError('Root filesystem or swap configuration changed')


def restore_kernel_history(kernel, retained):
    """Restore an evicted prefix only across an exact, unambiguous overlap.

    Linux dmesg is a ring, not an append-only file. The retained bytes remain
    evidence; newly observed lines still require the caller's fault validation.
    A gap, edited overlap or short/ambiguous match grants no recovery.
    """
    if (type(kernel) is not str or type(retained) is not str
            or not kernel.endswith('\n') or not retained.endswith('\n')
            or max(len(kernel.encode()), len(retained.encode())) > 262144):
        raise ValueError('Kernel history is incomplete or exceeds its bound')
    lines, baseline = kernel.splitlines(), retained.splitlines()
    if not lines or not baseline:
        raise ValueError('Kernel history is empty')
    if lines[:len(baseline)] == baseline:
        return kernel
    # Sixty-four identical timestamped records are required, including the
    # retained tail. This deliberately rejects weak one-line boot matches.
    matches = [i for i in range(1, len(baseline)-63)
               if baseline[i] == lines[0] and baseline[i:] == lines[:len(baseline)-i]]
    if len(matches) != 1:
        raise ValueError('Kernel history has a gap or changed overlap; cannot reconstruct it')
    offset = matches[0]
    overlap = baseline[offset:]
    records = overlap + lines[len(overlap):]
    # cfg80211 emits this fixed regulatory notice as one multiline printk.
    # Recognize only its complete, timestamp-anchored continuation; preserve
    # every byte for the unchanged fault classifier below.
    banner = [
        '               *******************************************************',
        '               ** CAUTION: USING PERMISSIVE CUSTOM REGULATORY RULES **',
        '               *******************************************************',
    ]
    continuations = set()
    for index, line in enumerate(records):
        if (re.fullmatch(r'\[\s*\d+(?:\.\d+)?\] ieee80211 phy\d+: ', line)
                and records[index+1:index+4] == banner):
            continuations.update(range(index+1, index+4))
    times = []
    for index, line in enumerate(records):
        match = re.match(r'^\[\s*(\d+(?:\.\d+)?)\]', line)
        if not match:
            # Existing multiline printk continuations are already covered by
            # the exact overlap. New unframed text cannot establish continuity.
            if index < len(overlap) or index in continuations:
                continue
            raise ValueError('New kernel tail contains an untimestamped record')
        times.append(float(match[1]))
    if len(times)<2 or times[-1] < 1 or any(a > b for a,b in zip(times,times[1:])):
        raise ValueError('Kernel overlap timestamps are invalid')
    restored = '\n'.join(baseline[:offset]+lines)+'\n'
    if len(restored.encode()) > 262144:
        raise ValueError('Reconstructed kernel history exceeds its bound')
    return restored


def boot_expected(template, metadata, classify):
    check_platform(metadata)
    kernel = metadata['kernel_log']
    lines = kernel.splitlines()
    first = re.match(r'^\[\s*(\d+(?:\.\d+)?)\]', lines[0]) if lines else None
    if len(kernel.encode()) > 262144 or not kernel.endswith('\n') or not first or float(first[1]) > 1:
        raise ValueError('Complete early boot kernel history is unavailable')
    # Never absorb a new fault into an allowed baseline. Only a source-reviewed
    # exact history may pass persistent_kernel afterwards, retaining its fault.
    end = next((i for i,line in enumerate(lines) if classify(line) and not any(p.fullmatch(line) for p in KNOWN_BOOT_WARNINGS)), len(lines))
    prefix = '\n'.join(lines[:end])+'\n'
    value = copy.deepcopy(template)
    value.pop('reviewed_reconnect_proof', None)
    value.update({key:metadata[key] for key in ('boot_id','cid','model','root_mount_line')})
    value['kernel_prefix'] = prefix
    value['historical_faults'] = [dict(line=line,reason=classify(line)) for line in lines[:end] if classify(line)]
    value['native_persistent_boot'] = dict(schema_version=1, boot_id=value['boot_id'], cid=CARD,
        kernel_sha256=hashlib.sha256(prefix.encode()).hexdigest())
    return value


def persistent_kernel(kernel, expected, classify):
    marker = expected.get('native_persistent_boot')
    prefix = expected['kernel_prefix']
    if marker != dict(schema_version=1,boot_id=expected['boot_id'],cid=CARD,kernel_sha256=hashlib.sha256(prefix.encode()).hexdigest()):
        raise ValueError('Installed boot baseline changed')
    if expected['cid'] != CARD or expected['model'] != MODEL:
        raise ValueError('Installed boot/card identity changed')
    kernel = restore_kernel_history(kernel, prefix)
    lines, baseline = kernel.splitlines(), prefix.splitlines()
    if not baseline or lines[:len(baseline)] != baseline or len(kernel.encode()) > 262144:
        raise ValueError('Retained kernel history changed or lost its boot prefix')
    faults = [dict(line=line,reason=classify(line)) for line in baseline if classify(line)]
    if faults != expected['historical_faults'] or any(not any(p.fullmatch(f['line']) for p in KNOWN_BOOT_WARNINGS) for f in faults):
        raise ValueError('Unrecognized boot fault; hardware remains locked')
    new = [dict(line=line,reason=classify(line)) for line in lines[len(baseline):] if classify(line)]
    reviewed = []
    receipt = REVIEWED_USB_RETURN
    count = receipt['kernel_lines']
    if (expected['boot_id'] == receipt['boot_id'] and count > len(baseline)
            and len(lines) >= count
            and hashlib.sha256(('\n'.join(lines[:count])+'\n').encode()).hexdigest() == receipt['kernel_sha256']):
        reviewed = [dict(line=line,reason=classify(line)) for line in lines[len(baseline):count] if classify(line)]
        if [f['line'] for f in reviewed] != [receipt['fault_line']]:
            raise ValueError('Reviewed USB return does not match retained faults')
        new = [dict(line=line,reason=classify(line)) for line in lines[count:] if classify(line)]
    if new:
        evidence = dict(new_fault_count=len(new),faults=[dict(source='new_kernel_tail',line=f['line'],classifier_reason=f['reason'],line_truncated=False) for f in new[:4]],fault_details_truncated=len(new)>4)
        raise ValueError('New kernel fault; classifier result requires source review: '+json.dumps(evidence,ensure_ascii=False,separators=(',',':')))
    return dict(kernel_log=kernel,kernel_sha256=hashlib.sha256(('\n'.join(lines)+'\n').encode()).hexdigest(),
        kernel_lines=len(lines),historical_faults=faults+reviewed,historical_faults_cleared=False,
        reviewed_usb_return=None if not reviewed else dict(boot_id=receipt['boot_id'],
            kernel_sha256=receipt['kernel_sha256'],cause_known=False,automatic_retry=False))


# This file is also appended to the original definitions in each guarded child.
# Old RAM deployments retain their existing fixed-profile validator.
_installed_previous_validate_kernel = globals().get('validate_kernel')
def validate_kernel(kernel, expected):
    if 'native_persistent_boot' not in expected:
        return _installed_previous_validate_kernel(kernel, expected)
    try:
        return persistent_kernel(kernel, expected, classify_feedback_log)
    except ValueError as error:
        raise ProbeError(str(error)) from error
