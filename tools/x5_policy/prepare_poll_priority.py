"""Create a hardware-denied v4 descendant; never install/restart production."""
import argparse,hashlib,json,shutil
from pathlib import Path
BASE_OWNER='a13c2809e4ca1dc62ff5c53fc5f583d729081005a25575a19b4636ab68adfd22'

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def prepare(source,destination,manifest_path):
    manifest=json.loads(manifest_path.read_text())
    if sha(source/'policy/serial_owner.py')!=BASE_OWNER:raise ValueError('Not reviewed v4 owner')
    if destination.exists():raise FileExistsError(destination)
    files={n.removeprefix('candidate/'):d for n,d in manifest['files'].items() if n.startswith('candidate/') and n.endswith('.py')}
    for name,digest in files.items():
        if sha(source/name)!=digest:raise ValueError('Source changed '+name)
    destination.mkdir(parents=True)
    for name in files:
        target=destination/name;target.parent.mkdir(exist_ok=True,parents=True);shutil.copyfile(source/name,target)
    target=destination/'policy/serial_owner.py';value=target.read_text()
    anchor='    def _run(self):\n'
    method='''    def _reserve_pending_target(self, now, target_due):
        """Delay a new STATE poll only until an already-admitted target is due.

        No new action, no changed deadline, no bypass of the final write fence.
        A missing/stale/superseded target never delays STATE acquisition.
        Called with cv held. All intervals stay in their original clock domains.
        """
        ticket = self.target
        if (target_due is None or target_due <= now or ticket is None or
                ticket.done.is_set() or ticket.cancelled or ticket.epoch != self.epoch or
                self.latest is None or self.latest.serial != ticket.state_serial):
            return False
        return self.clock() + (target_due-now) < ticket.deadline

'''
    if value.count(anchor)!=1:raise ValueError('Run anchor changed')
    value=value.replace(anchor,method+anchor,1)
    anchor='                    poll_at=max(self.next_poll,self.revalidation_until)\n'
    if value.count(anchor)!=1:raise ValueError('Poll anchor changed')
    value=value.replace(anchor,anchor+'''                    if self._reserve_pending_target(now,target_due):
                        # Waiting is bounded by the unchanged 20ms write interval.
                        # Queue/STOP notifications still wake the condition first.
                        poll_at=max(poll_at,target_due)
''',1)
    compile(value,str(target),'exec');target.write_text(value)
    report=dict(hardware_access=False,deployed=False,source=str(source.resolve()),base_files=files,
        changed={'policy/serial_owner.py':sha(target)},copied_only_python_allowlist=True,
        forbidden_contents_copied=False,parameters_unchanged=dict(minimum_write_interval_s=.02,freshness_s=.1,watchdog_s=.25),
        semantics='Do not start a new blocking STATE transaction while an already-admitted current-state TARGET waits for its original next-write time. Final write fence, original deadlines, single outstanding TARGET, ACK recurrence and priority STOP unchanged.')
    (destination/'poll-priority-manifest.json').write_text(json.dumps(report,indent=2)+'\n');return report

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for n in ('source','destination','manifest'):p.add_argument('--'+n,type=Path,required=True)
    a=p.parse_args();print(json.dumps(prepare(a.source,a.destination,a.manifest),indent=2))
