"""Separate measured timeout excess from controller computation, by host."""
import argparse,hashlib,json,statistics
from pathlib import Path


def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def stats(values):
    values=sorted(values)
    return dict(count=len(values),mean=statistics.mean(values),maximum=max(values),
        p95=values[int(.95*(len(values)-1))]) if values else dict(count=0)


def summarize(directory):
    meta=json.loads((directory/'meta.json').read_text());assert not meta['dropped']
    rows=[json.loads(s) for s in (directory/'events.jsonl').read_text().splitlines()]
    segments={}
    for seg in ('no_display_readers','cached_http_process_eight_readers'):
        r=[x for x in rows if x['segment']==seg];summary={}
        for name in ('controller_event_wait','owner_condition_wait'):
            waits=[x for x in r if x['event']==name and not x['signalled']]
            summary[name]={k:stats([x[k] for x in waits if x[k] is not None]) for k in
                ('requested_ms','measured_wait_ms','excess_over_requested_ms')}
        summary['controller_active_ms']=stats([x['wall_ms'] for x in r if x['event']=='_cycle'])
        summary['control_lock_ms']=stats([x['wall_ms'] for x in r if x['event']=='control_lock'])
        segments[seg]=summary
    return dict(source=str(directory),files_sha256={p.name:digest(p) for p in (directory/'meta.json',directory/'events.jsonl')},segments=segments)


def main():
    p=argparse.ArgumentParser();p.add_argument('--host',required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('directories',type=Path,nargs='+');a=p.parse_args()
    report=dict(hardware_access=False,host=a.host,runs=[summarize(d) for d in a.directories],
        limits=['Observed timeout excess is not alone a diagnosis of OS cause or CPU throttling.',
                'Instrumentation has nonzero overhead. Do not extrapolate Mac timing to X5.',
                'Historical delay injection includes prior host time; it is not a wire-level measurement.'])
    a.output.write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
