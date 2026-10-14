"""Summarize observed fake-UART events without counting nested spans twice."""
import argparse
from collections import Counter,defaultdict
import json
from pathlib import Path
import statistics


def distribution(values):
    values=sorted(values)
    return dict(count=len(values),mean=statistics.mean(values),maximum=max(values),
                p95=values[min(len(values)-1,int(.95*(len(values)-1)))]) if values else dict(count=0)


def union_ms(intervals,start,end):
    intervals=sorted((max(a,start),min(b,end)) for a,b in intervals if b>start and a<end)
    total=0.;left=right=None
    for a,b in intervals:
        if b<=a:continue
        if left is None:left,right=a,b
        elif a<=right:right=max(right,b)
        else:total+=right-left;left,right=a,b
    if left is not None:total+=right-left
    return total*1000


def summarize(directory):
    directory=Path(directory)
    meta=json.loads((directory/'meta.json').read_text())
    if meta['dropped']:raise ValueError('Incomplete event log')
    rows=[json.loads(line) for line in (directory/'events.jsonl').read_text().splitlines()]
    results={}
    for segment in dict.fromkeys(row['segment'] for row in rows):
        events=[r for r in rows if r['segment']==segment]
        ticks=[r for r in events if r['event']=='_cycle']
        logical=defaultdict(list)
        for row in events:
            if row.get('logical_id') is not None:logical[row['logical_id']].append(row)
        all_writes=[r for r in events if r['event']=='target_write']
        groups=[]
        for logical_id,items in logical.items():
            enqueues=[r for r in items if r['event']=='target_enqueue']
            writes=[r for r in items if r['event']=='target_write']
            acks=[r for r in items if r['event']=='transaction_end' and r['kind']==5 and r['transmitted'] and r['error'] is None]
            if len(writes)>1 or len(acks)>1:raise ValueError('Logical target repeated')
            groups.append(dict(logical_id=logical_id,enqueues=len(enqueues),writes=len(writes),acks=len(acks),
                retries=max(0,len(enqueues)-1),
                enqueue_to_write_ms=(writes[0]['at']-enqueues[0]['at'])*1000 if writes and enqueues else None,
                write_to_ack_ms=(acks[0]['at']-writes[0]['at'])*1000 if writes and acks else None))
        gaps=[]
        for first,last in zip(all_writes,all_writes[1:]):
            start,end=first['at'],last['at']
            active=[r for r in events if start<r['at']<=end]
            spans=[r for r in events if 'start' in r and r['at']>start and r['start']<end]
            injected=[(r['start'],r['at']) for r in spans if r['event']=='injected_wait']
            controller=[(r['start'],r['at']) for r in spans if r['event'] in ('_cycle','_io_drain_ack')]
            gaps.append(dict(from_logical=first['logical_id'],to_logical=last['logical_id'],
                gap_ms=(end-start)*1000,
                cycle_outcomes=dict(Counter(r['outcome'] for r in active if r['event']=='_cycle')),
                injected_wait_union_ms=union_ms(injected,start,end),
                control_active_union_ms=union_ms(controller,start,end),
                explained_activity_union_ms=union_ms(injected+controller,start,end),
                no_logged_active_span_ms=(end-start)*1000-union_ms(injected+controller,start,end),
                # Idle includes periodic waits, worker work outside this process,
                # scheduler delay and instrumentation. Do not label it a cause.
                idle_or_uninstrumented_is_not_proven_lock_wait=True))
        names=['_cycle','_io_drain_ack','_io_accept_cache_locked','_motion_gate_locked',
               '_raw_targets_locked','_async_policy_step','inference_submit','inference_take','control_lock']
        results[segment]=dict(tick_outcomes=dict(Counter(r['outcome'] for r in ticks)),
            logical_targets=len(groups),logical_acknowledged=sum(g['acks'] for g in groups),
            retry_histogram=dict(Counter(g['retries'] for g in groups)),
            write_spacing_ms=distribution([g['gap_ms'] for g in gaps]),
            intervals_exceeding_20ms=sum(g['gap_ms']>20 for g in gaps),
            wall_by_method_ms={name:distribution([r['wall_ms'] for r in events if r['event']==name]) for name in names},
            logical=groups,gaps=gaps)
    return dict(source=str(directory),physical_uart=False,
        interval_threshold_ms=20,origin_of_threshold='existing PERIOD=.02, not watchdog/freshness',
        grouping_edges_can_span_segments=True,segments=results)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('ledger',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();a.output.write_text(json.dumps(summarize(a.ledger),indent=2)+'\n')
