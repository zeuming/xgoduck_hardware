#!/usr/bin/env python3
"""Bounded off-board recorder: cached HTTP GET only, never motion commands.

Store sparse status outside X5 so a board power loss cannot erase everything.
One sample/second is not a voltage waveform or a control-cycle trace. Failed
GETs are recorded and never interpreted as a STOP request or proof of reboot.
"""
import argparse
import datetime
import http.cookiejar
import json
import time
import urllib.parse
import urllib.request
from pathlib import Path


def select_status(value):
    m = value.get('motion') or {}
    h = m.get('hardware') or {}
    proof = m.get('closed_proof') or {}
    health = value.get('system_health') or {}
    return dict(phase=m.get('phase'), owns_connection=m.get('owns_connection'),
        connection_sequence=m.get('connection_sequence'),
        hardware={k:h.get(k) for k in ('generation','mode','armed','sample_us',
            'imu_us','age_ms','imu_age_ms','command','imu','rawdata',
            'last_policy_target_deg','last_executed_target_deg','error',
            'firmware_error','firmware_first_stop','release_confirmed',
            'position_feedback_ages_us','rates','official_runtime')},
        operation={k:(m.get('last_operation') or {}).get(k) for k in
            ('action','mode','generation','state','error','stop_source','recorded_monotonic')},
        closure={k:proof.get(k) for k in ('serial_closed','release_confirmed','no_uart_opened')},
        system_health=health)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--url',required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=int,default=120)
    a=p.parse_args();url=urllib.parse.urlsplit(a.url)
    if url.scheme!='http' or url.username or url.password or url.query or url.fragment or url.path not in ('','/'):
        raise ValueError('Specify plain workbench origin without credentials')
    if not 1<=a.seconds<=300:raise ValueError('Capture must last 1..300 seconds')
    origin=urllib.parse.urlunsplit((url.scheme,url.netloc,'','',''))
    opener=urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
    with opener.open(origin+'/',timeout=3) as response:response.read(4*1024*1024)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    started=time.monotonic();next_at=started
    with a.output.open('x') as stream:
        while time.monotonic()-started<a.seconds:
            began=time.monotonic();row=dict(wall_time=datetime.datetime.now().astimezone().isoformat(),
                elapsed_s=began-started,scope='cached_GET_only_1Hz_not_cycle_trace')
            try:
                with opener.open(origin+'/api/live-status',timeout=3) as response:
                    raw=response.read(4*1024*1024+1)
                    if len(raw)>4*1024*1024:raise ValueError('Snapshot exceeds bound')
                row['status']=select_status(json.loads(raw))
            except Exception as error:row['read_error']=type(error).__name__+': '+str(error)
            row['request_ms']=(time.monotonic()-began)*1000
            stream.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n');stream.flush()
            next_at+=1.
            time.sleep(max(0.,min(next_at,started+a.seconds)-time.monotonic()))


if __name__=='__main__':main()
