"""Thermal/DVFS evidence, failure isolation and bounded diagnostic recording."""
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest import mock
import system_health as app


class HealthTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.sys, self.proc = self.root/'sys', self.root/'proc'
        self.zone = self.sys/'class/thermal/thermal_zone1'
        self.cool = self.sys/'class/thermal/cooling_device0'
        self.policy = self.sys/'devices/system/cpu/cpufreq/policy0'
        self.write(self.zone, type='thermal-cpu', temp='69866', trip_point_0_type='passive', trip_point_0_temp='95000')
        self.write(self.cool, type='cpufreq-cpu0', cur_state='0', max_state='4')
        self.write(self.policy, affected_cpus='0 1 2 3', scaling_governor='schedutil',
                   cpuinfo_cur_freq='300000', scaling_cur_freq='600000', scaling_max_freq='1500000', cpuinfo_max_freq='1500000')
        self.write(self.proc, stat='cpu 100 0 0 900 0 0 0 0 0 0\n')
        self.sampler = app.LinuxSampler(self.sys, self.proc)

    def write(self, directory, **values):
        directory.mkdir(parents=True, exist_ok=True)
        for name, value in values.items(): (directory/name).write_text(value)

    def test_idle_low_frequency_is_not_heat_limit(self):
        s=self.sampler.sample()
        self.assertEqual(s['state'],'no_limit_observed')
        self.assertAlmostEqual(s['zones'][0]['celsius'],69.866)
        self.assertEqual(s['policies'][0]['current_mhz'],300)
        self.assertIsNone(s['cpu_busy_percent'])

    def test_thermal_cooling_engaged_even_below_trip_after_hysteresis(self):
        self.write(self.cool, cur_state='2')
        self.assertEqual(self.sampler.sample()['state'],'thermal_limited')

    def test_unexplained_cap_does_not_claim_overheat(self):
        self.write(self.policy, scaling_max_freq='1200000')
        self.assertEqual(self.sampler.sample()['state'],'frequency_capped')

    def test_trip_comes_from_kernel_not_an_arbitrary_temperature(self):
        self.write(self.zone, temp='95000')
        self.assertEqual(self.sampler.sample()['state'],'trip_reached')
        self.write(self.zone, trip_point_0_temp='100000')
        self.assertEqual(self.sampler.sample()['state'],'no_limit_observed')

    def test_missing_cooling_is_unknown(self):
        (self.cool/'cur_state').unlink()
        self.assertEqual(self.sampler.sample()['state'],'unknown')

    def test_bad_temperature_is_not_zero(self):
        for value in ('invalid','-273000','9999999999'):
            self.write(self.zone,temp=value)
            s=self.sampler.sample();self.assertIsNone(s['zones'][0]['celsius'])
            self.assertEqual(s['state'],'unknown')

    def test_frequency_fallback_is_labelled_requested(self):
        (self.policy/'cpuinfo_cur_freq').unlink()
        p=self.sampler.sample()['policies'][0]
        self.assertEqual(p['current_mhz'],600); self.assertEqual(p['current_source'],'scaling_cur_freq')

    def test_cpu_utilization_delta_and_counter_reset(self):
        self.sampler.sample()
        self.write(self.proc,stat='cpu 150 0 0 950 0 0 0 0 0 0\n')
        self.assertEqual(self.sampler.sample()['cpu_busy_percent'],50)
        self.write(self.proc,stat='cpu 1 0 0 1 0 0 0 0 0 0\n')
        self.assertIsNone(self.sampler.sample()['cpu_busy_percent'])

    def test_missing_all_metrics_is_unknown(self):
        self.assertEqual(app.LinuxSampler(self.root/'absent', self.root/'absent').sample()['state'],'unknown')

    def test_stale_and_future_publication_never_claims_healthy(self):
        app.atomic_json(self.root/'system-health.json',dict(sampled_monotonic=10,state='no_limit_observed'))
        self.assertEqual(app.snapshot(self.root,now=17)['state'],'stale')
        self.assertEqual(app.snapshot(self.root,now=9)['state'],'unavailable')
        self.assertEqual(app.snapshot(self.root,now=12)['state'],'no_limit_observed')

    def test_summary_reads_only_ram_without_resampling_or_controller_rpc(self):
        app.atomic_json(self.root/'system-health.json',dict(sampled_monotonic=time.monotonic(),state='no_limit_observed'))
        with mock.patch.object(app.LinuxSampler,'sample',side_effect=AssertionError('HTTP must not sample')):
            for _ in range(10):self.assertEqual(app.snapshot(self.root)['state'],'no_limit_observed')

    def test_symlinks_and_oversized_reads_rejected(self):
        target=self.root/'target';target.write_text('{}')
        link=self.root/'link';link.symlink_to(target)
        with self.assertRaises(OSError):app.read_json(link)
        with mock.patch.object(app,'MAX_FILE',1):
            with self.assertRaises(ValueError):app.read_json(target)

    def monitor(self):
        return app.HealthMonitor(self.root,self.root/'logs',sampler=self.sampler)

    def test_context_whitelist_records_timing_and_drops_secrets(self):
        now=time.monotonic()
        packet=dict(published_at=now,snapshot=dict(token='private',motion=dict(phase='ready',
            hardware=dict(mode='walk',generation=2,inference_ms=12,timing=dict(cycle=dict(max_ms=29))),
            control_owner=dict(id='private'))))
        app.atomic_json(self.root/'web-state.json',packet)
        c=app.control_context(self.root/'web-state.json',now+.2)
        self.assertEqual(c['cycle_max_ms'],29);self.assertEqual(c['inference_ms'],12)
        self.assertTrue(c['available']);self.assertNotIn('private',json.dumps(c))
        self.assertFalse(app.control_context(self.root/'web-state.json',now+2)['available'])

    def test_publishes_peak_and_bounded_history_and_durable_log(self):
        m=self.monitor();m.history=app.deque(maxlen=2)
        m.tick();self.write(self.zone,temp='70000');m.tick();m.tick()
        self.assertEqual(app.snapshot(self.root)['peak_cpu_celsius'],70)
        self.assertEqual(len(app.read_json(self.root/'system-health-history.json')['samples']),2)
        self.assertEqual(len((self.root/'logs/samples.jsonl').read_text().splitlines()),1)
        self.write(self.cool,cur_state='1');m.tick()
        self.assertEqual(app.snapshot(self.root)['thermal_limited_samples'],1)
        self.assertIsNotNone(app.snapshot(self.root)['last_thermal_limited_utc'])
        self.assertEqual(len((self.root/'logs/samples.jsonl').read_text().splitlines()),2)

    def test_log_failure_does_not_stop_monitor_or_hide_temperature(self):
        m=self.monitor()
        with mock.patch.object(m,'append_log',side_effect=OSError('disk full')):m.tick()
        s=app.snapshot(self.root)
        self.assertEqual(s['state'],'no_limit_observed');self.assertIn('disk full',s['recording_error'])

    def test_logs_rotate_at_fixed_bound(self):
        m=self.monitor()
        with mock.patch.object(app,'LOG_LIMIT',1):
            for i in range(5):m.append_log(dict(i=i))
        self.assertEqual(sorted(p.name for p in (self.root/'logs').iterdir()),['samples.1.jsonl','samples.2.jsonl','samples.jsonl'])
        self.assertEqual(json.loads((self.root/'logs/samples.jsonl').read_text())['i'],4)

    def test_thread_survives_sampling_exception_and_stops(self):
        m=self.monitor()
        with mock.patch.object(m,'tick',side_effect=RuntimeError('sensor read failed')):
            m.start();time.sleep(.02);self.assertTrue(m.thread.is_alive());m.close()
        self.assertFalse(m.thread.is_alive())


if __name__=='__main__':unittest.main()
