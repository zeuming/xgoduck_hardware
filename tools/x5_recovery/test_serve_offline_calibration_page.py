import hashlib
import io
import json
import os
from pathlib import Path
import tempfile
import types
import unittest
from unittest import mock
import serve_offline_calibration_page as preview


class Tests(unittest.TestCase):
    def calibration(self):
        return {'version':1,'mechanical_reference_confirmed':True,
                'joints':[{'id':sid,'zero':1900+i,'min':None,'max':None,'sign':-1,'direction_confirmed':False}
                          for i,sid in enumerate([10,11,12,13,14,20,21,22,23,24,30,31,32,33,34])],
                'imu':{'axes':['x','y','z'],'axes_confirmed':False,'calibrated':False,'bias_dps':[0.,0.,0.]}}

    def test_snapshot_preserves_15_zeros_but_never_claims_current_feedback_or_release(self):
        cfg=self.calibration()
        stop={'reason':'guard heartbeat stopped','serial_shutdown_proof':{'release_confirmed':True,'serial_closed':True,'worker_stopped':True}}
        before=json.dumps(cfg,sort_keys=True)
        result=preview.offline_snapshot(cfg,stop,'USB offline')
        self.assertEqual(before,json.dumps(cfg,sort_keys=True))
        self.assertEqual([j['zero'] for j in cfg['joints']],[j['zero'] for j in result['joints']])
        for field in ('connected','motor_output','pending_stop','imu_feedback_ok','pause_release_confirmed','live_feedback_available'):
            self.assertFalse(result[field])
        self.assertTrue(result['read_only']);self.assertTrue(result['paused']);self.assertEqual('off',result['mode'])
        self.assertIsNone(result['age_ms']);self.assertIsNone(result['imu']['acc_body']);self.assertEqual([],result['rawdata'])
        self.assertTrue(result['historical_stop_proof']['release_confirmed'])
        self.assertFalse(result['historical_stop_proof_is_live_feedback'])
        self.assertEqual('idle',result['imu_wizard']['stage']);self.assertFalse(result['imu_wizard']['busy'])

    def test_handler_get_host_gate_and_all_posts_are_rejected_without_reading_body(self):
        contents={'/':(b'page','text/html')}
        cls=preview.make_handler(contents)
        obj=object.__new__(cls);obj.server=types.SimpleNamespace(server_address=('127.0.0.1',8767));obj.reply=mock.Mock()
        obj.headers={'Host':'external.example'};obj.path='/';obj.do_GET();self.assertEqual(403,obj.reply.call_args.args[0])
        for host in ('127.0.0.1:8767','localhost:8767'):
            obj.headers={'Host':host};obj.do_GET();self.assertEqual((200,b'page','text/html'),obj.reply.call_args.args)
        obj.path='/missing';obj.do_GET();self.assertEqual(404,obj.reply.call_args.args[0])
        obj.rfile=mock.Mock()
        for path in ('/api/motion/capture-all-zeros','/api/motion/imu-wizard','/api/motion/start','/api/infer','/api/benchmark','/anything'):
            obj.path=path;obj.do_POST();self.assertEqual(409,obj.reply.call_args.args[0])
        obj.rfile.read.assert_not_called();self.assertEqual({'/':(b'page','text/html')},contents)

    def test_reply_sets_no_store_and_security_headers(self):
        cls=preview.make_handler({});obj=object.__new__(cls)
        obj.send_response=mock.Mock();obj.send_header=mock.Mock();obj.end_headers=mock.Mock();obj.wfile=io.BytesIO()
        obj.reply(409,{'read_only':True})
        headers=dict(call.args for call in obj.send_header.call_args_list)
        self.assertEqual('no-store',headers['Cache-Control']);self.assertEqual('nosniff',headers['X-Content-Type-Options'])
        self.assertEqual({'read_only':True},json.loads(obj.wfile.getvalue()))

    def test_inputs_are_sha_bound_private_and_build_is_readonly(self):
        with tempfile.TemporaryDirectory() as directory:
            d=Path(directory)
            data={'page.html':b'<html><div id="imuWizardPanel"></div><input id="imuMountedConfirmed" type="checkbox"></body></html>',
                  'cal.json':json.dumps(self.calibration()).encode(),'stop.json':json.dumps({'reason':'offline'}).encode()}
            for name,value in data.items():(d/name).write_bytes(value);(d/name).chmod(0o600)
            args=types.SimpleNamespace(new_html=d/'page.html',expected_html_sha256=hashlib.sha256(data['page.html']).hexdigest(),calibration=d/'cal.json',stop_report=d/'stop.json',reason='USB offline')
            before={p.name:(p.read_bytes(),p.stat().st_mtime_ns) for p in d.iterdir()}
            contents=preview.build_preview(args)
            status=json.loads(contents['/api/status'][0]);self.assertFalse(status['online']);self.assertIsNone(status['token'])
            self.assertEqual(4,len(status['models']));self.assertTrue(all(m['ready'] is False for m in status['models']))
            self.assertIn('本机只读预览'.encode(),contents['/'][0]);self.assertIn(b'state.token=null',contents['/'][0])
            self.assertEqual(before,{p.name:(p.read_bytes(),p.stat().st_mtime_ns) for p in d.iterdir()})
            args.expected_html_sha256='0'*64
            with self.assertRaises(ValueError):preview.build_preview(args)
            (d/'page.html').chmod(0o644)
            with self.assertRaises(ValueError):preview.read_input(d/'page.html',262144)


if __name__=='__main__':unittest.main(verbosity=2)
