"""Full source-state/host STOP replay, memory transport only."""
import ast, json, struct, unittest
from pathlib import Path

fixture_path=Path(__file__).with_name('test_policy_evidence.py')
tree=ast.parse(fixture_path.read_text());nodes=[]
for node in tree.body:
    if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='result' for t in node.targets):break
    nodes.append(node)
ns={'__file__':str(fixture_path),'__name__':'session_stop_fixture'}
exec(compile(ast.Module(body=nodes,type_ignores=[]),str(fixture_path),'exec'),ns)
motion=ns['motion'];args=ns['a'];Base=ns['EvidenceTests']

def stop_packet(cookie=123,cause=6):
    return motion.TARGET_STOP_PACKET.pack(motion.TARGET,2,1,cause,4,1,
        motion.ALL_MASK,0,0,123456,300000,1000,250001,0,255,0,0,0,0)+struct.pack('<I',cookie)

class SessionStopTests(Base):
    def fault_state(self,cause=6,cookie=None):
        packet=bytearray(self.t.request(motion.SOURCE_STATE))
        packet[14]=0;struct.pack_into('<I',packet,100,0);packet[104]=4
        struct.pack_into('<H',packet,106,motion.ALL_MASK);packet[109]=2
        return bytes(packet)+stop_packet(self.c._cookie if cookie is None else cookie,cause)

    def inject(self,packet):
        original=self.t.request
        self.t.request=lambda kind,payload=b'': packet if kind==motion.SOURCE_STATE else original(kind,payload)
        return self.c._read_state()

    def test_actual_cpp_packet_cross_language(self):
        path=args.output.parent/'firmware-stopped-state.bin'
        state=self.inject(path.read_bytes())
        self.assertEqual(state['firmware_first_stop']['cause'],6)
        self.assertEqual(state['firmware_first_stop']['arm_cookie'],123)
        self.assertGreater(state['firmware_first_stop']['target_age_us'],250000)

    def test_first_stop_survives_host_stop_and_trial_capture(self):
        self.step();packet=self.fault_state();state=self.inject(packet)
        self.c._accept_state_locked(state)
        expected=self.c._session_failure_locked(state)
        self.assertIn('目标指令间隔超过 250 毫秒',expected)
        before=sum(k==motion.SOURCE_TARGET for k,_ in self.t.requests)
        self.c._cycle_once()
        self.assertEqual(self.c._mode,'off')
        self.assertEqual(before,sum(k==motion.SOURCE_TARGET for k,_ in self.t.requests))
        trial=self.c.completed_trial_traces()[-1]
        self.assertEqual(trial['terminal_feedback']['firmware_first_stop']['cause'],6)
        self.assertEqual(trial['terminal_feedback']['last_error'],4)
        self.assertTrue(any(k==motion.STOP for k,_ in self.t.requests))

    def test_mismatched_cookie_not_attributed_to_current_arm(self):
        state=self.inject(self.fault_state(cookie=(self.c._cookie^1) or 1))
        text=self.c._session_failure_locked(state)
        self.assertNotIn('最先停机原因',text)
        self.assertIn('其他使能会话',text)

    def test_legacy_state_keeps_raw_error_without_invented_first_cause(self):
        packet=bytearray(self.t.request(motion.SOURCE_STATE));packet[14]=0;packet[104]=4
        state=self.inject(bytes(packet))
        self.assertNotIn('firmware_first_stop',state)
        self.assertIn('固件错误码 4',self.c._session_failure_locked(state))

    def test_armed_extension_rejected_without_target(self):
        packet=bytearray(self.fault_state());packet[14]=1
        with self.assertRaisesRegex(motion.MotionError,'运行中的状态'):
            self.inject(bytes(packet))

    def test_malformed_extension_rejected(self):
        packet=bytearray(self.fault_state());packet[173]=255
        with self.assertRaises(motion.MotionError):self.inject(bytes(packet))
        with self.assertRaises(motion.MotionError):self.inject(self.fault_state()[:-1])

    def test_imu_cause_distinct_from_target_watchdog(self):
        state=self.inject(self.fault_state(cause=3))
        text=self.c._session_failure_locked(state)
        self.assertIn('IMU 读取失败',text)
        self.assertNotIn('最先停机原因：X5',text)

    def test_cookie_and_configuration_failures_distinguished(self):
        state=self.c._state.copy();state['cookie']^=1
        self.assertIn('标识不匹配',self.c._session_failure_locked(state))
        state['configured']=False
        self.assertIn('校准配置失效',self.c._session_failure_locked(state))

    def test_cached_stop_summary_keeps_first_error_before_stop_clears_it(self):
        import sys
        sys.path.insert(0,str(args.candidate/'service'))
        import native_runtime
        state=self.inject(self.fault_state());self.c._accept_state_locked(state)
        before=self.c.status()
        compact=native_runtime._compact_diagnostic(dict(reason='stop',before=before))
        self.assertEqual(compact['before']['firmware_error'],4)
        self.assertEqual(compact['before']['firmware_first_stop']['cause'],6)
        state['last_error']=0;state['firmware_first_stop']['cause']=3
        self.assertEqual(compact['before']['firmware_error'],4)
        self.assertEqual(compact['before']['firmware_first_stop']['cause'],6)

result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(SessionStopTests))
args.output.write_text(json.dumps(dict(tests=result.testsRun,errors=len(result.errors),failures=len(result.failures),
    passed=result.wasSuccessful(),hardware_access=False,deployment=False),indent=2)+'\n')
raise SystemExit(not result.wasSuccessful())
