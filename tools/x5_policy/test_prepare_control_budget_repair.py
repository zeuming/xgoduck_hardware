"""Differential tests execute the inspected baseline and prepared decoder AST."""
import ast
import copy
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import threading
import types
import unittest

import numpy as np

from prepare_control_budget_repair import BASE, prepare


ROOT=Path(__file__).resolve().parents[2]
SOURCE=ROOT/'output/duck_sim/adaptation-20261011/candidate'
METHODS=('_decode_policy_action','_fixed_stance_locked','_record_policy_target_locked')


def load_file(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec)
    import sys
    sys.modules[name]=module
    spec.loader.exec_module(module)
    return module


def decoder_type(source,text):
    """Extract real methods without importing controller/device construction."""
    core=load_file('budget_core',source/'policy/rl_core.py')
    mathmod=load_file('budget_math',source/'policy/candidate_math.py')
    snapshots=load_file('budget_snapshots',source/'policy/status_snapshot.py')
    production=ast.parse((source/'policy/production_motion.py').read_text())
    parent=next(x for x in production.body if isinstance(x,ast.ClassDef) and x.name=='MotionController')
    record=next(x for x in parent.body if isinstance(x,ast.FunctionDef) and x.name=='_record_policy_target_locked')
    parent=ast.ClassDef(name='MotionController',bases=[],keywords=[],body=[record],decorator_list=[])
    module=ast.parse(text)
    cls=next(x for x in module.body if isinstance(x,ast.ClassDef) and x.name=='ResearchMotionController')
    cls.body=[x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name in METHODS]
    nodes=[parent]
    nodes.extend(x for x in module.body if isinstance(x,(ast.Assign,ast.FunctionDef,ast.ClassDef)) and (
        isinstance(x,ast.Assign) and any(isinstance(t,ast.Name) and t.id in ('ORIGINAL_HOME','POLICY_IDS') for t in x.targets)
        or isinstance(x,(ast.FunctionDef,ast.ClassDef)) and x.name in (
            '_clone_candidate_tree','_diagnostic_angles','CandidateTarget','ResearchMotionController')))
    env=dict(np=np,math=math,production_core=core,MOTOR_IDS=core.MOTOR_IDS,
        ACTION_ALPHA=core.ACTION_ALPHA,ACTION_ALPHA_HEAD=core.ACTION_ALPHA_HEAD,
        action_to_deg=core.action_to_deg,clone_display_value=snapshots.clone_display_value,
        ExponentialRollMap=mathmod.ExponentialRollMap,MotionError=ValueError)
    exec(compile(ast.fix_missing_locations(ast.Module(body=nodes,type_ignores=[])),
                 '<inspected-candidate-methods>','exec'),env)
    return env


def instance(env,calibration):
    controller=env['ResearchMotionController']()
    home=env['ORIGINAL_HOME'].copy()
    for sid,offset in ((12,-1),(14,-1),(22,1),(24,1)):
        home[env['POLICY_IDS'].index(sid)]+=math.radians(offset)
    home.setflags(write=False)
    controller._policy_profiles={'walk':dict(home_rad=home,scale=1.)}
    controller._lock=threading.RLock()
    controller._config=copy.deepcopy(calibration)
    controller._generation=17
    controller._hold_head_deg={30:20.,31:20.,32:0.,33:0.,34:0.}
    controller._stand_deg=env['production_core'].stance_from_home(home)
    return controller


class BudgetRepairTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory()
        cls.destination=Path(cls.tmp.name)/'patch'
        cls.manifest=prepare(SOURCE,cls.destination)
        cls.before=(SOURCE/'policy/candidate_controller.py').read_text()
        cls.after=(cls.destination/'policy/candidate_controller.py').read_text()
        cls.original=decoder_type(SOURCE,cls.before)
        cls.candidate=decoder_type(SOURCE,cls.after)
        cls.calibration=json.loads((SOURCE/'policy/calibration.json').read_text())

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def pair(self):
        return (instance(self.original,self.calibration),instance(self.candidate,self.calibration))

    def assert_decode_equal(self,controllers,raw,previous,smooth):
        first,second=(c._decode_policy_action('walk',raw,previous,smooth,.5,.7) for c in controllers)
        self.assertEqual(first[0],second[0])
        self.assertEqual(first[0].diagnostics,second[0].diagnostics)
        np.testing.assert_array_equal(first[1],second[1])
        np.testing.assert_array_equal(first[2],second[2])
        self.assertEqual(first[3],second[3])
        for c,result in zip(controllers,(first,second)):
            c._record_policy_target_locked(result[0])
        self.assertEqual(controllers[0]._candidate_last_inference,controllers[1]._candidate_last_inference)
        self.assertEqual(controllers[0]._last_policy_target_rejections,controllers[1]._last_policy_target_rejections)
        return first,second

    def test_1000_sequential_decodes_preserve_target_history_diagnostics_and_range_rejections(self):
        rng=np.random.default_rng(581)
        controllers=self.pair()
        previous=smooth=np.zeros(14,np.float32)
        for i in range(1000):
            raw=rng.normal(0,3,(1,14)).astype(np.float32)
            results=self.assert_decode_equal(controllers,raw,previous,smooth)
            previous,smooth=results[0][1:3]
        self.assertEqual(controllers[0]._fixed_stance_locked(),controllers[1]._fixed_stance_locked())

    def test_calibration_changes_are_observed_on_the_next_decode_without_a_cache(self):
        controllers=self.pair()
        raw=np.ones((1,14),np.float32)*2
        z=np.zeros(14,np.float32)
        first=self.assert_decode_equal(controllers,raw,z,z)[0]
        for controller in controllers:
            for row in controller._config['joints']:
                if row['id'] in (11,21):row['min']-=70;row['max']+=70
        updated=self.assert_decode_equal(controllers,raw,z,z)[0]
        self.assertNotEqual(first[0],updated[0])

    def test_mutation_of_returned_target_or_diagnostics_does_not_mutate_saved_record(self):
        controllers=self.pair();z=np.zeros(14,np.float32)
        results=self.assert_decode_equal(controllers,np.ones((1,14),np.float32),z,z)
        for c,result in zip(controllers,results):
            expected=copy.deepcopy(c._candidate_last_inference)
            result[0].diagnostics['adapted_raw_action'][0]=12345
            result[0].diagnostics['roll_decoder']['11']['saved_raw_bounds'][0]=12345
            result[0][0]=12345
            self.assertEqual(c._candidate_last_inference,expected)
            stance=c._fixed_stance_locked();stance[0]=12345
            self.assertNotEqual(c._stand_deg[0],12345)

    def test_flat_copy_fast_path_still_detaches_nested_and_non_json_values(self):
        # Unknown future values retain clone_display_value's deepcopy fallback.
        original=self.original['clone_display_value']
        optimized=self.candidate['_clone_candidate_tree']
        value={'flat':[None,False,1,-0.0,'x',b'y'],
               'nested':[[1,2],{'leaf':[3,4]}],
               'tuple':([5,6],), 'array':np.array([7,8])}
        a,b=original(value),optimized(value)
        np.testing.assert_array_equal(a.pop('array'),b.pop('array'))
        self.assertEqual(a,b)
        b['nested'][0][0]=99;b['tuple'][0][0]=99
        self.assertEqual(value['nested'][0][0],1)
        self.assertEqual(value['tuple'][0][0],5)

    def test_bad_actions_profiles_and_roll_limits_keep_error_type_and_message(self):
        changes=[lambda c:None,
            lambda c:c._policy_profiles['walk'].update(scale=0),
            lambda c:c._policy_profiles['walk'].update(scale=float('nan')),
            lambda c:c._policy_profiles['walk'].update(home_rad=np.zeros(13)),
            lambda c:c._policy_profiles['walk'].update(home_rad=np.ones(14)*float('nan')),
            lambda c:next(r for r in c._config['joints'] if r['id']==11).update(min=3470,max=3480)]
        raw_cases=[np.zeros((1,14),np.float32),np.zeros((1,13),np.float32),
                   np.ones((1,14),np.float32)*float('nan'),np.ones((1,14),np.float32)*float('inf')]
        z=np.zeros(14,np.float32)
        for change in changes:
            for raw in raw_cases:
                outcome=[]
                for c in self.pair():
                    change(c)
                    try:
                        with np.errstate(all='ignore'):c._decode_policy_action('walk',raw,z,z,.5,.7)
                        outcome.append(None)
                    except Exception as e:outcome.append((type(e).__name__,str(e)))
                self.assertEqual(*outcome)

    def test_every_other_method_including_all_gates_and_phase_timers_is_byte_ast_equivalent(self):
        def methods(text):
            cls=next(n for n in ast.parse(text).body if isinstance(n,ast.ClassDef) and n.name=='ResearchMotionController')
            return {n.name:ast.dump(n) for n in cls.body if isinstance(n,ast.FunctionDef) and n.name not in METHODS}
        self.assertEqual(methods(self.before),methods(self.after))
        self.assertEqual(set(self.manifest['after']),{'candidate_controller.py'})

    def test_multiple_bad_roll_rows_keep_the_original_first_error(self):
        results=[];z=np.zeros(14,np.float32)
        for c in self.pair():
            by_id={r['id']:r for r in c._config['joints']}
            by_id[11].update(min=3470,max=3480)
            del by_id[21]['min']
            try:c._decode_policy_action('walk',np.zeros((1,14),np.float32),z,z,0.,.7)
            except Exception as error:results.append((type(error).__name__,str(error)))
        self.assertEqual(len(results),2)
        self.assertEqual(*results)
        self.assertIn('cannot fit unchanged saved bounds',results[0][1])

    def test_source_pin_rejects_an_unreviewed_baseline_before_writing(self):
        bad=Path(self.tmp.name)/'bad';(bad/'policy').mkdir(parents=True)
        for name in BASE:(bad/'policy'/name).write_bytes((SOURCE/'policy'/name).read_bytes())
        (bad/'policy/candidate_controller.py').write_text(self.before+'\n# changed\n')
        dest=Path(self.tmp.name)/'rejected'
        with self.assertRaisesRegex(ValueError,'Baseline differs'):prepare(bad,dest)
        self.assertFalse(dest.exists())


if __name__=='__main__':
    unittest.main()
