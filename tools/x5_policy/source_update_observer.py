"""Read-only diagnostics around the original native PPO Adam updates.

No parameter groups, gradients, learning rates, update admission or samples are
modified. The disabled mode exists for an actual native-PPO equivalence check.
CPU update/RNG bitwise parity passed. GPU passive/off parity did not, while two
unobserved GPU runs matched exactly; the numerical cause is unproven. Keep this
observer disabled for the native-source training experiment.
"""
import copy
import math


class PassiveSourceObserver:
    def __init__(self,optimizer,measure,enabled=True):
        self.optimizer=optimizer;self.measure=measure;self.enabled=enabled
        self.rollouts=0;self.updates=0;self.errors=0
        self.maximum_committed_kl=0.;self.records=[]
        self.handle=optimizer.register_step_post_hook(self.after) if enabled else None

    def begin_rollout(self):
        self.rollouts+=1

    def after(self,optimizer,args,kwargs):
        self.updates+=1
        try:
            measured=self.measure()
        except (ValueError,FloatingPointError,RuntimeError) as error:
            self.errors+=1;measured={'diagnostic_error':repr(error)}
        kl=measured.get('mean_kl')
        if kl is not None and math.isfinite(kl):
            self.maximum_committed_kl=max(self.maximum_committed_kl,kl)
        if len(self.records)<100:
            self.records.append(dict(update=self.updates,rollout=self.rollouts,
                learning_rates=[g['lr'] for g in optimizer.param_groups],measurement=measured))

    def continuation_state(self):
        return dict(schema_version=1,kind='passive_native_PPO',enabled=self.enabled,
            rollouts=self.rollouts,updates=self.updates,errors=self.errors,
            maximum_committed_kl=self.maximum_committed_kl,records=copy.deepcopy(self.records))

    def restore_continuation_state(self,state):
        if (set(state)!=set(self.continuation_state()) or state['schema_version']!=1
                or state['kind']!='passive_native_PPO' or state['enabled']!=self.enabled):
            raise ValueError('Passive observer checkpoint context differs')
        for key in ('rollouts','updates','errors'):
            if type(state[key]) is not int or state[key]<0:raise ValueError('Invalid diagnostic count')
        for key in ('rollouts','updates','errors','maximum_committed_kl','records'):
            setattr(self,key,copy.deepcopy(state[key]))

    def summary(self):
        return dict(kind='passive_native_PPO',enabled=self.enabled,
            optimizer_or_parameter_groups_modified=False,gradient_or_learning_rate_modified=False,
            update_admission=False,random_sampling_modified=False,**{
                k:v for k,v in self.continuation_state().items() if k not in ('kind','enabled')})
