"""Audit cross-engine model fields without modifying either original plant."""
import dataclasses
import json
import sys
from pathlib import Path
import numpy as np


def main():
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    from native_bam_prediction_branch import NativeBamPredictionBranch
    import validate_native_bam_prediction as run
    out=Path(sys.argv[sys.argv.index('--output')+1]);audit={}
    old=NativeBamPredictionBranch.__init__
    fields=('body_pos','body_quat','body_mass','body_ipos','body_iquat','body_inertia','body_gravcomp',
        'jnt_pos','jnt_axis','jnt_range','jnt_limited','jnt_stiffness','jnt_solref','jnt_solimp',
        'dof_armature','dof_damping','dof_frictionloss','dof_solref','dof_solimp',
        'geom_type','geom_contype','geom_conaffinity','geom_condim','geom_size','geom_pos','geom_quat',
        'geom_friction','geom_solmix','geom_solref','geom_solimp','geom_margin','geom_gap',
        'pair_dim','pair_friction','pair_solref','pair_solimp','pair_margin','pair_gap',
        'actuator_gear','actuator_gainprm','actuator_biasprm','actuator_forcerange','actuator_ctrlrange')
    def audit_init(self,env):
        old(self,env)
        model={};options={}
        for name in fields:
            if not hasattr(self.model,name) or not hasattr(self.native_model,name):continue
            a=getattr(self.model,name).numpy();b=np.asarray(getattr(self.native_model,name))
            if a.shape!=b.shape and a.shape[1:]==b.shape:a=a[0]
            if a.shape!=b.shape:
                model[name]=dict(shape_mismatch=True,warp_shape=a.shape,native_shape=b.shape);continue
            difference=float(abs(a.astype(float)-b.astype(float)).max(initial=0))
            model[name]=dict(identical_at_warp_precision=bool(np.array_equal(a,b.astype(a.dtype))),maximum_absolute_difference=difference)
        numerical=('iterations','ls_iterations','tolerance','ls_tolerance')
        for field in dataclasses.fields(self.model.opt):
            name=field.name
            if not hasattr(self.native_model.opt,name):continue
            a=getattr(self.model.opt,name);b=getattr(self.native_model.opt,name)
            try:
                a=np.asarray(a.numpy() if hasattr(a,'numpy') else a);b=np.asarray(b)
                if a.size!=b.size:continue
                b=b.reshape(a.shape)
                options[name]=dict(allowed_numerical_difference=name in numerical,
                    identical_at_warp_precision=bool(np.array_equal(a,b.astype(a.dtype))),
                    warp=a.tolist(),native=b.tolist())
            except (TypeError,ValueError):continue
        audit.update(model_fields=model,options=options,
            native_noslip_iterations=int(self.native_model.opt.noslip_iterations),
            changed_physical_fields=[k for k,v in model.items() if not v.get('identical_at_warp_precision',False)],
            changed_non_numerical_options=[k for k,v in options.items() if not v['allowed_numerical_difference'] and not v['identical_at_warp_precision']],
            cross_engine_contact_implementation_not_asserted_identical=True)
        (out/'physics-field-audit.json').write_text(json.dumps(audit,indent=2)+'\n')
    NativeBamPredictionBranch.__init__=audit_init
    try:run.main()
    finally:NativeBamPredictionBranch.__init__=old
    (out/'physics-audit-source.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps(audit,indent=2))


if __name__=='__main__':main()
