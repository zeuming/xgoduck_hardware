"""New offline stateful policy decoder; never a hardware execution clamp.

Identity model-angle center [-T,T]; C1 monotone exponential tails approach
saved limits with a3tick interior margin. Original NN raw history is cached.
No sensors, gravity, qrel, qvel, calibration or other actions are transformed.
"""
from __future__ import annotations
import math
import numpy as np


class ExponentialRollMap:
    def __init__(self, lower_deg, upper_deg, threshold_deg):
        self.lower,self.upper,self.threshold=map(float,(lower_deg,upper_deg,threshold_deg))
        if not all(math.isfinite(x) for x in (self.lower,self.upper,self.threshold)) or not self.lower < -self.threshold < self.threshold < self.upper:
            raise ValueError('Require finite lower<-threshold<threshold<upper')

    def __call__(self, value):
        q=float(value)
        if not math.isfinite(q): raise ValueError('Nonfinite original target')
        t=self.threshold
        if q < -t:
            width=-t-self.lower
            return self.lower+width*math.exp((q+t)/width)
        if q > t:
            width=self.upper-t
            return self.upper-width*math.exp(-(q-t)/width)
        return q

    def derivative(self,value):
        q=float(value)
        if not math.isfinite(q): raise ValueError('Nonfinite original target')
        t=self.threshold
        if q < -t: return math.exp((q+t)/(-t-self.lower))
        if q > t: return math.exp(-(q-t)/(self.upper-t))
        return 1.


class BoundedRollActor:
    """Stateful adapter rawNN→s95roll→nonlinear target→adaptedraw→hostEMA.

    Original NN previous14raw is explicitly cached and reset per episode;
    no nonlinear EMA commutation or affine inverse is assumed. This wrapper
    cannot replace a stateless ONNX without explicit external history state.
    """
    ids=(11,21)
    indices=(1,10)

    def __init__(self,session,home_rad,scale,calibration,threshold_deg,margin_ticks=3,roll_scale=.95):
        self.session=session
        self.home=np.asarray(home_rad,dtype=float)
        self.scale=float(scale)
        self.roll_scale=float(roll_scale)
        if self.home.shape!=(14,) or not np.isfinite(self.home).all() or not math.isfinite(self.scale) or self.scale<=0:
            raise ValueError('Invalid HOME/action scale')
        if type(margin_ticks) is not int or margin_ticks<1: raise ValueError('Positive integer interior margin required')
        if not math.isfinite(self.roll_scale) or not 0<self.roll_scale<=1: raise ValueError('Invalid roll scale')
        self.maps={}
        for sid in self.ids:
            row=calibration[sid]
            raw=(row['min']+margin_ticks,row['max']-margin_ticks)
            lo,hi=sorted((value-row['zero'])*360/4095*row['sign'] for value in raw)
            self.maps[sid]=ExponentialRollMap(lo,hi,threshold_deg)
        for sid,index in zip(self.ids,self.indices):
            if self.maps[sid](math.degrees(self.home[index]))!=math.degrees(self.home[index]):
                raise ValueError('HOME must lie in identity center')
        self.reset()

    def reset(self):
        self.previous_raw=np.zeros(14,np.float32)
        self.raw_actions=[]
        self.raw_previous_actions=[]
        self.nominal_roll_targets=[]
        self.adapted_roll_targets=[]

    def get_inputs(self): return self.session.get_inputs()

    def adapt_observation(self,obs):
        value=np.asarray(obs,np.float32).copy()
        if value.shape!=(1,61) or not np.isfinite(value).all(): raise ValueError('Invalid observation')
        value[:,34:48]=self.previous_raw
        return value

    def run(self,output_names,inputs):
        name=self.get_inputs()[0].name
        observation=self.adapt_observation(inputs[name])
        outputs=self.session.run(output_names,{**inputs,name:observation})
        raw=np.asarray(outputs[0],np.float32)
        if raw.shape!=(1,14) or not np.isfinite(raw).all(): raise ValueError('Bad original NN output')
        self.raw_previous_actions.append(self.previous_raw.tolist())
        self.previous_raw=raw[0].copy()
        self.raw_actions.append(self.previous_raw.tolist())
        adapted=raw.copy()
        nominal,decoded=[],[]
        for sid,index in zip(self.ids,self.indices):
            q=math.degrees(self.home[index]+self.scale*self.roll_scale*float(raw[0,index]))
            bounded=self.maps[sid](q)
            adapted[0,index]=(math.radians(bounded)-self.home[index])/self.scale
            nominal.append(q)
            decoded.append(bounded)
        self.nominal_roll_targets.append(nominal)
        self.adapted_roll_targets.append(decoded)
        return [adapted,*outputs[1:]]
