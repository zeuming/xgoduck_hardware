"""Exact joint-speed bound for sums of cubic ramps, used only offline."""
import numpy as np


def maximum_speed(pulses):
    """Return exact per-joint sup |dq/dt| of sum(delta*smoothstep).

    pulses is [(start_s, duration_s, delta_rad[J]), ...]. On each interval
    between ramp knots velocity is quadratic; its extrema are endpoints or
    its single stationary point. No sampling estimate is used for retiming.
    """
    if not pulses:raise ValueError('Need at least one ramp')
    n=len(np.asarray(pulses[0][2]));knots=[]
    for start,duration,delta in pulses:
        if not np.isfinite(start) or not np.isfinite(duration) or duration<=0 or start<0:
            raise ValueError('Finite positive duration and nonnegative start required')
        if np.shape(delta)!=(n,) or not np.isfinite(delta).all():raise ValueError('Invalid joint vector')
        knots.extend((start,start+duration))
    knots=sorted(set(knots));maximum=np.zeros(n)
    for lo,hi in zip(knots[:-1],knots[1:]):
        # velocity(lo+u) = a*u^2+b*u+c in this knot interval.
        a=np.zeros(n);b=np.zeros(n);c=np.zeros(n);mid=(lo+hi)/2
        for start,duration,delta in pulses:
            if start<mid<start+duration:
                z=(lo-start)/duration;delta=np.asarray(delta)
                a+=-6*delta/duration**3
                b+=6*(1-2*z)*delta/duration**2
                c+=6*z*(1-z)*delta/duration
        width=hi-lo;maximum=np.maximum(maximum,np.maximum(abs(c),abs(a*width**2+b*width+c)))
        nonzero=abs(a)>1e-18
        u=np.divide(-b,2*a,out=np.zeros(n),where=nonzero)
        valid=nonzero&(u>0)&(u<width)
        maximum=np.maximum(maximum,np.where(valid,abs(a*u*u+b*u+c),0.))
    return maximum


def evaluate(pulses,t):
    out=np.zeros_like(np.asarray(pulses[0][2]),dtype=float)
    for start,duration,delta in pulses:
        z=np.clip((t-start)/duration,0.,1.)
        out+=np.asarray(delta)*(z*z*(3-2*z))
    return out


def retime(pulses,limit):
    if not np.isfinite(limit) or limit<=0:raise ValueError('Positive finite speed required')
    before=maximum_speed(pulses);scale=max(1.,float(before.max())/limit*(1+1e-10))
    adjusted=[(start*scale,duration*scale,np.asarray(delta)) for start,duration,delta in pulses]
    after=maximum_speed(adjusted)
    if after.max()>limit+1e-9:raise ValueError('Analytic joint-rate bound violated')
    return adjusted,dict(time_stretch=scale,maximum_speed_before_rad_s=before.tolist(),maximum_speed_after_rad_s=after.tolist())
