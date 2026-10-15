"""Pure math for motor autotune: multi-level step identification + SIMC PI tuning.

Plant per motor:  rpm = K*(pwm - pwm0), first-order lag tau, dead time L.
Firmware feedforward: pwm = target*F + sign(target)*pwm0, F = FF_SCALE/K.

Changes vs. previous version:
- K, pwm0: least-squares line through N open-loop steady states (not 2 points).
- tau, L: least-squares FOPDT curve fit per step, median over steps (robust to 1.2 rpm quantization).
- common_tau_c(): one shared closed-loop time constant so all motors respond alike.
- step_metrics: adds ripple_pp (steady-state chatter).
"""
import math


def _smooth(x, n=3):
    h = n // 2
    return [sum(x[max(0, i - h): i + h + 1]) / len(x[max(0, i - h): i + h + 1]) for i in range(len(x))]


def _steady(x, frac=0.3):
    tail = x[-max(3, int(len(x) * frac)):]
    return sum(tail) / len(tail)


def _crossing(x, level, dt):
    """First time (s) x reaches level, linearly interpolated. Sample i is at t=(i+1)*dt."""
    prev_t, prev_v = 0.0, 0.0
    for i, v in enumerate(x):
        t = (i + 1) * dt
        if v >= level:
            if v == prev_v:
                return t
            return prev_t + (level - prev_v) / (v - prev_v) * (t - prev_t)
        prev_t, prev_v = t, v
    raise ValueError("response never reached the expected level")


_TAUS = [0.03 * 1.06 ** i for i in range(75)]      # 0.03 .. ~2.4 s
_LS = [k * 0.0125 for k in range(33)]              # 0 .. 0.4 s


def _fit_fopdt(y, ss, dt):
    """Grid-search LS fit of y(t)=ss*(1-exp(-(t-L)/tau)) for t>L. Returns (tau, L, rms)."""
    ts = [(j + 1) * dt for j in range(len(y))]
    best = (float("inf"), dt, dt)
    for tau in _TAUS:
        for L in _LS:
            sse = 0.0
            for t, v in zip(ts, y):
                m = 0.0 if t <= L else ss * (1.0 - math.exp(-(t - L) / tau))
                sse += (v - m) ** 2
            if sse < best[0]:
                best = (sse, tau, L)
    sse, tau, L = best
    return tau, max(L, dt), math.sqrt(sse / len(y))


def identify(levels, rpms, dt):
    """levels: PWM of each open-loop step from rest. rpms: list of RPM sample lists (every dt s)."""
    steady = [_steady(r) for r in rpms]
    pts = [(p, s) for p, s in zip(levels, steady) if s >= 3.0]
    if len(pts) < 2:
        raise ValueError(f"motor barely moves (steady rpm {[round(s, 1) for s in steady]}): raise TEST_PWMS")
    n = len(pts)
    mx = sum(p for p, _ in pts) / n
    my = sum(s for _, s in pts) / n
    K = sum((p - mx) * (s - my) for p, s in pts) / sum((p - mx) ** 2 for p, _ in pts)
    if K <= 0:
        raise ValueError(f"non-positive gain K={K:.3f}: check encoder polarity / PWM levels")
    pwm0 = mx - my / K
    lin_err = max(abs(s - K * (p - pwm0)) for p, s in pts)

    fits = [_fit_fopdt(r, s, dt) for r, s in zip(rpms, steady) if s >= 10.0]
    if not fits:
        i = max(range(len(steady)), key=lambda k: steady[k])
        fits = [_fit_fopdt(rpms[i], steady[i], dt)]
    taus = sorted(f[0] for f in fits)
    Ls = sorted(f[1] for f in fits)
    return {"K": K, "pwm0": pwm0, "tau": taus[len(taus) // 2], "L": Ls[len(Ls) // 2],
            "ss_max": max(steady), "lin_err_rpm": lin_err, "fit_rms": max(f[2] for f in fits)}


def common_tau_c(plants, tau_c_factor=0.5, l_mult=2.0):
    """Shared closed-loop time constant: slowest motor sets it, so all motors respond alike.
    tau_c = max_m max(l_mult*L_m, tau_c_factor*tau_m). l_mult >= 1 (tau_c < L is never allowed)."""
    return max(max(max(l_mult, 1.0) * p["L"], tau_c_factor * p["tau"]) for p in plants)


def simc_pi(K, tau, L, tau_c_factor=0.5, kp_max=20.0, ki_max=60.0, tau_c=None):
    """SIMC PI for FOPDT. Returns (Kp [PWM/rpm], Ki [PWM/(rpm*s)])."""
    if tau_c is None:
        tau_c = max(2.0 * L, tau_c_factor * tau)
    else:
        tau_c = max(tau_c, L)
    kp = tau / (K * (tau_c + L))
    ti = min(tau, 4.0 * (tau_c + L))
    return min(kp, kp_max), min(kp / ti, ki_max)


def step_metrics(rpm, target, dt):
    """Closed-loop step 0 -> target metrics. ripple_pp: peak-to-peak of the last 30% (raw)."""
    y = _smooth(rpm)
    try:
        rise = _crossing(y, 0.9 * target, dt)
    except ValueError:
        rise = None
    band = max(0.05 * target, 1.3)
    last_bad = 0
    for i, v in enumerate(y):
        if abs(v - target) > band:
            last_bad = i + 1
    tail = rpm[-max(3, int(len(rpm) * 0.3)):]
    return {
        "overshoot_pct": max(0.0, (max(y) - target) / target * 100.0),
        "rise90_s": rise,
        "settle_s": last_bad * dt,
        "final_rpm": _steady(rpm),
        "ripple_pp": max(tail) - min(tail),
    }