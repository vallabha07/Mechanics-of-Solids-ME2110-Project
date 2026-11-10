"""mos.py - Mechanics of Solids subsurface Cauchy stress and failure evaluation."""
import math
from .materials import OBJECTS, PADS, e_star, hertz_a, hertz_delta, hertz_p0, r_eff

ZETA = 0.48

def axis_stresses(p0, nu, zeta=ZETA):
    term = (1.0 + nu) * (1.0 - zeta * math.atan(1.0 / zeta)) - 0.5 / (1.0 + zeta ** 2)
    sig_r = -p0 * term
    sig_z = -p0 / (1.0 + zeta ** 2)
    return sig_r, sig_r, sig_z

def mos_state(N, fy, obj_key, pad_key='small'):
    o, p = OBJECTS[obj_key], PADS[pad_key]
    Es = e_star(p['E'], p['nu'], o['E'], o['nu'])
    Re = r_eff(p['R'], o['R'])
    K = 4.0 / 3.0 * Es * math.sqrt(Re)
    nu, mu = o['nu'], o['mu']
    sig_y_allow = (0.62 * o['p_allow']) if not math.isinf(o['p_allow']) else None

    out = dict(Es=Es, Re=Re, K=K, N=N, fy=fy, mode=o.get('mode', 'Von Mises'))
    if N <= 1e-6:
        out.update(delta=0.0, a=0.0, eps_c=0.0, p0=0.0, p_mean=0.0, tau_mean=0.0, tau_edge=0.0, c_over_a=1.0,
                   sig1=0.0, sig2=0.0, sig3=0.0, tau_sub=0.0, sig_tresca=0.0, sig_vm=0.0, sig_rankine=0.0,
                   sig_y_allow=sig_y_allow, fos=99.9, fos_basis='-')
        return out

    a = hertz_a(N, Re, Es)
    area = math.pi * a * a
    p0 = hertz_p0(N, Re, Es)
    q = min(1.0, abs(fy) / max(mu * N, 1e-9))
    c_over_a = (1.0 - q) ** (1.0 / 3.0)
    mu_mob = min(mu, abs(fy) / N)

    sr, st, sz = axis_stresses(p0, nu)
    tau_axis = 0.5 * abs(sr - sz)
    tau_edge = mu_mob * p0
    sig_tresca = 2.0 * math.sqrt(tau_axis ** 2 + tau_edge ** 2)
    sig_vm = math.sqrt((sr - sz) ** 2 + 3.0 * tau_edge ** 2)
    sig_rankine = (1.0 - 2.0 * nu) / 3.0 * p0 + (4.0 + nu) * math.pi / 8.0 * tau_edge

    fos_p = (o['p_allow'] / p0) if not math.isinf(o['p_allow']) else math.inf
    fos_f = (o['F_allow'] / N) if not math.isinf(o['F_allow']) else math.inf
    fos = min(fos_p, fos_f, 99.9)
    basis = 'Hertz / Von Mises yield' if fos_p <= fos_f else 'Thin-shell force limit'

    out.update(delta=hertz_delta(N, Re, Es), a=a, eps_c=0.2 * a / Re, p0=p0, p_mean=N / area,
               tau_mean=abs(fy) / area, tau_edge=tau_edge, c_over_a=c_over_a, sig1=sr, sig2=st, sig3=sz,
               tau_sub=tau_axis, sig_tresca=sig_tresca, sig_vm=sig_vm, sig_rankine=sig_rankine,
               sig_y_allow=sig_y_allow, fos=fos, fos_basis=basis)
    return out
