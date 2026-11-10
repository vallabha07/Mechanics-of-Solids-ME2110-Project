"""nptel_priority.py - stress, shaft, beam, energy and failure analysis of the ACTUAL grasp (six graph pages + text report).

  *   complete stress / strain tensor analysis for every gripping condition + differential-element equilibrium
  *   combined loading of the hollow wrist shaft (axial + bending + torsion + transverse shear), Von Mises & Tresca, twist, FoS
  *   geometry-dependent beam analysis (real arm tube / real finger capsules): SFD, BMD, sigma = M y / I, deflection,
      Euler-Bernoulli vs Timoshenko, cross-section comparison at equal mass
  *   energy methods on one finger: direct integration vs Castigliano vs unit-load, compared with the spring-joint finger model
  *   failure theories as the decision tool: Rankine / Tresca / Von Mises / St. Venant / Mohr-Coulomb -> permissible grip force

Everything is a closed-form textbook result (or an exact quadrature of one) at the CURRENT contact state, so each number can
be re-derived on paper.  Hardware data that is not in the MuJoCo model (wrist-shaft loads, finger wall thickness...) are
listed in WRIST / FINGER_ASSUME below and are meant to be edited.

Offline report (no GUI, no MuJoCo):   python3 -m grip_gz.nptel_priority [object] [pad] [outdir]
"""
import math
import numpy as np

from . import mech_extras as MX
from . import mos as MOS
from . import hand as HD
from .materials import OBJECTS, PADS, GRIPPER, G, truth

A = MX.ASSUME

WRIST = dict(
    E=200e9,
    nu=0.30,
    tilt_deg=15.0,
    overhang_hand=0.040,
    overhang_obj=0.100,
    a_lift=2.0,
    fos_target=2.0,
)
FINGER_ASSUME = dict(wall=0.002, nu=0.33, rect_b=A['ph_b'], rect_h=A['ph_h'])
SLIP_SF = 1.25
THEORIES = ('Rankine', 'Tresca', 'Von Mises', 'St. Venant', 'Mohr-Coulomb')
FINGER_SECTIONS = ('capsule (solid, as modelled)', 'hollow tube (wall 2 mm)', 'rectangular block')
COMPONENTS = ('arm link', 'finger')
DEFAULTS = dict(theta=30.0, phi=0.0, comp='arm link', fsec=FINGER_SECTIONS[0], theory='Von Mises', fos=1.5)


def _unit(x):
    m = float(np.max(np.abs(x))) if np.size(x) else 0.0
    if m >= 1e9: return 1e9, 'GPa'
    if m >= 1e6: return 1e6, 'MPa'
    if m >= 1e3: return 1e3, 'kPa'
    return 1.0, 'Pa'


def _mat(M, sc, d=3):
    return '\n'.join('  [' + ' '.join(f'{v / sc:10.{d}f}' for v in row) + ' ]' for row in np.atleast_2d(M))


def _vec(v, d=3):
    return '(' + ', '.join(f'{x:+.{d}f}' for x in v) + ')'


def _cumtrap_mid(f_mid, dx):
    out = np.zeros(len(f_mid) + 1)
    out[1:] = np.cumsum(f_mid * dx)
    return out


def _trap(y, x):
    return float(np.sum(0.5 * (y[1:] + y[:-1]) * np.diff(x)))


def _norm(v):
    n = np.linalg.norm(v)
    return v / n if n > 0 else v


def _sy(ctx):
    return MX._yield_strength(ctx['o'], ctx['key'], ctx['ms'])


def plane_normal(theta_deg, phi_deg):
    th, ph = math.radians(theta_deg), math.radians(phi_deg)
    return np.array([math.sin(th) * math.cos(ph), math.sin(th) * math.sin(ph), math.cos(th)])


def tensor_report(ms, o, theta_deg=30.0, phi_deg=0.0):
    St = MX.stress_tensor(ms)
    S = St['S']
    E, nu = o['E'], o['nu']
    I3 = np.eye(3)
    tol = 1e-9 * max(float(np.abs(S).max()), 1e-30)

    n = plane_normal(theta_deg, phi_deg)
    t = S @ n
    sn = float(t @ n)
    ts = t - sn * n
    tau = float(np.linalg.norm(ts))
    if tau > tol:
        e1 = ts / tau
    else:
        a = np.array([1.0, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1.0, 0])
        e1 = _norm(a - (a @ n) * n)
    e2 = np.cross(n, e1)
    Q = np.array([e1, e2, n])
    Sp = Q @ S @ Q.T
    inv = MX.invariants(S)
    inv_p = MX.invariants(Sp)

    w, V = np.linalg.eigh(S)
    w, V = w[::-1], V[:, ::-1]
    for j in range(3):
        if V[np.argmax(np.abs(V[:, j])), j] < 0: V[:, j] = -V[:, j]
    tau_max = 0.5 * (w[0] - w[2])
    n45 = [_norm(V[:, 0] + V[:, 2]), _norm(V[:, 0] - V[:, 2])]

    circles = [((w[0] + w[1]) / 2, abs(w[0] - w[1]) / 2, 'C12'), ((w[1] + w[2]) / 2, abs(w[1] - w[2]) / 2, 'C23'),
               ((w[0] + w[2]) / 2, abs(w[0] - w[2]) / 2, 'C13 (max shear)')]
    rng = np.random.default_rng(0)
    Nn = rng.normal(size=(3500, 3))
    Nn /= np.linalg.norm(Nn, axis=1)[:, None]
    Tn = Nn @ S
    sn_c = np.einsum('ij,ij->i', Tn, Nn)
    tau_c = np.sqrt(np.clip(np.einsum('ij,ij->i', Tn, Tn) - sn_c ** 2, 0, None))

    th = np.linspace(0, 180, 181)
    sweep_sn, sweep_tau = [], []
    for a_ in th:
        nn = plane_normal(a_, phi_deg)
        tt = S @ nn
        s_ = float(tt @ nn)
        sweep_sn.append(s_)
        sweep_tau.append(float(np.linalg.norm(tt - s_ * nn)))

    sm = inv['sig_oct']
    dev = inv['dev']
    J2 = inv['J2']
    J3 = float(np.linalg.det(dev))
    cos3 = 1.5 * math.sqrt(3.0) * J3 / max(J2 ** 1.5, 1e-300)
    lode = math.degrees(math.acos(max(-1.0, min(1.0, cos3))) / 3.0)

    eps = (1 + nu) / E * S - nu / E * inv['I1'] * I3
    we = np.sort(np.linalg.eigvalsh(eps))[::-1]
    e_v = float(np.trace(eps))
    Kb = E / (3 * (1 - 2 * nu))
    Gm = E / (2 * (1 + nu))
    e_dev = eps - e_v / 3 * I3
    gam_max = we[0] - we[2]
    gam_oct = 2.0 / 3.0 * math.sqrt((we[0] - we[1]) ** 2 + (we[1] - we[2]) ** 2 + (we[2] - we[0]) ** 2)
    U_h = 0.5 * sm * e_v
    U_d = J2 / (2 * Gm)
    U = 0.5 * float(np.sum(S * eps))
    return dict(S=S, sym=float(np.abs(S - S.T).max()), n=n, t=t, sn=sn, tau=tau, shear_dir=e1, Q=Q, Sp=Sp,
                chk_Sp_normal=abs(Sp[2, 2] - sn), chk_Sp_shear=abs(math.hypot(Sp[0, 2], Sp[1, 2]) - tau),
                inv=inv, inv_p=inv_p, chk_inv=max(abs(inv['I1'] - inv_p['I1']), abs(inv['I2'] - inv_p['I2']) / max(abs(inv['I2']), 1e-30) * abs(inv['I1']) * 0, 0.0),
                dI1=abs(inv['I1'] - inv_p['I1']) / max(abs(inv['I1']), 1e-30),
                dI2=abs(inv['I2'] - inv_p['I2']) / max(abs(inv['I2']), 1e-30), dI3=abs(inv['I3'] - inv_p['I3']) / max(abs(inv['I3']), 1e-30),
                p=w, V=V, tau_max=tau_max, n45=n45, circles=circles, cloud=(sn_c, tau_c),
                sweep=(th, np.array(sweep_sn), np.array(sweep_tau)),
                sm=sm, dev=dev, J2=J2, J3=J3, lode=lode, tau_oct=inv['tau_oct'], svm=inv['svm'],
                eps=eps, pe=we, e_v=e_v, e_dev=e_dev, gam_max=gam_max, gam_oct=gam_oct, K=Kb, G=Gm,
                chk_ev=abs(e_v - sm / Kb) / max(abs(e_v), 1e-30), chk_g=abs(tau_max - Gm * gam_max) / max(tau_max, 1e-30),
                chk_goct=abs(inv['tau_oct'] - Gm * gam_oct) / max(inv['tau_oct'], 1e-30),
                U=U, U_h=U_h, U_d=U_d, chk_U=abs(U - U_h - U_d) / max(abs(U), 1e-30))


def _density(o):
    if o['shape'] == 'box':
        V = o['dims'][0] * o['dims'][1] * o['dims'][2]
    else:
        V = math.pi * o['dims'][0] ** 2 * o['dims'][1]
    return o['m'] / max(V, 1e-12)


def equilibrium_element(ms, o, pt=(0.5, 0.0, 0.48), d=0.02):
    nu = o['nu']
    p0, a = ms['p0'], ms['a']
    c = np.array(pt, float)
    face = {}
    for ax in range(3):
        for sg in (1, -1):
            q = c.copy()
            q[ax] += sg * d / 2
            face[(ax, sg)] = MX._hertz_cart(q[0], q[1], q[2], nu)
    S0 = MX._hertz_cart(c[0], c[1], c[2], nu)
    terms = np.array([[(face[(ax, 1)][i, ax] - face[(ax, -1)][i, ax]) / d for ax in range(3)] for i in range(3)])
    div = terms.sum(axis=1)
    scale = max(float(np.abs(terms).max()), 1e-12)
    rho = _density(o)
    grad_phys = p0 / a
    return dict(pt=c, d=d, S0=S0 * p0, face=face, terms=terms * grad_phys, div=div * grad_phys, rel=np.abs(div) / scale,
                scale=scale * grad_phys, rho=rho, rho_g=rho * G, body_ratio=rho * G / (scale * grad_phys),
                sym=float(np.abs(S0 - S0.T).max()), a=a, p0=p0, d_phys=d * a)


def all_conditions(pad_key='small', n_f=None):
    n_f = n_f or int(HD.HAND['n_fingers'])
    rows = []
    for key, o in OBJECTS.items():
        try:
            tr = truth(key, pad_key, n=n_f)
            N = min(1.5 * tr['N_slip'], GRIPPER['F_max'])
            fy = o['m'] * G / n_f
            ms = MOS.mos_state(N, fy, key, pad_key)
            r = tensor_report(ms, o, 30.0, 0.0)
            rows.append(dict(key=key, label=o['label'], N=N, ms=ms, rep=r))
        except Exception as e:
            rows.append(dict(key=key, label=o.get('label', key), N=float('nan'), err=repr(e)))
    return rows


def wrist_state(m_obj, W=None):
    W = dict(WRIST, **(W or {}))
    ro, ri, L = A['sh_OD'] / 2, A['sh_ID'] / 2, A['sh_L']
    Ar = math.pi * (ro ** 2 - ri ** 2)
    I = math.pi / 4 * (ro ** 4 - ri ** 4)
    J = 2 * I
    E, Gs, sy = W['E'], A['sh_G'], A['sh_sy']
    ge = G + W['a_lift']
    Wo, Wh = m_obj * ge, A['hand_mass'] * ge
    b = math.radians(W['tilt_deg'])
    Vt = (Wo + Wh) * math.cos(b)
    F = (Wo + Wh) * math.sin(b)
    M_root = (Wo * (L + W['overhang_obj']) + Wh * (L + W['overhang_hand'])) * math.cos(b)
    M_tip = (Wo * W['overhang_obj'] + Wh * W['overhang_hand']) * math.cos(b)
    T = Wo * A['com_offset']
    # Patched P2: Correct hollow-tube transverse shear formula tau_V = 4V(r_o^2 + r_o r_i + r_i^2) / [3pi(r_o^4 - r_i^4)]
    # and correct vector sign addition tau = T*r/J + tauV * sin(phi)
    tauV = Vt * 4.0 * (ro ** 2 + ro * ri + ri ** 2) / (3.0 * math.pi * (ro ** 4 - ri ** 4))
    phi = np.radians(np.linspace(0, 360, 361))
    grid = {}
    best = None
    for nm, rr in (('outer', ro), ('inner', ri)):
        sig = F / Ar + M_root * rr * np.cos(phi) / I
        tau = T * rr / J + tauV * np.sin(phi)
        r_ = np.sqrt((sig / 2) ** 2 + tau ** 2)
        p1, p2 = sig / 2 + r_, sig / 2 - r_
        p3 = np.zeros_like(p1)
        vm = np.sqrt(sig ** 2 + 3 * tau ** 2)
        tr = np.maximum.reduce([np.abs(p1 - p2), np.abs(p1 - p3), np.abs(p2 - p3)])
        grid[nm] = dict(sig=sig, tau=tau, p1=p1, p2=p2, vm=vm, tresca=tr)
        k = int(np.argmax(vm))
        if best is None or vm[k] > best['vm']:
            best = dict(where=nm, phi_deg=float(np.degrees(phi[k])), sig=float(sig[k]), tau=float(tau[k]), p1=float(p1[k]),
                        p2=float(p2[k]), vm=float(vm[k]), tresca=float(tr[k]), k=k)
    tr_max = max(float(g['tresca'].max()) for g in grid.values())
    twist = T * L / (Gs * J)
    defl = Vt * L ** 3 / (3 * E * I) + M_tip * L ** 2 / (2 * E * I)
    return dict(W=W, ro=ro, ri=ri, L=L, Ar=Ar, I=I, J=J, Wo=Wo, Wh=Wh, V=Vt, F=F, M=M_root, M_tip=M_tip, T=T, tauV=tauV,
                sig_ax=F / Ar, sig_b=M_root * ro / I, tau_T=T * ro / J, phi=phi, grid=grid, crit=best, tresca_max=tr_max,
                twist_deg=math.degrees(twist), defl=defl, sy=sy, fos_vm=sy / max(best['vm'], 1e-9), fos_tr=sy / max(tr_max, 1e-9),
                fos_vm_max_over=None)


def wrist_m_allow(fos, W=None):
    lo, hi = 0.0, 500.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        s = wrist_state(mid, W)
        if min(s['fos_vm'], s['fos_tr']) >= fos: lo = mid
        else: hi = mid
    return lo


def kappa_circle(nu): return 6 * (1 + nu) / (7 + 6 * nu)
def kappa_rect(nu): return 10 * (1 + nu) / (12 + 11 * nu)


def kappa_tube(nu, ro, ri):
    m = ri / ro if ro > 0 else 0.0
    return 6 * (1 + nu) * (1 + m ** 2) ** 2 / ((7 + 6 * nu) * (1 + m ** 2) ** 2 + (20 + 12 * nu) * m ** 2)


def sec_circle(r, nu):
    return dict(name='solid circle', A=math.pi * r ** 2, I=math.pi * r ** 4 / 4, c=r, kap=kappa_circle(nu), tauf=4 / 3, d=2 * r)


def sec_tube(ro, ri, nu):
    Ar = math.pi * (ro ** 2 - ri ** 2)
    Q = (2.0 / 3.0) * (ro ** 3 - ri ** 3)
    I = math.pi / 4 * (ro ** 4 - ri ** 4)
    b = 2.0 * (ro - ri)
    return dict(name='tube', A=Ar, I=I, c=ro, kap=kappa_tube(nu, ro, ri),
                tauf=Q * Ar / (I * b), d=2 * ro)


def sec_rect(b, h, nu):
    return dict(name='rectangle', A=b * h, I=b * h ** 3 / 12, c=h / 2, kap=kappa_rect(nu), tauf=1.5, d=h)


def sec_box(h, t, nu):
    Ar = h * h - (h - 2 * t) ** 2
    I = (h ** 4 - (h - 2 * t) ** 4) / 12
    Q = h * t * (h / 2 - t / 2) + 2 * t * (h / 2 - t) * (h / 2 - t) / 2
    return dict(name='square box', A=Ar, I=I, c=h / 2, kap=2 * t * h / Ar, tauf=Q * Ar / (I * 2 * t), d=h)


def sec_I(b, h, tf, tw, nu):
    Ar = 2 * b * tf + tw * (h - 2 * tf)
    I = (b * h ** 3 - (b - tw) * (h - 2 * tf) ** 3) / 12
    Q = b * tf * (h / 2 - tf / 2) + tw * (h / 2 - tf) ** 2 / 2
    return dict(name='I-beam', A=Ar, I=I, c=h / 2, kap=tw * h / Ar, tauf=Q * Ar / (I * tw), d=h)


def equal_area_sections(A_t, nu, tube):
    out = [dict(tube, name='tube (as built)')]
    out.append(sec_circle(math.sqrt(A_t / math.pi), nu))
    s = math.sqrt(A_t)
    out.append(dict(sec_rect(s, s, nu), name='solid square'))
    b = math.sqrt(A_t / 2)
    out.append(dict(sec_rect(b, 2 * b, nu), name='rect 1:2 (tall)'))
    a1 = sec_box(1.0, 0.1, nu)['A']
    out.append(sec_box(math.sqrt(A_t / a1), 0.1 * math.sqrt(A_t / a1), nu))
    a2 = sec_I(0.5, 1.0, 0.1, 1 / 16, nu)['A']
    hh = math.sqrt(A_t / a2)
    out.append(sec_I(0.5 * hh, hh, 0.1 * hh, hh / 16, nu))
    return out


def finger_sections(kind, nu=None):
    nu = FINGER_ASSUME['nu'] if nu is None else nu
    out = []
    for r in HD.HAND['cap_r']:
        if kind.startswith('capsule'): out.append(sec_circle(r, nu))
        elif kind.startswith('hollow'): out.append(sec_tube(r, r - FINGER_ASSUME['wall'], nu))
        else: out.append(sec_rect(FINGER_ASSUME['rect_b'], FINGER_ASSUME['rect_h'], nu))
    return out


def _mesh(Ls, n=240):
    xs = [0.0]
    seg = []
    for k, Lk in enumerate(Ls):
        x0 = xs[-1]
        pts = np.linspace(x0, x0 + Lk, n + 1)[1:]
        xs.extend(pts.tolist())
        seg.extend([k] * n)
    x = np.array(xs)
    return x, 0.5 * (x[1:] + x[:-1]), np.diff(x), np.array(seg)


def finger_Ls():
    """Phalanx lengths up to the CONTACT point: the force acts on the dome pad_back before the end of the last phalanx."""
    L = list(HD.HAND['L'])
    L[-1] = L[-1] - HD.HAND['pad_back']
    return L


def beam_case(comp, ctx, fsec):
    nu = FINGER_ASSUME['nu']
    if comp == 'arm link':
        E = A['arm_E']
        ro, ri = A['arm_OD'] / 2, A['arm_ID'] / 2
        secs = [sec_tube(ro, ri, nu)]
        Ls = [0.40]
        w = A['arm_rho'] * secs[0]['A'] * G
        P = (ctx['o']['m'] + A['hand_mass']) * G
        title = 'arm link 2 (OD %.0f / ID %.0f mm, L %.0f mm)' % (2e3 * ro, 2e3 * ri, 1e3 * Ls[0])
        load = 'tip payload P = (m_obj + m_hand) g = %.2f N + self-weight w = %.2f N/m' % (P, w)
    else:
        E = A['ph_E']
        secs = finger_sections(fsec, nu)
        Ls = finger_Ls()
        w = 0.0
        P = ctx['N']
        title = 'finger (phalanges %s mm)' % '/'.join('%.0f' % (1e3 * l) for l in Ls)
        load = 'tip pad force P = N = %.2f N, normal to the straight finger' % P
    x, xm, dx, seg = _mesh(Ls)
    Lt = sum(Ls)
    Ik = np.array([s['I'] for s in secs])
    Ak = np.array([s['A'] for s in secs])
    kk = np.array([s['kap'] for s in secs])
    Gs = E / (2 * (1 + nu))
    I_m, A_m, k_m = Ik[seg], Ak[seg], kk[seg]
    M_m = P * (Lt - xm) + w * (Lt - xm) ** 2 / 2
    V_m = P + w * (Lt - xm)
    th = _cumtrap_mid(M_m / (E * I_m), dx)
    th_m = 0.5 * (th[1:] + th[:-1])
    w_eb = _cumtrap_mid(th_m, dx)
    w_sh = _cumtrap_mid(V_m / (k_m * Gs * A_m), dx)
    w_ti = w_eb + w_sh
    c_m = np.array([s['c'] for s in secs])[seg]
    sig_x = M_m * c_m / I_m
    M_node = P * (Lt - x) + w * (Lt - x) ** 2 / 2
    V_node = P + w * (Lt - x)
    if len(secs) == 1:
        d_cf = P * Lt ** 3 / (3 * E * Ik[0]) + w * Lt ** 4 / (8 * E * Ik[0])
    else:
        d_cf = None
    return dict(comp=comp, title=title, load=load, E=E, Gs=Gs, nu=nu, secs=secs, Ls=Ls, Lt=Lt, P=P, w=w, x=x, xm=xm, M=M_node, V=V_node,
                sig_x=sig_x, th=th, w_eb=w_eb, w_ti=w_ti, w_sh=w_sh, d_eb=float(w_eb[-1]), d_ti=float(w_ti[-1]), d_cf=d_cf,
                shear_share=float(w_sh[-1] / max(w_ti[-1], 1e-30)), M_root=float(M_node[0]), V_root=float(V_node[0]),
                sig_root=float(M_node[0] * secs[0]['c'] / Ik[0]), tau_root=float(secs[0]['tauf'] * V_node[0] / Ak[0]), Lh=Lt / secs[0]['d'])


def section_comparison(bc):
    nu = bc['nu']
    s0 = bc['secs'][0]
    if s0['name'] == 'tube':
        ref_tube = s0
    else:
        alpha = 0.7
        ro = math.sqrt(s0['A'] / (math.pi * (1.0 - alpha ** 2)))
        ref_tube = sec_tube(ro, alpha * ro, nu)
    shapes = equal_area_sections(s0['A'], nu, ref_tube)
    out = []
    P, Lt, E, Gs, w = bc['P'], bc['Lt'], bc['E'], bc['Gs'], bc['w']
    M_root = P * Lt + 0.5 * w * Lt ** 2
    for s in shapes:
        d_eb = P * Lt ** 3 / (3 * E * s['I']) + w * Lt ** 4 / (8 * E * s['I'])
        d_sh = (P * Lt + 0.5 * w * Lt ** 2) / (s['kap'] * Gs * s['A'])
        out.append(dict(s, sig=M_root * s['c'] / s['I'], d_eb=d_eb, d_sh=d_sh, d_ti=d_eb + d_sh, Z=s['I'] / s['c']))
    return out


def energy_finger(ctx, fsec, P=None):
    P = ctx['N'] if P is None else P
    secs = finger_sections(fsec)
    Ls = finger_Ls()
    E, nu = A['ph_E'], FINGER_ASSUME['nu']
    Gs = E / (2 * (1 + nu))
    Lt = sum(Ls)
    EIk = [E * s['I'] for s in secs]
    GAk = [s['kap'] * Gs * s['A'] for s in secs]

    x, xm, dx, seg = _mesh(Ls, 400)
    M = P * (Lt - xm)
    th = _cumtrap_mid(M / np.array(EIk)[seg], dx)
    w = _cumtrap_mid(0.5 * (th[1:] + th[:-1]), dx)
    d_direct = float(w[-1])
    th_direct = float(th[-1])

    edges = np.concatenate([[0.0], np.cumsum(Ls)])

    def U_bend(Pv, M0=0.0):
        U = 0.0
        for k in range(len(Ls)):
            a_, b_ = Lt - edges[k + 1], Lt - edges[k]
            U += (Pv ** 2 * (b_ ** 3 - a_ ** 3) / 3 + Pv * M0 * (b_ ** 2 - a_ ** 2) + M0 ** 2 * (b_ - a_)) / (2 * EIk[k])
        return U

    def U_shear(Pv):
        return sum(Pv ** 2 * Ls[k] / (2 * GAk[k]) for k in range(len(Ls)))

    h = max(1e-4 * P, 1e-6)
    d_cast_fd = (U_bend(P + h) - U_bend(P - h)) / (2 * h)
    d_cast = P * sum((((Lt - edges[k]) ** 3 - (Lt - edges[k + 1]) ** 3) / 3) / EIk[k] for k in range(len(Ls)))
    d_cast_shear = (U_shear(P + h) - U_shear(P - h)) / (2 * h)
    hM = 1e-6
    th_cast = (U_bend(P, hM) - U_bend(P, -hM)) / (2 * hM)

    xf = np.linspace(0, Lt, 8001)
    segf = np.clip(np.searchsorted(edges, xf, side='right') - 1, 0, len(Ls) - 1)
    Mf = P * (Lt - xf)
    mf = (Lt - xf) * 1.0
    d_unit = _trap(Mf * mf / np.array(EIk)[segf], xf)
    th_unit = _trap(Mf * 1.0 / np.array(EIk)[segf], xf)
    d_unit_shear = _trap(P * 1.0 / np.array(GAk)[segf], xf)

    # Patched P4: Correct lever arms using Lt_contact to align perfectly with hand.compliance_along_normal()
    lev = [Lt - edges[k] for k in range(len(Ls))]          # Lt already ends at the contact point
    kj = HD.HAND['k_joint']
    U_joint = lambda Pv: sum((Pv * lev[k]) ** 2 / (2 * kj[k]) for k in range(len(Ls)))
    d_joint = (U_joint(P + h) - U_joint(P - h)) / (2 * h)
    d_hand = P * HD.compliance_along_normal(0.0)
    s_now = ctx.get('s_curl', 0.0)
    d_hand_now = P * HD.compliance_along_normal(float(s_now))
    Ps = np.linspace(0, max(2.0 * P, 1.0), 41)
    return dict(P=P, secs=secs, Ls=Ls, Lt=Lt, EI=EIk, GA=GAk, x=xf, M=Mf, m=mf, EIx=np.array(EIk)[segf], d_direct=d_direct,
                d_cast=d_cast, d_cast_fd=d_cast_fd, d_unit=d_unit, th_direct=th_direct, th_cast=th_cast, th_unit=th_unit,
                d_shear_cast=d_cast_shear, d_shear_unit=d_unit_shear, d_timo=d_cast + d_cast_shear, d_joint=d_joint, d_hand=d_hand,
                d_hand_now=d_hand_now, s_now=float(s_now), d_total=d_cast + d_cast_shear + d_joint, Ps=Ps,
                U_curve=np.array([U_bend(p_) for p_ in Ps]), U=U_bend(P), kj=kj, lev=lev,
                dU_beam=Ps * (d_cast / max(P, 1e-12)), dU_joint=Ps * (d_joint / max(P, 1e-12)),
                agree=max(abs(d_direct - d_cast), abs(d_unit - d_cast), abs(d_cast_fd - d_cast)) / max(d_cast, 1e-30))


def utilisation(theory, p, nu, E, sy, st, sc):
    s1, s2, s3 = p
    if theory == 'Rankine':
        return max(s1 / st, -s3 / sc, 0.0)
    if theory == 'Tresca':
        return max(abs(s1 - s2), abs(s2 - s3), abs(s3 - s1)) / sy
    if theory == 'Von Mises':
        return math.sqrt(0.5 * ((s1 - s2) ** 2 + (s2 - s3) ** 2 + (s3 - s1) ** 2)) / sy
    if theory == 'St. Venant':
        e1 = s1 - nu * (s2 + s3)
        e3 = s3 - nu * (s1 + s2)
        return max(e1 / st, -e3 / sc, 0.0)
    if theory == 'Mohr-Coulomb':
        return max(max(s1, 0.0) / st + max(-s3, 0.0) / sc, 0.0)
    raise ValueError(theory)


def eq_stress(theory, p, nu, st=None, sc=None):
    s1, s2, s3 = p
    if theory == 'Rankine': return max(abs(s1), abs(s3))
    if theory == 'Tresca': return max(abs(s1 - s2), abs(s2 - s3), abs(s3 - s1))
    if theory == 'Von Mises': return math.sqrt(0.5 * ((s1 - s2) ** 2 + (s2 - s3) ** 2 + (s3 - s1) ** 2))
    if theory == 'St. Venant': return max(abs(s1 - nu * (s2 + s3)), abs(s3 - nu * (s1 + s2)))
    if theory == 'Mohr-Coulomb':
        if st and sc: return max(s1, 0.0) + (st / sc) * max(-s3, 0.0)
        return s1 - s3
    return float('nan')


def _principal_at(N, ctx):
    ms = MOS.mos_state(N, ctx['ms']['fy'], ctx['key'], ctx['pad'])
    if ms['N'] <= 1e-9: return np.zeros(3), ms
    return MX.stress_tensor(ms)['p'], ms


def failure_study(ctx, fos=1.5):
    o, key = ctx['o'], ctx['key']
    sy, brittle, st, sc = _sy(ctx)
    nu, E = o['nu'], o['E']
    Ns = np.logspace(-2, 2.7, 160)
    U = {t: np.zeros(len(Ns)) for t in THEORIES}
    for i, N in enumerate(Ns):
        p, _ = _principal_at(float(N), ctx)
        for t in THEORIES:
            U[t][i] = utilisation(t, p, nu, E, st if brittle else sy, st, sc)
    try:
        N_slip_v = truth(key, ctx['pad'], n=ctx.get('n_f') or int(HD.HAND['n_fingers']))['N_slip']
    except Exception:
        N_slip_v = o['m'] * G / (max(ctx.get('n_f') or 5, 1) * o['mu'])
    N_perm = {}
    for t in THEORIES:                       # below the no-slip force the part is not held, so those forces are not candidates
        tgt = 1.0 / fos
        idx = np.where((U[t] >= tgt) & (Ns >= N_slip_v))[0]
        if len(idx) == 0:
            N_perm[t] = float('inf'); continue
        j = int(idx[0])
        lo, hi = max(float(Ns[j - 1]) if j > 0 else float(Ns[0]), N_slip_v), float(Ns[j])
        p_lo, _ = _principal_at(lo, ctx)
        if utilisation(t, p_lo, nu, E, st if brittle else sy, st, sc) >= tgt:
            N_perm[t] = lo; continue         # already over the limit at the minimum holding force
        for _ in range(60):
            mid = math.sqrt(lo * hi)
            p, _ = _principal_at(mid, ctx)
            if utilisation(t, p, nu, E, st if brittle else sy, st, sc) >= tgt: hi = mid
            else: lo = mid
        N_perm[t] = math.sqrt(lo * hi)
    pN, msN = _principal_at(ctx['N'], ctx)
    cur = {t: dict(u=utilisation(t, pN, nu, E, st if brittle else sy, st, sc), eq=eq_stress(t, pN, nu, st, sc)) for t in THEORIES}
    try:
        N_slip = truth(key, ctx['pad'], n=ctx.get('n_f') or int(HD.HAND['n_fingers']))['N_slip']
    except Exception:
        N_slip = o['m'] * G / (max(ctx.get('n_f') or 5, 1) * o['mu'])
    F_other = min(GRIPPER['F_max'], o['F_allow'])
    return dict(Ns=Ns, U=U, N_perm=N_perm, cur=cur, sy=sy, brittle=brittle, st=st, sc=sc, p=pN, N_slip=N_slip, F_other=F_other,
                fos=fos, ref=st if brittle else sy)


def decision(fs, theory):
    N_hi = min(fs['N_perm'][theory], fs['F_other'])
    N_lo = fs['N_slip'] * SLIP_SF
    ok = N_hi >= N_lo
    lim = 'failure theory' if fs['N_perm'][theory] <= fs['F_other'] else ('gripper / shell force limit')
    return dict(theory=theory, N_hi=N_hi, N_lo=N_lo, ok=ok, limiter=lim, N_rec=(N_lo if ok else None))


def permissible_force(key, pad_key, theory, fos=1.5, n_f=None, fy=None):
    o = OBJECTS[key]
    n_f = n_f or int(HD.HAND['n_fingers'])
    fy = o['m'] * G / n_f if fy is None else fy
    tr = truth(key, pad_key, n=n_f)
    ms = MOS.mos_state(1.5 * tr['N_slip'], fy, key, pad_key)
    ctx = dict(key=key, o=o, ms=ms, N=ms['N'], pad=pad_key, n_f=n_f)
    return failure_study(ctx, fos)['N_perm'][theory]


COL = dict(a='#38bdf8', b='#f59e0b', c='#34d399', d='#f87171', e='#c084fc', f='#fbbf24', g='#94a3b8')


def _ax(fig, spec, C, title, text=False):
    ax = fig.add_subplot(spec)
    ax.set_facecolor(C['bg'])
    if title: ax.set_title(title, color=C['fg'], fontsize=8, fontweight='bold', loc='left', pad=4)
    if text:
        ax.axis('off')
    else:
        ax.tick_params(colors=C['fg'], labelsize=7)
        for s in ax.spines.values(): s.set_color(C['grid'])
        ax.grid(True, color=C['grid'], lw=0.4, alpha=0.6)
    return ax


def _style(ax, C, xl=None, yl=None):
    if xl: ax.set_xlabel(xl, color=C['fg'], fontsize=7)
    if yl: ax.set_ylabel(yl, color=C['fg'], fontsize=7)


_FIT_CLS = []


def _fit_cls():
    """Text subclass that always sits INSIDE its axes: on every draw (so also after a window resize) it shrinks the font,
    and if that is not enough wraps long lines, until the whole block fits the axes it belongs to."""
    if _FIT_CLS: return _FIT_CLS[0]
    import textwrap
    from matplotlib.text import Text

    class FitText(Text):
        def __init__(self, *a, fs_max=6.9, fs_min=5.2, wrap=True, **k):
            super().__init__(*a, **k)
            self._raw, self._fs_max, self._fs_min, self._wrap, self._key = self.get_text(), fs_max, fs_min, wrap, None

        def _wh(self, r):
            bb = self.get_window_extent(r)
            return max(bb.width, 1e-6), max(bb.height, 1e-6)

        def _fit(self, r):
            box = self.axes.get_window_extent(r)
            x, y = self.get_position()
            W, H = box.width * (1.0 - x) * 0.985, box.height * y * 0.985
            self.set_text(self._raw); self.set_fontsize(self._fs_max)
            w, h = self._wh(r)
            fs = self._fs_max * min(1.0, W / w, H / h)
            comf = max(self._fs_min, 0.92 * self._fs_max)          # prefer wrapping a long line to shrinking the whole block
            if fs < comf and self._wrap:
                self.set_fontsize(comf); w, h = self._wh(r)
                lines = self._raw.split('\n')
                nmax = max(len(l) for l in lines)
                width = max(20, int(nmax * min(1.0, W / w)))
                out = []
                for l in lines:
                    ind = len(l) - len(l.lstrip(' '))
                    out += textwrap.wrap(l, width=width, subsequent_indent=' ' * (ind + 2), initial_indent=' ' * ind,
                                         break_long_words=False, drop_whitespace=False) or ['']
                self.set_text('\n'.join(out)); self.set_fontsize(comf); w, h = self._wh(r)
                fs = comf * min(1.0, W / w, H / h)
            fs = max(fs, 3.5)
            self.set_fontsize(fs)
            for _ in range(6):                      # nudge down until the measured block really fits
                w, h = self._wh(r)
                if w <= W and h <= H: break
                fs *= 0.96; self.set_fontsize(fs)

        def draw(self, renderer):
            if self.axes is not None and renderer is not None and self._raw:
                box = self.axes.get_window_extent(renderer)
                key = (round(box.width), round(box.height), round(self.figure.dpi), self._raw)
                if key != self._key:
                    self._fit(renderer); self._key = key
            super().draw(renderer)

    _FIT_CLS.append(FitText)
    return FitText


def _txt(ax, s, C, fs=6.9, col=None, wrap=True, fs_min=None, x=0.0, y=1.0, **kw):
    """Panel text that cannot overflow its axes (fs is the MAXIMUM size; see _fit_cls).  wrap=False for aligned tables."""
    t = _fit_cls()(x, y, s, fs_max=fs, fs_min=(fs_min if fs_min is not None else (5.2 if wrap else 4.2)), wrap=wrap,
                   va='top', ha='left', family='monospace', fontsize=fs, color=col or C['fg'], transform=ax.transAxes, **kw)
    ax.add_artist(t); t.set_clip_on(False)
    return t


def _grid(fig, nr, nc):
    fig.clf()
    return fig.add_gridspec(nr, nc, left=0.045, right=0.985, top=0.925, bottom=0.07, wspace=0.26, hspace=0.34)


def _mohr_circles(ax, circles, C, sc, lw=1.4):
    t = np.linspace(0, math.pi, 120)
    cols = (COL['a'], COL['c'], COL['b'])
    for (c, r, nm), col in zip(circles, cols):
        ax.plot((c + r * np.cos(t)) / sc, (r * np.sin(t)) / sc, color=col, lw=lw, label=nm)
        ax.plot((c + r * np.cos(t)) / sc, (-r * np.sin(t)) / sc, color=col, lw=lw * 0.5, alpha=0.5)


def draw_p1_tensor(fig, ctx, par, C):
    ms, o = ctx['ms'], ctx['o']
    R = tensor_report(ms, o, par['theta'], par['phi'])
    gs = _grid(fig, 2, 3)
    sc, un = _unit(R['S'])
    p = R['p']
    ax = _ax(fig, gs[0, 0], C, 'Stress tensor and principal stresses', True)
    s = (f"point z = 0.48a under the pad, axes x = r, y = theta, z = depth\n"
         f"sigma_ij  [{un}]\n{_mat(R['S'], sc)}\n"
         f"symmetry (moment equilibrium)  max|S-S^T| = {R['sym']:.1e}\n\n"
         f"principal stresses [{un}]  (eigenvalues of S)\n"
         f"  s1 = {p[0] / sc:9.3f}   n1 = {_vec(R['V'][:, 0])}\n"
         f"  s2 = {p[1] / sc:9.3f}   n2 = {_vec(R['V'][:, 1])}\n"
         f"  s3 = {p[2] / sc:9.3f}   n3 = {_vec(R['V'][:, 2])}\n"
         f"tau_max = (s1-s3)/2 = {R['tau_max'] / sc:.3f} {un}\n"
         f"  acts on planes n = (n1 +- n3)/sqrt2\n"
         f"  (45 deg to the principal axes, normal stress\n   (s1+s3)/2 = {(p[0] + p[2]) / 2 / sc:.3f} {un})\n"
         f"p0 = {ms['p0'] / sc:.3f}  edge shear = {ms['tau_edge'] / sc:.3f} {un}  (Hertz patch, N = {ms['N']:.2f} N)")
    _txt(ax, s, C)
    ax = _ax(fig, gs[0, 1], C, "Mohr's circles and every plane through the point", False)
    sn_c, tau_c = R['cloud']
    ax.scatter(sn_c / sc, tau_c / sc, s=1.2, color=COL['g'], alpha=0.35, label='all planes')
    _mohr_circles(ax, R['circles'], C, sc)
    ax.scatter([R['sn'] / sc], [R['tau'] / sc], s=48, color=COL['d'], zorder=6, edgecolor='w', label='selected plane')
    ax.scatter(p / sc, np.zeros(3), s=22, color='w', zorder=5)
    for k, v in enumerate(p): ax.annotate(f's{k + 1}', (v / sc, 0), textcoords='offset points', xytext=(0, -11), color=C['fg'], fontsize=7, ha='center')
    ax.axhline(0, color=C['fg'], lw=0.6)
    _style(ax, C, f'normal stress [{un}]', f'shear stress [{un}]')
    ax.legend(fontsize=6, facecolor=C['bg'], labelcolor=C['fg'], edgecolor=C['grid'], loc='upper left')
    ax = _ax(fig, gs[0, 2], C, f"Traction on the chosen plane (theta {par['theta']:.0f} deg, phi {par['phi']:.0f} deg)", False)
    sn, tau = R['sn'], R['tau']
    m = max(abs(sn), tau, 1e-30)
    ax.plot([0, 0], [-1.1, 1.1], color=COL['g'], lw=2)
    ax.text(0.04, 1.02, 'plane', color=COL['g'], fontsize=7)
    ax.annotate('', xy=(1.0 * np.sign(1), 0), xytext=(0, 0), arrowprops=dict(arrowstyle='->', color=C['fg'], lw=1.0, ls='--'))
    ax.text(1.0, 0.07, 'n', color=C['fg'], fontsize=8)
    tn, tt = sn / m, tau / m
    ax.annotate('', xy=(tn, tt), xytext=(0, 0), arrowprops=dict(arrowstyle='->', color=COL['d'], lw=2))
    ax.annotate('', xy=(tn, 0), xytext=(0, 0), arrowprops=dict(arrowstyle='->', color=COL['a'], lw=1.6))
    ax.annotate('', xy=(tn, tt), xytext=(tn, 0), arrowprops=dict(arrowstyle='->', color=COL['c'], lw=1.6))
    bb = dict(fc=C['bg'], ec='none', alpha=0.85, pad=1.2)
    sd = -1 if tn <= 0 else 1                        # side of the plane the arrows point to
    ax.annotate(f'sigma_n {sn / sc:.3f}', (tn / 2, 0), xytext=(0, -9), textcoords='offset points', color=COL['a'], fontsize=7, ha='center', va='top', bbox=bb)
    ax.annotate(f'tau {tau / sc:.3f}', (tn, tt / 2), xytext=(4 * sd, 0), textcoords='offset points', color=COL['c'], fontsize=7,
                ha='right' if sd < 0 else 'left', va='center', bbox=bb)
    ax.annotate(f't = S n  ({np.linalg.norm(R["t"]) / sc:.3f})', (tn / 2, tt / 2), xytext=(0, 12), textcoords='offset points', color=COL['d'], fontsize=7,
                ha='center', va='bottom', bbox=bb)
    ax.set_xlim(-1.85, 1.35) if sd < 0 else ax.set_xlim(-1.35, 1.85)
    ax.set_ylim(-1.2, 1.25); ax.set_aspect('equal')
    ax.set_xlabel(f'axes: n (normal) and shear direction   [{un}]', color=C['fg'], fontsize=7)
    ax = _ax(fig, gs[1, 0], C, 'Rotated stress, invariants, hydrostatic / deviatoric split', True)
    I = R['inv']
    s = (f"S' = Q S Q^T   rows of Q: e1 = shear dir, e2 = n x e1, e3 = n\n"
         f"S' [{un}]\n{_mat(R['Sp'], sc)}\n"
         f"  S'_33 = {R['Sp'][2, 2] / sc:.3f} = sigma_n  (|d| {R['chk_Sp_normal'] / sc:.1e})\n"
         f"  S'_13 = {R['Sp'][0, 2] / sc:.3f} = tau      (|d| {R['chk_Sp_shear'] / sc:.1e})\n"
         f"invariants (same in S and S'):  rel.diff I1 {R['dI1']:.0e}  I2 {R['dI2']:.0e}  I3 {R['dI3']:.0e}\n"
         f"  I1 = {I['I1'] / sc:.3f} {un}   I2 = {I['I2'] / sc ** 2:.3f} {un}^2   I3 = {I['I3'] / sc ** 3:.3f} {un}^3\n"
         f"S = sigma_m I + s     sigma_m = I1/3 = {R['sm'] / sc:.3f} {un}\n"
         f"deviator s [{un}]\n{_mat(R['dev'], sc)}\n"
         f"J2 = {R['J2'] / sc ** 2:.3f} {un}^2   J3 = {R['J3'] / sc ** 3:.3f}   Lode angle {R['lode']:.1f} deg\n"
         f"tau_oct = sqrt(2 J2/3) = {R['tau_oct'] / sc:.3f}   sigma_VM = sqrt(3 J2) = {R['svm'] / sc:.3f} {un}")
    _txt(ax, s, C, 6.6)
    ax = _ax(fig, gs[1, 1], C, f"Normal and shear stress vs plane angle (phi {par['phi']:.0f} deg)", False)
    th, sw_n, sw_t = R['sweep']
    ax.plot(th, sw_n / sc, color=COL['a'], label='sigma_n(theta)')
    ax.plot(th, sw_t / sc, color=COL['c'], label='tau(theta)')
    ax.axvline(par['theta'], color=COL['d'], lw=1)
    ax.scatter([par['theta']] * 2, [R['sn'] / sc, R['tau'] / sc], color=COL['d'], zorder=5, s=20)
    ax.axhline(R['tau_max'] / sc, color=COL['b'], lw=0.8, ls='--')
    ax.text(2, R['tau_max'] / sc, ' tau_max', color=COL['b'], fontsize=7, va='bottom')
    ax.legend(fontsize=6, facecolor=C['bg'], labelcolor=C['fg'], edgecolor=C['grid'])
    _style(ax, C, 'plane normal inclination theta from z [deg]', f'[{un}]')
    ax = _ax(fig, gs[1, 2], C, 'Strains and strain energy', True)
    pe = R['pe']
    s = (f"object E = {o['E'] / 1e6:.4g} MPa, nu = {o['nu']:.2f}   (K = {R['K'] / 1e6:.4g}, G = {R['G'] / 1e6:.4g} MPa)\n"
         f"eps = (1+nu)/E S - nu/E I1 I      [1e-3]\n{_mat(R['eps'], 1e-3, 4)}\n"
         f"principal strains [1e-3]\n"
         f"  e1 = {pe[0] * 1e3:9.4f}   e2 = {pe[1] * 1e3:9.4f}   e3 = {pe[2] * 1e3:9.4f}\n"
         f"volumetric strain e_v = e1+e2+e3 = {R['e_v'] * 1e3:.4f}e-3\n"
         f"  check e_v = sigma_m / K  -> rel.err {R['chk_ev']:.0e}\n"
         f"max shear strain  g_max = e1 - e3 = {R['gam_max'] * 1e3:.4f}e-3\n"
         f"  check tau_max = G g_max  -> rel.err {R['chk_g']:.0e}\n"
         f"octahedral shear g_oct = {R['gam_oct'] * 1e3:.4f}e-3\n"
         f"  check tau_oct = G g_oct  -> rel.err {R['chk_goct']:.0e}\n"
         f"strain energy density  U = {R['U']:.3f} J/m3\n"
         f"  volumetric  sigma_m e_v/2 = {R['U_h']:.3f}   distortion  J2/2G = {R['U_d']:.3f}\n"
         f"  (U_vol + U_dist = U: rel.err {R['chk_U']:.0e};  Von Mises\n   = distortion-energy criterion)")
    _txt(ax, s, C, 6.6)
    fig.suptitle(f"Stress and strain state - {o['label']}   N = {ctx['N']:.2f} N", color=C['fg'], fontsize=10)
    return R


def draw_p1_equilibrium(fig, ctx, par, C):
    ms, o = ctx['ms'], ctx['o']
    E_ = equilibrium_element(ms, o)
    gs = _grid(fig, 2, 2)
    ax = _ax(fig, gs[0, 0], C, 'differential element dx dz in the Hertz field (section y = 0)', False)
    ax.set_aspect('equal'); ax.axis('off')
    ax.add_patch(__import__('matplotlib.patches', fromlist=['Rectangle']).Rectangle((-0.5, -0.5), 1, 1, fill=False, ec=COL['a'], lw=2))
    sc, un = _unit(E_['S0'])
    S0 = E_['S0']
    for (x0, y0, dx_, dy_, lab, col) in (
            (0.5, 0, 0.45, 0, 'sigma_xx + d(sigma_xx)/dx dx/2', COL['d']), (-0.5, 0, -0.45, 0, 'sigma_xx - d(sigma_xx)/dx dx/2', COL['d']),
            (0, 0.5, 0, 0.45, 'sigma_zz + d(sigma_zz)/dz dz/2', COL['c']), (0, -0.5, 0, -0.45, 'sigma_zz - d(sigma_zz)/dz dz/2', COL['c'])):
        ax.annotate('', xy=(x0 + dx_, y0 + dy_), xytext=(x0, y0), arrowprops=dict(arrowstyle='->', color=col, lw=2))
    for (x0, y0, dx_, dy_, col) in ((0.5, 0.25, 0, 0.35, COL['b']), (-0.5, -0.25, 0, -0.35, COL['b']),
                                    (-0.25, 0.5, 0.35, 0, COL['b']), (0.25, -0.5, -0.35, 0, COL['b'])):
        ax.annotate('', xy=(x0 + dx_, y0 + dy_), xytext=(x0, y0), arrowprops=dict(arrowstyle='->', color=col, lw=1.6))
    ax.text(0.98, 0.0, 'sigma_xx', color=COL['d'], fontsize=8, va='center'); ax.text(-1.12, 0.0, 'sigma_xx', color=COL['d'], fontsize=8, va='center', ha='right')
    ax.text(0, 1.02, 'sigma_zz', color=COL['c'], fontsize=8, ha='center'); ax.text(0, -1.06, 'sigma_zz', color=COL['c'], fontsize=8, ha='center')
    ax.text(0.72, 0.62, 'tau_xz', color=COL['b'], fontsize=8); ax.text(-0.92, -0.62, 'tau_xz', color=COL['b'], fontsize=8)
    ax.text(0, -1.32, f"point (x, y, z) = ({E_['pt'][0]:.2f}, {E_['pt'][1]:.2f}, {E_['pt'][2]:.2f}) a\nsigma_xx {S0[0, 0] / sc:.3f}   sigma_zz {S0[2, 2] / sc:.3f}\ntau_xz {S0[0, 2] / sc:.3f}   sigma_yy {S0[1, 1] / sc:.3f}  [{un}]",
            color=C['fg'], fontsize=7, ha='center', va='top')
    ax.set_xlim(-1.6, 1.6); ax.set_ylim(-1.95, 1.3)
    ax = _ax(fig, gs[0, 1], C, 'equilibrium: d sigma_ij / dx_j + rho b_i = 0', True)
    T, dv = E_['terms'], E_['div']
    s = (f"Cauchy tetrahedron -> force balance on the element (edge d = {E_['d']:.2f} a = {E_['d_phys'] * 1e3:.3f} mm):\n"
         f"  [sigma_ij(x_j + d/2) - sigma_ij(x_j - d/2)] dA  summed over the 6 faces = 0\n"
         f"stress gradients from the face tractions  [Pa/m]\n"
         f"  x:  d sxx/dx {T[0, 0]:+.3e}  d txy/dy {T[0, 1]:+.3e}  d txz/dz {T[0, 2]:+.3e}   sum {dv[0]:+.2e}\n"
         f"  y:  d tyx/dx {T[1, 0]:+.3e}  d syy/dy {T[1, 1]:+.3e}  d tyz/dz {T[1, 2]:+.3e}   sum {dv[1]:+.2e}\n"
         f"  z:  d tzx/dx {T[2, 0]:+.3e}  d tzy/dy {T[2, 1]:+.3e}  d szz/dz {T[2, 2]:+.3e}   sum {dv[2]:+.2e}\n"
         f"residual / largest term:  x {E_['rel'][0] * 100:.2f} %   y {E_['rel'][1] * 100:.2f} %   z {E_['rel'][2] * 100:.2f} %\n"
         f"  (finite element + quadrature error; the Boussinesq/Hertz field satisfies div S = 0 exactly)\n"
         f"moment equilibrium  tau_ij = tau_ji :  max|S-S^T| = {E_['sym']:.1e}\n\n"
         f"body force: rho g = {E_['rho_g']:.0f} N/m3  (rho ~ {E_['rho']:.0f} kg/m3)\n"
         f"  vs stress gradient ~ p0/a = {E_['scale']:.2e} N/m3   ratio {E_['body_ratio']:.1e}\n"
         f"  -> the object's own weight is negligible inside the contact patch;\n"
         f"     it only matters through the friction force fy = {ctx['ms']['fy']:.2f} N per pad.\n"
         f"(frictionless Hertz field shown; the friction (Cerruti) part is self-equilibrated too.)")
    _txt(ax, s, C, 6.6)
    ax = _ax(fig, gs[1, :], C, 'all gripping conditions: same pipeline, N = 1.5 N_slip (<= gripper limit), fy = m g / n  [stresses in kPa]', True)
    rows = all_conditions(ctx['pad'], ctx.get('n_f'))
    head = f"{'object':<22}{'N [N]':>7}{'s1':>10}{'s2':>10}{'s3':>10}{'tau_max':>10}{'I1':>10}{'sigma_VM':>10}{'e_v 1e-3':>10}{'g_max 1e-3':>11}{'U J/m3':>10}"
    lines = [head, '-' * len(head)]
    for r in rows:
        if 'err' in r: lines.append(f"{r['label'][:21]:<22}  (skipped: {r['err'][:60]})"); continue
        R = r['rep']
        pk = R['p']
        lines.append(f"{r['label'][:21]:<22}{r['N']:7.2f}{pk[0] / 1e3:10.1f}{pk[1] / 1e3:10.1f}{pk[2] / 1e3:10.1f}{R['tau_max'] / 1e3:10.1f}"
                     f"{R['inv']['I1'] / 1e3:10.1f}{R['svm'] / 1e3:10.1f}{R['e_v'] * 1e3:10.4f}{R['gam_max'] * 1e3:11.4f}{R['U']:10.2f}")
    _txt(ax, '\n'.join(lines), C, 8.0, wrap=False, fs_min=4.2)
    fig.suptitle(f"Element equilibrium and all gripping cases - {o['label']}", color=C['fg'], fontsize=10)
    return E_, rows


def draw_p2_wrist(fig, ctx, par, C):
    o = ctx['o']
    S = wrist_state(o['m'])
    gs = _grid(fig, 2, 3)
    ax = _ax(fig, gs[0, 0], C, 'wrist shaft free-body diagram', False)
    ax.axis('off')
    L, ov_h, ov_o = S['L'], S['W']['overhang_hand'], S['W']['overhang_obj']
    from matplotlib.patches import Rectangle
    ax.add_patch(Rectangle((-0.02, -0.012), 0.02, 0.024 + 0.01, color=COL['g'], alpha=0.5))
    ax.add_patch(Rectangle((0, -S['ro']), L, 2 * S['ro'], fc='none', ec=COL['a'], lw=2))
    ax.add_patch(Rectangle((0, -S['ri']), L, 2 * S['ri'], fc='none', ec=COL['a'], lw=0.8, ls='--'))
    ax.add_patch(Rectangle((L, -0.012), ov_h + 0.01, 0.024, fc=COL['g'], alpha=0.4))
    for x0, W_, col, lab in ((L + ov_h, S['Wh'], COL['b'], 'W_hand'), (L + ov_o, S['Wo'], COL['d'], 'W_obj')):
        ax.annotate('', xy=(x0, -0.05), xytext=(x0, 0.0), arrowprops=dict(arrowstyle='->', color=col, lw=2))
        ax.text(x0, -0.062, f'{lab}\n{W_:.1f} N', color=col, fontsize=7, ha='center', va='top')
    ax.annotate('', xy=(L * 0.5 + 0.03, 0.0), xytext=(L * 0.5 - 0.01, 0.0), arrowprops=dict(arrowstyle='->', color=COL['e'], lw=1.5))
    ax.text(L * 0.5, 0.018, f"F axial {S['F']:.1f} N", color=COL['e'], fontsize=7, ha='center', va='bottom',
            bbox=dict(fc=C['bg'], ec='none', alpha=0.85, pad=1.2))
    _txt(ax, f"OD {2e3 * S['ro']:.0f}/ID {2e3 * S['ri']:.0f} mm, L {1e3 * L:.0f} mm\n"
             f"M_root {S['M']:.2f} N.m   V {S['V']:.1f} N   T {S['T'] * 1e3:.0f} N.mm", C, 7, wrap=False, fs_min=4.5)
    ax.set_xlim(-0.03, L + ov_o + 0.06); ax.set_ylim(-0.12, 0.12)
    ax = _ax(fig, gs[0, 1], C, 'normal stress across the section: F/A + M y / I', False)
    y = np.linspace(-S['ro'], S['ro'], 200)
    sg = S['sig_ax'] + S['M'] * y / S['I']
    sgm = np.where(np.abs(y) < S['ri'], np.nan, sg)
    ax.plot(sg / 1e6, y * 1e3, color=COL['g'], lw=0.8, ls=':')
    ax.plot(sgm / 1e6, y * 1e3, color=COL['a'], lw=2)
    ax.axhspan(-S['ri'] * 1e3, S['ri'] * 1e3, color=COL['g'], alpha=0.15)
    ax.axvline(S['sig_ax'] / 1e6, color=COL['e'], lw=0.8, ls='--')
    ax.text(S['sig_ax'] / 1e6, 0, ' axial F/A', color=COL['e'], fontsize=7)
    _style(ax, C, 'sigma [MPa]', 'y [mm]  (hollow core shaded)')
    ax = _ax(fig, gs[0, 2], C, f"Mohr's circles at the critical point ({S['crit']['where']} surface, phi {S['crit']['phi_deg']:.0f} deg)", False)
    cr = S['crit']
    circ = [((cr['p1'] + cr['p2']) / 2, abs(cr['p1'] - cr['p2']) / 2, 'C12'), ((cr['p2'] + 0) / 2, abs(cr['p2']) / 2, 'C23'), ((cr['p1'] + 0) / 2, abs(cr['p1']) / 2, 'C13')]
    _mohr_circles(ax, circ, C, 1e6)
    ax.scatter([cr['sig'] / 1e6, 0], [cr['tau'] / 1e6, -cr['tau'] / 1e6], color=COL['d'], s=26, zorder=5)
    ax.plot([cr['sig'] / 1e6, 0], [cr['tau'] / 1e6, -cr['tau'] / 1e6], color=COL['d'], lw=0.8)
    ax.scatter([cr['p1'] / 1e6, cr['p2'] / 1e6, 0], [0, 0, 0], color='w', s=14, zorder=6)
    ax.axhline(0, color=C['fg'], lw=0.5); ax.set_aspect('equal', adjustable='datalim')
    _style(ax, C, 'sigma [MPa]', 'tau [MPa]')
    ax.legend(fontsize=6, facecolor=C['bg'], labelcolor=C['fg'], edgecolor=C['grid'])
    ax = _ax(fig, gs[1, 0], C, 'Von Mises and Tresca around the section', False)
    ph = np.degrees(S['phi'])
    for nm, ls in (('outer', '-'), ('inner', '--')):
        g = S['grid'][nm]
        ax.plot(ph, g['vm'] / 1e6, color=COL['a'], ls=ls, label=f'Von Mises ({nm})')
        ax.plot(ph, g['tresca'] / 1e6, color=COL['b'], ls=ls, label=f'Tresca ({nm})')
    ax.axhline(S['sy'] / 1e6, color=COL['d'], lw=1)
    ax.text(5, S['sy'] / 1e6, ' yield', color=COL['d'], fontsize=7, va='bottom')
    ax.set_yscale('log') if S['sy'] / max(S['crit']['vm'], 1) > 30 else None
    ax.legend(fontsize=6, facecolor=C['bg'], labelcolor=C['fg'], edgecolor=C['grid'], ncol=2)
    _style(ax, C, 'angle phi from the top fibre [deg]', 'equivalent stress [MPa]')
    ax = _ax(fig, gs[1, 1], C, 'factor of safety vs object mass', False)
    ms_ = np.linspace(0.05, max(3 * o['m'], 5.0), 60)
    f_vm = [wrist_state(m)['fos_vm'] for m in ms_]
    f_tr = [wrist_state(m)['fos_tr'] for m in ms_]
    ax.semilogy(ms_, f_vm, color=COL['a'], label='Von Mises'); ax.semilogy(ms_, f_tr, color=COL['b'], label='Tresca')
    ax.axhline(S['W']['fos_target'], color=COL['d'], lw=0.8, ls='--'); ax.axhline(1.0, color=COL['d'], lw=1)
    ax.axvline(o['m'], color=COL['c'], lw=1)
    ax.legend(fontsize=6, facecolor=C['bg'], labelcolor=C['fg'], edgecolor=C['grid'])
    _style(ax, C, 'object mass [kg]', 'factor of safety')
    ax = _ax(fig, gs[1, 2], C, 'results', True)
    m_allow = wrist_m_allow(S['W']['fos_target'])
    s = (f"loads (g_eff = g + a_lift = {G + S['W']['a_lift']:.2f} m/s2, tilt {S['W']['tilt_deg']:.0f} deg)\n"
         f"  axial F = (W_o+W_h) sin(tilt) = {S['F']:.2f} N\n"
         f"  bending M = sum W l cos(tilt) = {S['M']:.3f} N.m  (root)\n"
         f"  torque T = W_o e = {S['Wo']:.2f} x {A['com_offset'] * 1e3:.0f} mm = {S['T'] * 1e3:.1f} N.mm\n"
         f"section A {S['Ar'] * 1e6:.1f} mm2  I {S['I'] * 1e12:.0f} mm4  J {S['J'] * 1e12:.0f} mm4\n"
         f"stresses at the root:\n"
         f"  sigma_axial {S['sig_ax'] / 1e6:6.2f}   sigma_bend {S['sig_b'] / 1e6:6.2f} MPa\n"
         f"  tau_torsion {S['tau_T'] / 1e6:6.2f}   tau_shear(V) {S['tauV'] / 1e6:6.2f} MPa\n"
         f"critical point: sigma {cr['sig'] / 1e6:.2f}  tau {cr['tau'] / 1e6:.2f} MPa\n"
         f"  principal s1 {cr['p1'] / 1e6:.2f}  s2 {cr['p2'] / 1e6:.2f}  s3 0\n"
         f"  Von Mises  sqrt(s^2+3t^2) = {cr['vm'] / 1e6:.2f} MPa   FoS {S['fos_vm']:.1f}\n"
         f"  Tresca     sqrt(s^2+4t^2) = {S['tresca_max'] / 1e6:.2f} MPa   FoS {S['fos_tr']:.1f}\n"
         f"angle of twist  T L / G J = {S['twist_deg']:.4f} deg\n"
         f"tip deflection {S['defl'] * 1e6:.2f} um\n"
         f"heaviest object at FoS >= {S['W']['fos_target']:.1f}:  {m_allow:.1f} kg\n"
         f"(editable assumptions: WRIST table in the analysis module)")
    _txt(ax, s, C, 6.6)
    fig.suptitle(f"Wrist shaft under combined loading - {o['label']}", color=C['fg'], fontsize=10)
    return S


def draw_p3_beams(fig, ctx, par, C):
    bc = beam_case(par['comp'], ctx, par['fsec'])
    gs = _grid(fig, 2, 3)
    x = bc['x'] * 1e3
    ax = _ax(fig, gs[0, 0], C, 'shear force diagram V(x)', False)
    ax.fill_between(x, bc['V'], color=COL['c'], alpha=0.35); ax.plot(x, bc['V'], color=COL['c'])
    _style(ax, C, 'x from the fixed end [mm]', 'V [N]')
    ax = _ax(fig, gs[1, 0], C, 'bending moment diagram M(x)', False)
    ax.fill_between(x, bc['M'], color=COL['b'], alpha=0.35); ax.plot(x, bc['M'], color=COL['b'])
    _style(ax, C, 'x from the fixed end [mm]', 'M [N.m]')
    ax = _ax(fig, gs[0, 1], C, 'stress distribution at the root: sigma = M y / I', False)
    s0 = bc['secs'][0]
    y = np.linspace(-s0['c'], s0['c'], 80)
    sg = bc['M_root'] * y / s0['I']
    ax.plot(sg / 1e6, y * 1e3, color=COL['a'], lw=2); ax.fill_betweenx(y * 1e3, 0, sg / 1e6, color=COL['a'], alpha=0.25)
    ax.axvline(0, color=C['fg'], lw=0.5)
    ax.text(0.98, 0.03, f"sigma_max = M c / I = {bc['sig_root'] / 1e6:.3f} MPa\ntau_max = {s0['tauf']:.2f} V/A = {bc['tau_root'] / 1e6:.3f} MPa",
            transform=ax.transAxes, color=C['fg'], fontsize=6.5, ha='right', va='bottom', bbox=dict(fc=C['bg'], ec=C['grid'], alpha=0.9, pad=2))
    _style(ax, C, 'sigma [MPa]', 'y [mm]')
    ax = _ax(fig, gs[1, 1], C, 'deflection: Euler-Bernoulli vs Timoshenko', False)
    ax.plot(x, bc['w_eb'] * 1e3, color=COL['a'], label=f"Euler-Bernoulli  {bc['d_eb'] * 1e3:.4g} mm")
    ax.plot(x, bc['w_ti'] * 1e3, color=COL['d'], ls='--', label=f"Timoshenko  {bc['d_ti'] * 1e3:.4g} mm")
    ax.legend(fontsize=6, facecolor=C['bg'], labelcolor=C['fg'], edgecolor=C['grid'])
    _style(ax, C, 'x [mm]', 'w(x) [mm]')
    ax = _ax(fig, gs[0, 2], C, 'cross-section comparison at equal area (= equal mass)', False)
    cmp_ = section_comparison(bc)
    names = [c['name'] for c in cmp_]
    sig = np.array([c['sig'] for c in cmp_]); dd = np.array([c['d_ti'] for c in cmp_])
    xi = np.arange(len(cmp_))
    ax.bar(xi - 0.2, sig / sig[0], 0.4, color=COL['b'], label='sigma_max / tube')
    ax.bar(xi + 0.2, dd / dd[0], 0.4, color=COL['a'], label='deflection / tube')
    ax.set_xticks(xi); ax.set_xticklabels(names, rotation=30, ha='right', fontsize=6, color=C['fg'])
    ax.axhline(1, color=C['fg'], lw=0.6)
    ax.legend(fontsize=6, facecolor=C['bg'], labelcolor=C['fg'], edgecolor=C['grid'])
    ax.set_yscale('log')
    _style(ax, C, None, 'ratio (log)')
    ax = _ax(fig, gs[1, 2], C, 'numbers', True)
    s = [f"{bc['title']}", f"{bc['load']}", f"E = {bc['E'] / 1e9:.0f} GPa  G = {bc['Gs'] / 1e9:.1f} GPa  L/h = {bc['Lh']:.1f}"]
    for k, sec in enumerate(bc['secs']):
        s.append(f"  segment {k + 1}: {sec['name']:<12} d {sec['d'] * 1e3:5.1f} mm  A {sec['A'] * 1e6:7.1f} mm2  I {sec['I'] * 1e12:10.1f} mm4  kappa {sec['kap']:.3f}")
    s += ["", f"sigma_b = M y / I      : root M = {bc['M_root']:.4g} N.m -> {bc['sig_root'] / 1e6:.4g} MPa",
          f"delta (EB, numeric)   : {bc['d_eb'] * 1e3:.5g} mm"]
    if bc['d_cf'] is not None:
        s.append(f"P L^3/3EI (+wL^4/8EI) : {bc['d_cf'] * 1e3:.5g} mm   (rel.err {abs(bc['d_eb'] - bc['d_cf']) / bc['d_cf']:.0e})")
    s += [f"delta (Timoshenko)    : {bc['d_ti'] * 1e3:.5g} mm   shear part {bc['shear_share'] * 100:.2f} %",
          "", "Timoshenko adds  dw_s/dx = V / (kappa G A).  For slender beams (L/h > 10)",
          "the shear share is ~1 % or less, so Euler-Bernoulli is adequate; for", "short, stubby members (L/h < 5) it is not.",
          "", "Equal-area comparison: moving material away from the neutral axis", "(tube / box / I) raises I and cuts both sigma and delta at no extra mass."]
    _txt(ax, '\n'.join(s), C, 6.4)
    fig.suptitle(f"Beam bending, shear and deflection ({par['comp']})", color=C['fg'], fontsize=10)
    return bc


def draw_p4_energy(fig, ctx, par, C):
    EN = energy_finger(ctx, par['fsec'])
    gs = _grid(fig, 2, 3)
    x = EN['x'] * 1e3
    ax = _ax(fig, gs[0, 0], C, 'M(x) (real load)  and  m(x) (unit load at the tip)', False)
    ax.plot(x, EN['M'], color=COL['b'], label='M(x) = P (L - x)')
    ax.set_ylabel('M [N.m]', color=COL['b'], fontsize=7)
    ax2 = ax.twinx(); ax2.plot(x, EN['m'] * 1e3, color=COL['c'], label='m(x)'); ax2.set_ylabel('m [mm per unit N]', color=COL['c'], fontsize=7)
    ax2.tick_params(colors=C['fg'], labelsize=7)
    ax.set_xlabel('x [mm]', color=C['fg'], fontsize=7)
    ax = _ax(fig, gs[0, 1], C, 'unit-load integrand  M m / EI   (area = delta)', False)
    integ = EN['M'] * EN['m'] / EN['EIx']
    ax.fill_between(x, integ * 1e6, color=COL['e'], alpha=0.4); ax.plot(x, integ * 1e6, color=COL['e'])
    _style(ax, C, 'x [mm]', 'M m / EI [um/mm]')
    ax = _ax(fig, gs[0, 2], C, "Castigliano: U(P), slope = delta", False)
    ax.plot(EN['Ps'], EN['U_curve'] * 1e6, color=COL['a'])
    P = EN['P']
    ax.scatter([P], [EN['U'] * 1e6], color=COL['d'], zorder=5)
    Pt = EN['Ps']
    ax.plot(Pt, (EN['U'] + EN['d_cast'] * (Pt - P)) * 1e6, color=COL['d'], ls='--', lw=1, label=f"tangent, slope dU/dP = {EN['d_cast'] * 1e6:.3g} um")
    ax.legend(fontsize=6, facecolor=C['bg'], labelcolor=C['fg'], edgecolor=C['grid'])
    _style(ax, C, 'tip force P [N]', 'strain energy U [uJ]')
    ax = _ax(fig, gs[1, 0], C, 'tip deflection by method (log scale)', False)
    labs = ['direct\nintegration', 'Castigliano\ndU/dP', 'unit-load', '+shear\n(Timoshenko)', 'spring-joint\nmodel', 'total\n(joint+beam)']
    vals = [EN['d_direct'], EN['d_cast'], EN['d_unit'], EN['d_timo'], EN['d_joint'], EN['d_total']]
    cols = [COL['a'], COL['b'], COL['c'], COL['e'], COL['d'], COL['f']]
    ax.bar(range(6), np.array(vals) * 1e3, color=cols)
    ax.set_yscale('log'); ax.set_xticks(range(6)); ax.set_xticklabels(labs, fontsize=6, color=C['fg'])
    for i, v in enumerate(vals): ax.text(i, v * 1e3 * 1.15, f'{v * 1e3:.3g}', ha='center', fontsize=6, color=C['fg'])
    _style(ax, C, None, 'deflection [mm]')
    ax = _ax(fig, gs[1, 1], C, 'deflection vs tip force: beam vs spring-joint finger', False)
    Pp = EN['Ps'][1:]
    ax.plot(Pp, EN['dU_beam'][1:] * 1e3, color=COL['a'], label='beam (Castigliano)')
    ax.plot(Pp, EN['dU_joint'][1:] * 1e3, color=COL['d'], label='spring joints')
    ax.plot(Pp, (EN['dU_beam'] + EN['dU_joint'])[1:] * 1e3, color=COL['f'], ls='--', label='series total')
    ax.axvline(P, color=COL['c'], lw=0.8)
    ax.set_yscale('log') if EN['d_joint'] > 20 * EN['d_cast'] else None
    ax.legend(fontsize=6, facecolor=C['bg'], labelcolor=C['fg'], edgecolor=C['grid'])
    _style(ax, C, 'P [N]', 'delta [mm]')
    ax = _ax(fig, gs[1, 2], C, 'results', True)
    s = (f"finger (straight, s = 0), 3 phalanges L = {'/'.join('%.0f' % (1e3 * l) for l in EN['Ls'])} mm, tip force P = {EN['P']:.2f} N\n"
         f"EI per phalanx [N.m2]: {', '.join('%.3g' % e for e in EN['EI'])}\n\n"
         f"(1) direct   EI w'' = M          delta = {EN['d_direct'] * 1e6:.5f} um   theta {EN['th_direct'] * 1e6:.4f} urad\n"
         f"(2) Castigliano U = int M^2/2EI dx\n"
         f"        dU/dP analytic          delta = {EN['d_cast'] * 1e6:.5f} um\n"
         f"        dU/dP central diff      delta = {EN['d_cast_fd'] * 1e6:.5f} um\n"
         f"        dU/dM0 (dummy moment)   theta {EN['th_cast'] * 1e6:.4f} urad\n"
         f"(3) unit load  int M m/EI dx     delta = {EN['d_unit'] * 1e6:.5f} um   theta {EN['th_unit'] * 1e6:.4f} urad\n"
         f"    max disagreement of the three: {EN['agree']:.1e} (relative)\n"
         f"shear energy  dU_s/dP = {EN['d_shear_cast'] * 1e6:.5f} um  (unit-load {EN['d_shear_unit'] * 1e6:.5f})\n\n"
         f"compliant finger model (hand.py, joint springs k = {', '.join('%.0f' % k for k in EN['kj'])} N.m/rad)\n"
         f"  Castigliano on the springs: sum (P l_k)^2/2k_k -> {EN['d_joint'] * 1e3:.4f} mm\n"
         f"  hand.compliance_along_normal(0) x P        -> {EN['d_hand'] * 1e3:.4f} mm\n"
         f"  same at the current curl s = {EN['s_now']:.2f}         -> {EN['d_hand_now'] * 1e3:.4f} mm\n"
         f"joint / beam deflection = {EN['d_joint'] / max(EN['d_cast'], 1e-30):.0f} : 1  -> the finger is a RIGID-LINK + SPRING\n"
         f"mechanism; phalanx bending is a {100 * EN['d_cast'] / max(EN['d_total'], 1e-30):.2f} % correction.")
    _txt(ax, s, C, 6.3)
    fig.suptitle(f"Finger deflection by three methods ({par['fsec']})", color=C['fg'], fontsize=10)
    return EN


def draw_p5_failure(fig, ctx, par, C):
    fs = failure_study(ctx, par['fos'])
    th = par['theory']
    dc = decision(fs, th)
    gs = _grid(fig, 2, 3)
    ax = _ax(fig, gs[0, 0], C, 'same contact, five theories', True)
    basis = {'Rankine': 'max principal stress', 'Tresca': 'max shear stress', 'Von Mises': 'distortion energy',
             'St. Venant': 'max principal strain', 'Mohr-Coulomb': 'different st / sc'}
    sc_, un = _unit([fs['ref']])
    lines = [f"strength: {'brittle' if fs['brittle'] else 'ductile'}   st = {fs['st'] / sc_:.4g}  sc = {fs['sc'] / sc_:.4g} {un}",
             f"principal stresses at N = {ctx['N']:.2f} N:  {fs['p'][0] / sc_:.3f}  {fs['p'][1] / sc_:.3f}  {fs['p'][2] / sc_:.3f} {un}", '',
             f"{'theory':<13}{'evaluates':<22}{'eq. stress':>11}{'util.':>7}{'FoS':>7}{'N_perm':>9}", '-' * 69]
    for t in THEORIES:
        c = fs['cur'][t]
        eq = f"{c['eq'] / sc_:.3f}" if not math.isnan(c['eq']) else '  --'
        fosv = (1.0 / c['u']) if c['u'] > 1e-12 else float('inf')
        Np = fs['N_perm'][t]
        lines.append(f"{t:<13}{basis[t]:<22}{eq:>11}{c['u']:7.2f}{(format(fosv, '7.1f') if fosv < 1000 else format(fosv, '7.1e'))}{('inf' if math.isinf(Np) else format(Np, '.2f')):>9}")
    lines += ['', f"N_perm = force per pad at which the theory reaches FoS = {fs['fos']:.2f}", 'Mohr-Coulomb = Tresca when st = sc (ductile).',
              'Brittle solids: the ductile theories use st as reference.']
    _txt(ax, '\n'.join(lines), C, 6.5, wrap=False, fs_min=4.2)
    ax = _ax(fig, gs[0, 1], C, 'permissible grip force by theory vs no-slip force', False)
    cap = fs['F_other']
    vals = [min(fs['N_perm'][t], 3 * max(cap, 1)) for t in THEORIES]
    cols = [COL['d'] if t == th else COL['a'] for t in THEORIES]
    left_min = 1e-2
    ax.barh(range(5), [max(v - left_min, 1e-9) for v in vals], left=left_min, color=cols)
    ax.set_yticks(range(5)); ax.set_yticklabels(THEORIES, fontsize=7, color=C['fg'])
    ax.axvline(fs['N_slip'] * SLIP_SF, color=COL['c'], lw=1.5); ax.text(fs['N_slip'] * SLIP_SF, 4.62, f'slip x{SLIP_SF} ', color=COL['c'], fontsize=7, ha='right', va='bottom')
    if ctx['N'] > left_min:
        ax.axvline(ctx['N'], color='w', lw=0.8, ls=':')
        ax.text(ctx['N'], 4.62, ' now', color=C['fg'], fontsize=7, ha='left', va='bottom')
    ax.set_xscale('log')
    ax.set_xlim(left_min, max(max(vals) * 2.2, 10.0, cap * 1.3))
    ax.axvline(cap, color=COL['b'], lw=1.2, ls='--'); ax.text(cap, -0.62, 'gripper / shell limit ', color=COL['b'], fontsize=7, ha='right', va='top')
    for i, t in enumerate(THEORIES):
        Np = fs['N_perm'][t]
        ax.text(max(vals[i], left_min) * 1.2, i, ('inf' if math.isinf(Np) else f'{Np:.2f} N'), va='center', fontsize=7, color=C['fg'])
    ax.set_ylim(-1.2, 5.2)
    _style(ax, C, 'grip force per pad [N]  (log)', None)
    ax = _ax(fig, gs[0, 2], C, 'utilisation vs grip force (failure at 1)', False)
    cs = (COL['b'], COL['a'], COL['d'], COL['e'], COL['c'])
    for t, col in zip(THEORIES, cs):
        ax.loglog(fs['Ns'], np.maximum(fs['U'][t], 1e-6), color=col, lw=2.4 if t == th else 1.2, label=t)
    ax.axhline(1.0, color=C['fg'], lw=0.8); ax.axhline(1.0 / fs['fos'], color=C['fg'], lw=0.8, ls='--')
    if ctx['N'] > left_min:
        ax.axvline(ctx['N'], color='w', lw=0.8, ls=':')
    ax.set_ylim(1e-3, 30)
    ax.axvspan(Ns_lo := float(fs['Ns'][0]), fs['N_slip'], color=COL['d'], alpha=0.13)
    ax.text(fs['N_slip'] * 0.92, 1.6e-3, 'part slips', color=COL['d'], fontsize=7, ha='right')
    ax.legend(fontsize=6, facecolor=C['bg'], labelcolor=C['fg'], edgecolor=C['grid'])
    _style(ax, C, 'grip force per pad N [N]', 'equivalent stress / strength')
    ax = _ax(fig, gs[1, 0], C, 'utilisation at the current force', False)
    us = [fs['cur'][t]['u'] for t in THEORIES]
    ax.bar(range(5), us, color=[COL['d'] if t == th else COL['a'] for t in THEORIES])
    ax.axhline(1.0, color=C['fg'], lw=0.8); ax.axhline(1.0 / fs['fos'], color=C['fg'], lw=0.8, ls='--')
    ax.set_xticks(range(5)); ax.set_xticklabels(THEORIES, rotation=20, fontsize=6, color=C['fg'])
    _style(ax, C, None, 'utilisation')
    ax = _ax(fig, gs[1, 1], C, "Mohr circles with Tresca / Mohr-Coulomb limits", False)
    p = fs['p']
    circ = [((p[0] + p[1]) / 2, abs(p[0] - p[1]) / 2, 'C12'), ((p[1] + p[2]) / 2, abs(p[1] - p[2]) / 2, 'C23'), ((p[0] + p[2]) / 2, abs(p[0] - p[2]) / 2, 'C13')]
    _mohr_circles(ax, circ, C, sc_)
    st, sc2 = fs['st'], fs['sc']
    cosa = (sc2 - st) / (sc2 + st); sina = math.sqrt(max(0.0, 1 - cosa ** 2))
    d_ = st / 2 + (st / 2) * cosa
    sg = np.linspace(-sc2 * 1.05, st * 1.05, 50)
    tau_l = (d_ - sg * cosa) / max(sina, 1e-6)
    ax.plot(sg / sc_, tau_l / sc_, color=COL['d'], lw=1.5, label='Mohr-Coulomb')
    ax.plot(sg / sc_, -tau_l / sc_, color=COL['d'], lw=0.8)
    ax.axhline(fs['ref'] / 2 / sc_, color=COL['f'], lw=1, ls='--', label='Tresca  tau = sy/2')
    tt = np.linspace(0, math.pi, 100)
    for c_, r_ in ((st / 2, st / 2), (-sc2 / 2, sc2 / 2)):
        ax.plot((c_ + r_ * np.cos(tt)) / sc_, r_ * np.sin(tt) / sc_, color=COL['g'], lw=0.6, ls=':')
    ax.axhline(0, color=C['fg'], lw=0.5)
    ax.set_ylim(-0.1 * fs['ref'] / sc_, 0.9 * max(fs['ref'], abs(p[0] - p[2])) / sc_ + 0.1 * fs['ref'] / sc_)
    ax.legend(fontsize=6, facecolor=C['bg'], labelcolor=C['fg'], edgecolor=C['grid'])
    _style(ax, C, f'sigma [{un}]', f'tau [{un}]')
    ax = _ax(fig, gs[1, 2], C, f'DECISION  using {th}  (FoS {fs["fos"]:.2f})', True)
    col = COL['c'] if dc['ok'] else COL['d']
    ver = 'SAFE GRIP WINDOW' if dc['ok'] else 'NO SAFE GRIP'
    lines = [ver, '',
             f"force needed to hold (no slip x {SLIP_SF}):   {dc['N_lo']:.3f} N",
             f"force allowed by {th}:  {fs['N_perm'][th]:.3f} N" if not math.isinf(fs['N_perm'][th]) else f"force allowed by {th}:  no limit below 500 N",
             f"gripper / shell limit:                {fs['F_other']:.2f} N",
             f"ceiling used:  {dc['N_hi']:.3f} N   ({dc['limiter']})",
             f"window  {dc['N_lo']:.3f} -> {dc['N_hi']:.3f} N   ratio {dc['N_hi'] / max(dc['N_lo'], 1e-12):.1f}", '']
    if dc['ok']: lines.append(f"recommended grip force: {dc['N_rec']:.3f} N per pad")
    else: lines.append('the object would fail (or the pad slip) before it is held:\nreduce mass, add friction, use a larger pad (R) or more fingers.')
    lines += ['', 'spread of the permissible force over the five theories:']
    fin = [fs['N_perm'][t] for t in THEORIES if not math.isinf(fs['N_perm'][t])]
    if fin: lines.append(f"  min {min(fin):.3f} N   max {max(fin):.3f} N   (x{max(fin) / max(min(fin), 1e-12):.2f})")
    _txt(ax, lines[0], C, 11, col=col, wrap=False, fs_min=7, fontweight='bold')
    _txt(ax, '\n'.join(lines[1:]), C, 6.7, y=0.84)
    fig.suptitle(f"Permissible grip force by failure criterion - {ctx['o']['label']}", color=C['fg'], fontsize=10)
    return fs, dc


PAGES = (('STRESS STATE', draw_p1_tensor), ('ELEMENT EQUILIBRIUM', draw_p1_equilibrium), ('WRIST SHAFT', draw_p2_wrist),
         ('BEAMS', draw_p3_beams), ('FINGER DEFLECTION', draw_p4_energy), ('FAILURE LIMITS', draw_p5_failure))


def draw_page(fig, page, ctx, par, C):
    p = dict(DEFAULTS, **par)
    return PAGES[page][1](fig, ctx, p, C)


def _fmt_n(x): return 'no limit below 500 N' if math.isinf(x) else f'{x:.3f} N'


def info_text(ctx, par=None):
    """Plain-text report of the same numbers the graph pages show (shown under the graph button in the dashboard)."""
    p = dict(DEFAULTS, **(par or {}))
    o = ctx['o']
    R = tensor_report(ctx['ms'], o, p['theta'], p['phi'])
    sc, un = _unit(R['S'])
    S = wrist_state(o['m'])
    E_ = equilibrium_element(ctx['ms'], o)
    arm = beam_case('arm link', ctx, p['fsec'])
    fin = beam_case('finger', ctx, p['fsec'])
    EN = energy_finger(ctx, p['fsec'])
    fs = failure_study(ctx, p['fos'])
    dc = decision(fs, p['theory'])
    L = []
    L.append('STRESS AND STRAIN STATE  (point 0.48a under the pad, pad force %.2f N)' % ctx['N'])
    L.append('  principal stresses [%s]   %.3f / %.3f / %.3f    max shear %.3f   mean (I1/3) %.3f' % (un, R['p'][0] / sc, R['p'][1] / sc, R['p'][2] / sc, R['tau_max'] / sc, R['sm'] / sc))
    L.append('  Von Mises %.3f %s   octahedral shear %.3f %s   Lode angle %.1f deg' % (R['svm'] / sc, un, R['tau_oct'] / sc, un, R['lode']))
    L.append('  plane at %.0f deg from the contact plane:  normal %.3f  shear %.3f %s' % (p['theta'], R['sn'] / sc, R['tau'] / sc, un))
    L.append('  strains: volumetric %.4e   max shear %.4e   strain energy %.3f J/m3 (volume change %.3f + shape change %.3f)' % (R['e_v'], R['gam_max'], R['U'], R['U_h'], R['U_d']))
    L.append('')
    L.append('ELEMENT EQUILIBRIUM  (differential element in the contact field)')
    L.append('  residual of the force balance / largest term:  x %.2e   y %.2e   z %.2e' % tuple(E_['rel']))
    L.append('  own weight / stress gradient = %.1e  (negligible inside the patch)' % E_['body_ratio'])
    L.append('')
    L.append('WRIST SHAFT  (OD %.0f / ID %.0f mm, %.0f mm long)' % (2e3 * S['ro'], 2e3 * S['ri'], 1e3 * S['L']))
    L.append('  loads: bending %.3f N.m, shear force %.1f N, axial %.1f N, torque %.1f N.mm' % (S['M'], S['V'], S['F'], S['T'] * 1e3))
    L.append('  critical point stress %.2f MPa, shear %.2f MPa -> Von Mises %.2f MPa (FoS %.1f), Tresca %.2f MPa (FoS %.1f)' % (S['crit']['sig'] / 1e6, S['crit']['tau'] / 1e6, S['crit']['vm'] / 1e6, S['fos_vm'], S['tresca_max'] / 1e6, S['fos_tr']))
    L.append('  twist %.4f deg, tip deflection %.1f um, heaviest part at FoS %.1f: %.1f kg' % (S['twist_deg'], S['defl'] * 1e6, S['W']['fos_target'], wrist_m_allow(S['W']['fos_target'])))
    L.append('')
    L.append('BEAMS')
    for b in (arm, fin):
        L.append('  %s' % b['title'])
        L.append('    root bending stress %.3f MPa, peak shear stress %.3f MPa, tip deflection %.4g mm (shear share %.2f %%)' % (b['sig_root'] / 1e6, b['tau_root'] / 1e6, b['d_ti'] * 1e3, 100 * b['shear_share']))
    L.append('')
    L.append('FINGER DEFLECTION  (%s, tip force %.2f N)' % (p['fsec'], EN['P']))
    L.append('  phalanx bending %.5f mm by direct integration, Castigliano and unit load (agree to %.0e)' % (EN['d_cast'] * 1e3, EN['agree']))
    L.append('  joint springs %.4f mm  ->  joints give %.0fx the bending deflection' % (EN['d_joint'] * 1e3, EN['d_joint'] / max(EN['d_cast'], 1e-30)))
    L.append('')
    L.append('FAILURE LIMITS  (safety factor %.2f)' % p['fos'])
    L.append('  largest grip force per pad that is still safe:  ' + '   '.join('%s %s' % (t, ('no limit' if math.isinf(fs['N_perm'][t]) else ('over limit even when just holding' if fs['N_perm'][t] <= fs['N_slip'] * 1.001 else '%.2f N' % fs['N_perm'][t]))) for t in THEORIES))
    L.append('  needed to hold without slipping (x%.2f): %.3f N     gripper / shell limit: %.2f N' % (SLIP_SF, dc['N_lo'], fs['F_other']))
    L.append('  verdict with %s: %s' % (p['theory'], ('safe window %.3f - %.3f N' % (dc['N_lo'], dc['N_hi'])) if dc['ok'] else 'NO SAFE GRIP'))
    return '\n'.join(L)


def make_ctx(key='apple', pad='small', N=None, n_f=None):
    n_f = n_f or int(HD.HAND['n_fingers'])
    o = OBJECTS[key]
    tr = truth(key, pad, n=n_f)
    N = min(1.5 * tr['N_slip'], GRIPPER['F_max']) if N is None else N
    ms = MOS.mos_state(N, o['m'] * G / n_f, key, pad)
    return dict(key=key, o=o, ms=ms, N=ms['N'], pad=pad, n_f=n_f, s_curl=0.0)


def main(argv=None):
    import sys, os
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib.figure import Figure
    a = list(sys.argv[1:] if argv is None else argv)
    key = a[0] if len(a) > 0 else 'apple'
    pad = a[1] if len(a) > 1 else 'small'
    out = a[2] if len(a) > 2 else 'nptel_report'
    os.makedirs(out, exist_ok=True)
    ctx = make_ctx(key, pad)
    C = dict(bg='#0f172a', fg='#e2e8f0', grid='#334155')
    fig = Figure(figsize=(14.5, 8.2), dpi=110, facecolor=C['bg'])
    for i, (nm, _) in enumerate(PAGES):
        draw_page(fig, i, ctx, dict(comp='arm link'), C)
        fig.savefig(os.path.join(out, f'{key}_{i + 1}_{nm.lower().replace(" ", "_")}.png'), facecolor=C['bg'])
    draw_page(fig, 3, ctx, dict(comp='finger'), C)
    fig.savefig(os.path.join(out, f'{key}_4b_beams_finger.png'), facecolor=C['bg'])
    print(info_text(ctx))
    print('PNG pages written to', os.path.abspath(out))


if __name__ == '__main__':
    main()

