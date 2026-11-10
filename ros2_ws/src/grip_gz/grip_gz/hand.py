"""hand.py - n-finger 3-phalanx curling hand kinematics, Jacobian statics, compliance, and grasp planner."""
import math
import numpy as np

G = 9.81
HAND = dict(
    n_fingers=5,
    L=(0.060, 0.045, 0.038),
    r_base=0.095,
    syn=(1.0, 1.0, 0.8),
    q_max=math.radians(80.0),
    k_joint=(30.0, 20.0, 12.0),
    c_joint=(0.05, 0.035, 0.02),
    tau_max=(12.0, 8.0, 4.0),
    pad_off=0.011,
    pad_back=0.012,
    palm_clear=0.012,
    palm_r=0.115,
    palm_thick=0.060,
    hover_margin=0.03,
    table_clear=0.004,
    approach_gap=0.003,
    env=(0.0155, 0.0145, 0.0140, 0.0100),
)

def n_f(): return int(HAND['n_fingers'])
def azimuths(n=None, phi0=None):
    n = n or n_f()
    phi0 = math.pi / 2.0 if phi0 is None else phi0
    return [phi0 + 2.0 * math.pi * i / n for i in range(n)]

def joint_angles(s, h=HAND):
    s = max(0.0, min(1.0, s))
    return tuple(k * s * h['q_max'] for k in h['syn'])

def finger_points(s, h=HAND):
    q = joint_angles(s, h)
    pts = [(h['r_base'], 0.0)]
    psi, r, z = 0.0, h['r_base'], 0.0
    for L, qk in zip(h['L'], q):
        psi += qk
        r -= L * math.sin(psi)
        z -= L * math.cos(psi)
        pts.append((r, z))
    return pts, q, psi

def contact_normal(): return -1.0, 0.0

def contact_point(s, h=HAND):
    pts, q, psi = finger_points(s, h)
    tr, tz = pts[-1]
    r = tr + h['pad_back'] * math.sin(psi) - h['pad_off'] * math.cos(psi)
    z = tz + h['pad_back'] * math.cos(psi) + h['pad_off'] * math.sin(psi)
    return r, z, psi

def s_limit(h=HAND, n=401):
    best_s, best_r = 0.0, contact_point(0.0, h)[0]
    for i in range(1, n):
        s = i / (n - 1.0)
        r = contact_point(s, h)[0]
        if r < best_r - 1e-12: best_s, best_r = s, r
        else: break
    return best_s

def r_min(h=HAND): return contact_point(s_limit(h), h)[0]

def solve_curl(rho, h=HAND):
    r0 = contact_point(0.0, h)[0]
    if rho >= r0: return 0.0
    sl = s_limit(h)
    if rho < contact_point(sl, h)[0]: return None
    lo, hi = 0.0, sl
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        if contact_point(mid, h)[0] > rho: lo = mid
        else: hi = mid
    return 0.5 * (lo + hi)

def jacobian(s, h=HAND):
    pts, q, psi = finger_points(s, h)
    cr, cz, _ = contact_point(s, h)
    cols = []
    for k in range(3):
        jr, jz = pts[k]
        cols.append((cz - jz, -(cr - jr)))
    return cols

def joint_torques(F_n, s, h=HAND):
    nr, nz = contact_normal()
    return [abs(F_n * (c[0] * nr + c[1] * nz)) for c in jacobian(s, h)]

def force_available(s, h=HAND):
    nr, nz = contact_normal()
    best = float('inf')
    for k, c in enumerate(jacobian(s, h)):
        lever = abs(c[0] * nr + c[1] * nz)
        if lever > 1e-6:
            best = min(best, h['tau_max'][k] / lever)
    return best

def compliance_along_normal(s, h=HAND):
    nr, nz = contact_normal()
    return sum((c[0] * nr + c[1] * nz) ** 2 / h['k_joint'][k] for k, c in enumerate(jacobian(s, h)))

def box_radius(phi, sx, sy):
    c, s_ = abs(math.cos(phi)), abs(math.sin(phi))
    cands = []
    if c > 1e-9: cands.append((sx / 2.0) / c)
    if s_ > 1e-9: cands.append((sy / 2.0) / s_)
    return min(cands)

def object_radii(o, phis):
    if o['shape'] == 'box': return [box_radius(p, o['dims'][0], o['dims'][1]) for p in phis]
    return [o['dims'][0]] * len(phis)

def obj_height(o): return o['dims'][2] if o['shape'] == 'box' else o['dims'][1]
def r_open(h=HAND): return contact_point(0.0, h)[0]
def drop_open(h=HAND): return -contact_point(0.0, h)[1]

def lowest_z(s, h=HAND):
    pts, q, psi = finger_points(s, h)
    return min(z - r for (_, z), r in zip(pts, h['env']))

def s_for_height(z_palm, h=HAND, n=200):
    sl = s_limit(h)
    for i in range(n + 1):
        s = sl * i / n
        if z_palm + lowest_z(s, h) >= h['table_clear'] - 1e-9:
            return s
    return None

def preshape_curl(z_palm, s_clear, h=HAND):
    s = s_for_height(z_palm, h)
    return min(s if s is not None else s_limit(h), s_clear)

def curl_tables(h=HAND, n=241):
    ss = np.linspace(0.0, s_limit(h), n)
    r = np.array([contact_point(float(s), h)[0] for s in ss])
    psi = np.array([contact_point(float(s), h)[2] for s in ss])
    return r_open(h) - r, ss, psi

def plan_grasp(o, h=HAND, phi0=None):
    n = int(h['n_fingers'])
    phis = azimuths(n, phi0)
    rhos = object_radii(o, phis)
    hh = obj_height(o)
    msgs, s_c = [], []
    ro = r_open(h)
    for rho in rhos:
        if rho > ro + 1e-6:
            msgs.append('object too wide: radius %.1f mm > hand opening %.1f mm' % (rho * 1000, ro * 1000))
        s = solve_curl(rho, h)
        if s is None:
            msgs.append('object too small for this hand (radius %.1f mm)' % (rho * 1000))
            s = 1.0
        s_c.append(s)
    sm = sum(s_c) / n
    cz_rel = sum(contact_point(s, h)[1] for s in s_c) / n
    psi = sum(contact_point(s, h)[2] for s in s_c) / n
    z_pref = o['grasp_z'] if o.get('grasp_z') is not None else 0.55 * hh
    z_c = max(min(max(z_pref, hh + h['palm_clear'] + cz_rel), hh - 0.006), 0.010)
    z_palm = z_c - cz_rel
    s_clear = []
    for rho, s in zip(rhos, s_c):
        sc_ = solve_curl(rho + h['approach_gap'], h)
        s_clear.append(s if sc_ is None else min(sc_, s))
    z_floor = max(h['table_clear'] - lowest_z(s, h) for s in list(s_clear) + list(s_c))
    raised = z_palm < z_floor - 1e-9
    if raised:
        z_palm = z_floor
        z_c = z_palm + cz_rel
    if z_palm < hh + h['palm_clear'] - 1e-6: msgs.append('palm would touch the object top')
    if z_c > hh - 0.002: msgs.append('object too short: the fingertips cannot reach it without touching the table')
    return dict(n=n, phis=phis, rhos=rhos, s_contact=s_c, s_mean=sm, z_contact=z_c, z_palm=z_palm, psi=psi,
                ok=not msgs, msgs=msgs, r_open=ro, s_clear=s_clear, z_floor=z_floor, grasp_raised=raised,
                z_hover=hh + drop_open(h) + h['hover_margin'])

def slip_force(o, n): return o['m'] * G / (n * o['mu'])

def _quat_zy(phi, beta):
    cz, sz = math.cos(phi / 2.0), math.sin(phi / 2.0)
    cy, sy = math.cos(beta / 2.0), math.sin(beta / 2.0)
    return (-sz * sy, cz * sy, cy * sz, cz * cy)

def phalanx_poses(tcp_x, tcp_z, s_list, z0, phis, h=HAND):
    out = {}
    for i, (s, phi) in enumerate(zip(s_list, phis)):
        pts, q, psi = finger_points(s, h)
        psi_k = 0.0
        for k in range(3):
            psi_k += q[k]
            r, zr = pts[k]
            beta = math.pi / 2.0 + psi_k
            out['grip_f%d_p%d' % (i, k + 1)] = (tcp_x + r * math.cos(phi), r * math.sin(phi), z0 + tcp_z + zr) + _quat_zy(phi, beta)
    return out
