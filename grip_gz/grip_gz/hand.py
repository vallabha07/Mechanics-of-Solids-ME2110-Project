"""hand.py - n-finger adaptive hand (v3): kinematics, statics, compliance and a geometry-true grasp planner.

Design (after Barrett Hand / Robotiq 3F / Yale OpenHand practice):
  * Opposed layout: floor(n/2) "thumbs" on one side of the palm, the rest on the other.  Every finger has a
    SPREAD joint (vertical axis at its base) so its closing plane can be aimed, and the whole hand has a wrist-YAW
    joint.  Aiming all fingers at one point gives a radial wrap (cylinders, spheres); leaving them parallel gives
    a pinch (boxes, brackets, plates), exactly the Robotiq "wide / pinch" idea.
  * Each finger is a 3-phalanx tendon-synergy finger ending in a compliant DOME fingertip (sphere, R = tip_r).
    A sphere touches a face at one point whatever the face orientation, so no edge or corner of the finger can
    dig into the part the way the old flat/oblique pad did.
  * Finger forces are weighted (g_i) so the horizontal force/torque balance of the object is exactly zero.
The planner searches yaw x aim-point on the REAL part geometry (signed-distance field), rejects grasps whose
contact normal is outside the friction cone, solves the force balance and the minimum holding force with the
real contact layout and centre of mass, and finally verifies swept collision clearance.
"""
import math
import numpy as np

from .shapes import get_shape

G = 9.81
HAND = dict(
    n_fingers=5,
    L=(0.060, 0.045, 0.038),
    r_base=0.085,            # |x| of the finger bases from the palm centre
    e_pitch=0.036,           # lateral pitch of fingers inside one group
    syn=(1.0, 1.0, 0.8),
    q_max=math.radians(80.0),
    k_joint=(30.0, 20.0, 12.0),
    c_joint=(0.05, 0.035, 0.02),
    tau_max=(12.0, 8.0, 4.0),
    tip_r=0.011,             # dome fingertip radius
    pad_off=0.011,
    pad_back=0.012,
    palm_clear=0.012,
    palm_x=0.108, palm_y=0.082, palm_thick=0.020,
    hover_margin=0.03,
    table_clear=0.004,
    approach_gap=0.003,
    env=(0.0155, 0.0145, 0.0140, 0.0115),
    spread_max=math.radians(50.0),
    cap_r=(0.013, 0.0125, 0.0105),   # capsule radius per phalanx for finger-finger clearance (dome = tip_r)
)
BODY = (  # per phalanx: (x0, x1, width, thickness, hub radius)
    (0.010, 0.050, 0.020, 0.022, 0.0145),
    (0.010, 0.035, 0.020, 0.022, 0.0135),
    (0.010, 0.014, 0.019, 0.022, 0.0125),
)
HUB_W = 0.026
G_MIN = 0.15


def n_f(): return int(HAND['n_fingers'])


def layout(n=None):
    n = n or n_f()
    nt = n // 2
    nf = n - nt
    e, d = HAND['e_pitch'], HAND['r_base']
    out = []
    for grp, (k, x, phi0) in enumerate(((nt, -d, math.pi), (nf, d, 0.0))):
        for j in range(k):
            out.append(dict(base=(x, (j - (k - 1) / 2.0) * e), phi0=phi0, group=grp))
    return out


# ------------------------------------------------------------------------------------------ finger kinematics
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


def dome_center(s, h=HAND):
    pts, q, psi = finger_points(s, h)
    tr, tz = pts[-1]
    return tr + h['pad_back'] * math.sin(psi), tz + h['pad_back'] * math.cos(psi), psi


def contact_normal(): return -1.0, 0.0


def contact_point(s, h=HAND):
    pts, q, psi = finger_points(s, h)
    tr, tz = pts[-1]
    r = tr + h['pad_back'] * math.sin(psi) - h['pad_off'] * math.cos(psi)
    z = tz + h['pad_back'] * math.cos(psi) + h['pad_off'] * math.sin(psi)
    return r, z, psi


def _hkey(h):
    return (h['r_base'], h['q_max'], tuple(h['L']), tuple(h['syn']), h['pad_back'], h['pad_off'])


_SL = {}
def s_limit(h=HAND, n=401):
    """Curl at which the dome centre stops moving inward.  Pure function of the finger geometry, so it is computed once
    (it used to be recomputed - 400 kinematics evaluations - every simulation tick and every planner call)."""
    key = (_hkey(h), n)
    v = _SL.get(key)
    if v is None:
        best_s, best_r = 0.0, dome_center(0.0, h)[0]
        for i in range(1, n):
            s = i / (n - 1.0)
            r = dome_center(s, h)[0]
            if r < best_r - 1e-12: best_s, best_r = s, r
            else: break
        v = _SL[key] = best_s
    return v


def r_open(h=HAND): return contact_point(0.0, h)[0]
def drop_open(h=HAND): return -contact_point(0.0, h)[1]


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


_RELAX = [False]


def lowest_z(s, h=HAND):
    pts, q, psi = finger_points(s, h)
    if _RELAX[0]:                                                            # small parts: the fingertip is the dome, not the phalanx end
        low = min(z - r for (_, z), r in zip(pts[:3], h['env'][:3]))
        return min(low, dome_center(s, h)[1] - h['tip_r'])
    return min(z - r for (_, z), r in zip(pts, h['env']))


_LZ = {}
def _lz_table(h, n=200):
    key = (_hkey(h), tuple(h['env']), h['tip_r'], _RELAX[0], n)
    t = _LZ.get(key)
    if t is None:
        sl = s_limit(h)
        ss = np.array([sl * i / n for i in range(n + 1)])
        t = _LZ[key] = (ss, np.array([lowest_z(float(s), h) for s in ss]))
    return t


def s_for_height(z_palm, h=HAND, n=200):
    ss, lz = _lz_table(h, n)
    ok = (z_palm + lz) >= h['table_clear'] - 1e-9
    return float(ss[int(np.argmax(ok))]) if ok.any() else None


def preshape_curl(z_palm, s_clear, h=HAND):
    s = s_for_height(z_palm, h)
    return min(s if s is not None else s_limit(h), s_clear)


_CT = {}
def curl_tables(h=HAND, n=241):
    """x = closing travel of the dome centre (m), s = curl, psi = tip tilt.  Cached (read-only arrays)."""
    key = (_hkey(h), n)
    t = _CT.get(key)
    if t is None:
        ss = np.linspace(0.0, s_limit(h), n)
        rc = np.array([dome_center(float(s), h)[0] for s in ss])
        psi = np.array([dome_center(float(s), h)[2] for s in ss])
        t = _CT[key] = (rc[0] - rc, ss, psi)
        for a in t: a.setflags(write=False)
    return t


_TAB = {}
def _dome_tab(K=56):
    key = (K, HAND['r_base'], HAND['q_max'], HAND['L'])
    t = _TAB.get(key)
    if t is None:
        S = np.linspace(0.0, s_limit(), K)
        rc = np.array([dome_center(float(s))[0] for s in S])
        zc = np.array([dome_center(float(s))[1] for s in S])
        t = _TAB[key] = (S, rc, zc)
    return t


# ------------------------------------------------------------------------------------------ 3-D geometry
def spread_angles(lay, Q):
    """Plane azimuth phi_i and spread sigma_i.  Q = (Q_group0, Q_group1); each is an aim point (x, y) in the hand
    frame the group's fingers converge on, or None for parallel closing planes."""
    phis, sig = [], []
    for f in lay:
        q = Q[f['group']] if Q is not None else None
        phi = f['phi0'] if q is None else math.atan2(f['base'][1] - q[1], f['base'][0] - q[0])
        s = (phi - f['phi0'] + math.pi) % (2 * math.pi) - math.pi
        phis.append(f['phi0'] + s)
        sig.append(s)
    return phis, sig


def _local_templates():
    tpl = []
    for k in range(3):
        x0, x1, w, t, hub = BODY[k]
        L = HAND['L'][k]
        pts = []
        nx = max(2, int(math.ceil((x1 - x0) / 0.005)) + 1)
        for x in np.linspace(x0, x1, nx):
            for y in (-w / 2, 0.0, w / 2):
                for z in (-t / 2, 0.0, t / 2):
                    pts.append((x, y, z))
        if k < 2:                                               # end block of phalanx 1,2
            for x in (L - 0.018, L - 0.006):
                for y in (-0.012, 0.0, 0.012):
                    for z in (-0.013, 0.0, 0.013):
                        pts.append((x, y, z))
        for a in np.linspace(0.0, 2 * math.pi, 8, endpoint=False):  # joint hub (axis = lateral y)
            for y in (-HUB_W / 2, 0.0, HUB_W / 2):
                pts.append((hub * math.cos(a), y, hub * math.sin(a)))
        tpl.append(np.array(pts))
    return tpl


_TPL = []
def finger_geometry(base, phi, s, h=HAND):
    """Hand-frame geometry of one finger: rigid sample points (N,3), dome centre (3,), capsule segments (3,2,3)."""
    if not _TPL: _TPL.append(_local_templates())
    pts, q, psi_tot = finger_points(s, h)
    u = np.array([math.cos(phi), math.sin(phi), 0.0])
    v = np.array([-math.sin(phi), math.cos(phi), 0.0])
    b = np.array([base[0], base[1], 0.0])

    def to_hand(r, z, lat=0.0):
        return b + (r - h['r_base']) * u + np.array([0.0, 0.0, z]) + lat * v

    rig, caps, psi = [], [], 0.0
    for k in range(3):
        psi += q[k]
        Jr, Jz = pts[k]
        d = np.array([-math.sin(psi), -math.cos(psi)])            # (r,z) direction along the phalanx
        nout = np.array([math.cos(psi), -math.sin(psi)])          # local +z (outward)
        T = _TPL[0][k]
        rr = Jr + T[:, 0] * d[0] + T[:, 2] * nout[0]
        zz = Jz + T[:, 0] * d[1] + T[:, 2] * nout[1]
        rig.append(b[None] + (rr - h['r_base'])[:, None] * u[None] + zz[:, None] * np.array([[0, 0, 1.0]]) + T[:, 1:2] * v[None])
        caps.append((to_hand(*pts[k]), to_hand(*pts[k + 1])))
    rc, zc, _ = dome_center(s, h)
    return np.vstack(rig), to_hand(rc, zc), np.array(caps)


def palm_points(h=HAND):
    xs = np.linspace(-h['palm_x'], h['palm_x'], 12)
    ys = np.linspace(-h['palm_y'], h['palm_y'], 8)
    return np.array([(x, y, z) for x in xs for y in ys for z in (0.0, h['palm_thick'])])


def _seg_seg_dist(p1, q1, p2, q2):
    """Min distance between segments p1q1 and p2q2 (3-vectors)."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    if a < 1e-12 and e < 1e-12: return float(np.linalg.norm(r))
    if a < 1e-12: s, t = 0.0, min(max(f / e, 0.0), 1.0)
    else:
        c = d1 @ r
        if e < 1e-12: t, s = 0.0, min(max(-c / a, 0.0), 1.0)
        else:
            b = d1 @ d2
            den = a * e - b * b
            s = min(max((b * f - c * e) / den, 0.0), 1.0) if den > 1e-12 else 0.0
            t = (b * s + f) / e
            if t < 0.0: t, s = 0.0, min(max(-c / a, 0.0), 1.0)
            elif t > 1.0: t, s = 1.0, min(max((b - c) / a, 0.0), 1.0)
    return float(np.linalg.norm((p1 + d1 * s) - (p2 + d2 * t)))


def finger_finger_gap(geoms, h=HAND):
    """Smallest surface gap between any two different fingers (capsules per phalanx, domes r=tip_r)."""
    best = 1.0
    cr = h['cap_r']
    n = len(geoms)
    for i in range(n):
        for j in range(i + 1, n):
            ci, cj = geoms[i][2], geoms[j][2]
            for a in range(3):
                for b in range(3):
                    gap = _seg_seg_dist(ci[a][0], ci[a][1], cj[b][0], cj[b][1]) - cr[a] - cr[b]
                    best = min(best, gap)
            best = min(best, float(np.linalg.norm(geoms[i][1] - geoms[j][1])) - 2 * h['tip_r'])
    return best


def hand_geometry(bases, phis, s_list):
    return [finger_geometry(b, p, s) for b, p, s in zip(bases, phis, s_list)]


def pose_clearance(sh, geoms, yaw, z_tcp, with_palm=True):
    """Clearances (m) of a hand pose against the part: rigid, per-finger dome, table, finger-finger."""
    c, s_ = math.cos(yaw), math.sin(yaw)
    Rm = np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1.0]])
    off = np.array([0.0, 0.0, z_tcp])
    rig = np.vstack([g[0] for g in geoms] + ([palm_points()] if with_palm else []))
    P = rig @ Rm.T + off
    d_rig = float(sh.sdf(P).min())
    dome = np.array([g[1] for g in geoms]) @ Rm.T + off
    d_dome = sh.sdf(dome) - HAND['tip_r']
    table = float(min(P[:, 2].min(), (dome[:, 2] - HAND['tip_r']).min()))
    return dict(rigid=d_rig, dome=d_dome, table=table, ff=finger_finger_gap(geoms))


def pose_clearance_fast(sh, geoms, yaw, z_tcp):
    """Cheaper per-tick check: rigid points + domes + table (finger-finger gap skipped, returned as None)."""
    c, s_ = math.cos(yaw), math.sin(yaw)
    Rm = np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1.0]])
    off = np.array([0.0, 0.0, z_tcp])
    P = np.vstack([g[0] for g in geoms]) @ Rm.T + off
    dome = np.array([g[1] for g in geoms]) @ Rm.T + off
    return dict(rigid=float(sh.sdf(P).min()), dome=sh.sdf(dome) - HAND['tip_r'],
                table=float(min(P[:, 2].min(), (dome[:, 2] - HAND['tip_r']).min())), ff=None)


# ------------------------------------------------------------------------------------------ statics
def balance_gains(A, gmin=G_MIN):
    """g >= gmin with A g = 0 (3 x n force/torque rows) and g as uniform as possible; None if impossible."""
    n = A.shape[1]
    fixed = np.zeros(n, dtype=bool)
    for _ in range(n + 1):
        free = ~fixed
        if not free.any(): return None
        r = -A[:, free] @ np.ones(free.sum())
        if fixed.any(): r = r - A[:, fixed] @ np.full(fixed.sum(), gmin)
        d = np.linalg.pinv(A[:, free]) @ r
        g = np.where(fixed, gmin, 0.0)
        g[free] = 1.0 + d
        if np.linalg.norm(A @ g) > 1e-6 * max(1.0, np.linalg.norm(g)): return None
        low = free & (g < gmin - 1e-9)
        if not low.any(): break
        fixed |= low
    else:
        return None
    if g.min() < gmin - 1e-6: return None
    return g / g.max()


def required_force(pts_xy, kappa, W, com_xy):
    """Smallest reference force F so that the weight W (acting at com) can be carried by vertical friction forces
    0 <= v_i <= kappa_i * F at contact points pts_xy.  Exact zonotope-membership test (degenerate pinch included)."""
    n = len(pts_xy)
    a = np.c_[np.ones(n), pts_xy[:, 0], pts_xy[:, 1]]
    b = W * np.array([1.0, com_xy[0], com_xy[1]])
    dirs = []
    for i in range(n):
        for j in range(i + 1, n):
            d = np.cross(a[i], a[j])
            nd = np.linalg.norm(d)
            if nd > 1e-9: dirs += [d / nd, -d / nd]
    if not dirs: return math.inf
    sv = np.linalg.svd(a, compute_uv=False)
    if n < 3 or sv[-1] < 1e-6:                           # contacts collinear / only two: flat zonotope
        d0 = dirs[0]
        off = abs(float(d0 @ b)) / W                          # COM distance from the pinch line: the part pivots about it
        pen = 1.0 + 60.0 * max(off - 0.002, 0.0)              # and hangs; torsional friction has to cope, so penalise
        dirs = []
        for i in range(n):
            d = np.cross(d0, a[i])
            nd = np.linalg.norm(d)
            if nd > 1e-9: dirs += [d / nd, -d / nd]
    best = 0.0
    for d in dirs:
        db = float(d @ b)
        if db <= 1e-12: continue
        cap = float((kappa * np.maximum(0.0, a @ d)).sum())
        if cap <= 1e-12: return math.inf
        best = max(best, db / cap)
    return best * (pen if (n < 3 or sv[-1] < 1e-6) else 1.0)


def friction_vmax(u_out, n_out, mu):
    """Per unit push: largest upward friction force v (along +z) a contact can add on top of a horizontal push -u,
    given outward surface normal n_out and friction mu.  < 0 when the push alone is outside the friction cone."""
    p = -u_out
    m = -n_out
    ez = np.array([0.0, 0.0, 1.0])
    N0, N1 = p @ m, ez @ m
    a = p - N0 * m
    b = ez - N1 * m
    A = b @ b - mu * mu * N1 * N1
    B = 2.0 * (a @ b - mu * mu * N0 * N1)
    C = a @ a - mu * mu * N0 * N0
    if N0 <= 0.0 or C > 0.0: return -1.0
    if abs(A) < 1e-12: return 1e3 if B < 0 else (-C / B if B > 0 else 1e3)
    disc = B * B - 4 * A * C
    if disc < 0: return -1.0
    r = (-B + math.sqrt(disc)) / (2 * A), (-B - math.sqrt(disc)) / (2 * A)
    return max(r)


def slip_force(o, n): return o['m'] * G / (n * o['mu'])


# ------------------------------------------------------------------------------------------ planner
_PLAN_CACHE = {}


def _cands(lay, yaws=None):
    yaws = list(range(0, 360, 10)) if yaws is None else yaws
    Qs = [None]
    pts = [(qx, qy) for qx in (-0.12, -0.09, -0.06, -0.03, 0.0, 0.03, 0.06) for qy in (-0.03, 0.0, 0.03)]
    Qs += [(p, p) for p in pts]                                   # every finger aimed at one point
    for qt in (0.12, 0.09, 0.06, 0.03):                           # thumbs and fingers each converge on their own point
        for qf in (-0.12, -0.09, -0.06, -0.03):
            Qs.append(((qf if False else qt, 0.0), (qf, 0.0)))
    Qs += [((qt, 0.0), None) for qt in (0.12, 0.06)] + [(None, (qf, 0.0)) for qf in (-0.12, -0.06)]
    out = []
    if len(lay) == 2:                                             # 2-finger pinch: both push lines on the line joining the bases
        a, b = lay
        w = lambda x: (x + math.pi) % (2 * math.pi) - math.pi
        pa = math.atan2(a['base'][1] - b['base'][1], a['base'][0] - b['base'][0])
        pb = math.atan2(b['base'][1] - a['base'][1], b['base'][0] - a['base'][0])
        sig = [w(pa - a['phi0']), w(pb - b['phi0'])]
        if max(abs(x) for x in sig) <= HAND['spread_max']:
            for yaw_deg in yaws:
                out.append(dict(Q='line', phi=[a['phi0'] + sig[0], b['phi0'] + sig[1]], sig=sig, yaw=math.radians(yaw_deg)))
    for Q in Qs:
        if Q is None: Q = (None, None)
        phis, sig = spread_angles(lay, Q)
        if max(abs(s) for s in sig) > HAND['spread_max'] + 1e-9: continue
        for yaw_deg in yaws:
            out.append(dict(Q=Q, phi=phis, sig=sig, yaw=math.radians(yaw_deg)))
    return out


def _find_contacts(sh, lay, cands, z_palm, R):
    """First dome-surface contact along the curl path for every (candidate, finger)."""
    S, rc, zc = _dome_tab()
    C, n, K = len(cands), len(lay), len(S)
    phi = np.array([c['phi'] for c in cands])
    yaw = np.array([c['yaw'] for c in cands])
    base = np.array([l['base'] for l in lay])
    dr = (rc - HAND['r_base'])[None, None, :]
    X = base[None, :, 0, None] + dr * np.cos(phi)[:, :, None]
    Y = base[None, :, 1, None] + dr * np.sin(phi)[:, :, None]
    cy, sy = np.cos(yaw)[:, None, None], np.sin(yaw)[:, None, None]
    Xo, Yo = cy * X - sy * Y, sy * X + cy * Y
    Zo = np.broadcast_to(z_palm + zc[None, None, :], Xo.shape)
    d = sh.sdf(np.stack([Xo, Yo, Zo], -1).reshape(-1, 3)).reshape(C, n, K)
    hit = d <= R
    free0 = d[:, :, 0] > R + 0.0015
    any_hit = hit.any(2)
    k = np.maximum(np.argmax(hit, axis=2), 1)
    d0 = np.take_along_axis(d, (k - 1)[:, :, None], 2)[:, :, 0]
    d1 = np.take_along_axis(d, k[:, :, None], 2)[:, :, 0]
    t = np.clip((d0 - R) / np.maximum(d0 - d1, 1e-12), 0.0, 1.0)
    s_c = S[k - 1] + t * (S[k] - S[k - 1])
    return s_c, any_hit & free0


def _dome_world(lay, cand, z_palm, i, s):
    rc, zc, _ = dome_center(float(s))
    phi = cand['phi'][i]
    px = lay[i]['base'][0] + (rc - HAND['r_base']) * math.cos(phi)
    py = lay[i]['base'][1] + (rc - HAND['r_base']) * math.sin(phi)
    c, sn = math.cos(cand['yaw']), math.sin(cand['yaw'])
    return np.array([c * px - sn * py, sn * px + c * py, z_palm + zc])


def _contact_state(sh, lay, cand, z_palm, s_c):
    c, s = math.cos(cand['yaw']), math.sin(cand['yaw'])
    Ry = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    cen = np.array([_dome_world(lay, cand, z_palm, i, s_c[i]) for i in range(len(lay))])
    u_o = np.array([Ry @ np.array([math.cos(p), math.sin(p), 0.0]) for p in cand['phi']])
    return cen, sh.normal(cen), u_o


def _evaluate(sh, o, lay, cand, z_palm, s_c, R):
    mu, W = o['mu'], o['m'] * G
    cen, nrm, u = _contact_state(sh, lay, cand, z_palm, s_c)
    n = len(lay)
    kap = np.array([friction_vmax(u[i], nrm[i], mu) for i in range(n)])
    if (kap < 0).any(): return None, 'contact normal outside friction cone'
    A = np.zeros((3, n))
    for i in range(n):
        A[0, i], A[1, i] = -u[i, 0], -u[i, 1]
        A[2, i] = -(cen[i, 0] * u[i, 1] - cen[i, 1] * u[i, 0])
    g = balance_gains(A)
    if g is None: return None, 'no force balance'
    pts = cen - R * nrm
    F = required_force(pts[:, :2], kap * g, W, sh.com[:2])
    if not math.isfinite(F): return None, 'centre of mass outside the contact support'
    cos_t = np.array([float(nrm[i] @ u[i]) for i in range(n)])
    tilt = np.degrees(np.arccos(np.clip(cos_t, -1, 1)))
    edge = _edge_gap(sh, cen, nrm, R)
    com_dz = float(abs(np.mean(pts[:, 2]) - sh.com[2]))
    return dict(g=g, F_req=F, cos_t=cos_t, tilt=tilt, kappa=kap, pts=pts, cen=cen, edge=edge, com_dz=com_dz), ''


def _edge_gap(sh, cen, nrm, R, d=0.007):
    """How far the surface falls away from the fingertip 7 mm to each side of the contact (m, worst finger).  ~0 on a flat wall,
    a few tenths of a mm on a curved one, several mm when the contact sits on a rim / corner / thin wall top - a grasp that
    would slip off or pivot there, so the planner penalises it."""
    n = len(cen)
    up = np.array([0.0, 0.0, 1.0])
    t1 = np.cross(nrm, up)
    nn = np.linalg.norm(t1, axis=1, keepdims=True)
    t1 = np.where(nn > 1e-6, t1 / np.maximum(nn, 1e-12), np.array([1.0, 0.0, 0.0]))
    t2 = np.cross(nrm, t1)
    P = np.concatenate([cen + d * t1, cen - d * t1, cen + d * t2, cen - d * t2])
    g = (sh.sdf(P) - R).reshape(4, n)
    return float(max(g.max(), 0.0))


def _full(lay, idx, vals, idle):
    out = [idle(i) for i in range(len(lay))]
    for j, i in enumerate(idx): out[i] = vals[j]
    return out


def _path_clearance(sh, lay, idx, cand, z_palm, z_hover, s_clear, s_c):
    """Swept clearance (descent with preshape + closing) of the FULL hand; idle fingers stay open."""
    bases = [l['base'] for l in lay]
    phis = _full(lay, idx, cand['phi'], lambda i: lay[i]['phi0'])
    worst = dict(rigid=1.0, dome=1.0, table=1.0, ff=1.0)

    def upd(r, slack):
        worst['rigid'] = min(worst['rigid'], r['rigid'])
        worst['dome'] = min(worst['dome'], float(r['dome'].min()) + slack)
        worst['table'] = min(worst['table'], r['table'])
        worst['ff'] = min(worst['ff'], r['ff'])
        return r['rigid'] < 0.0008 or worst['dome'] < -0.0005 or r['table'] < 0.0005 or r['ff'] < 0.0008
    s0 = [preshape_curl(z_palm, sc) for sc in s_clear]
    poses = [(z_palm, [b for b in s_c], 0.0006)]                          # final grasp pose first: cheapest rejection
    poses += [(z_palm, [a + t * (b - a) for a, b in zip(s0, s_c)], 0.0006) for t in np.linspace(0.0, 0.93, 12)]
    poses += [(float(z), [preshape_curl(float(z), sc) for sc in s_clear], 0.0) for z in np.linspace(z_hover, z_palm, 8)]
    for z, sl, slack in poses:
        s_i = _full(lay, idx, sl, lambda i: 0.0)
        if upd(pose_clearance(sh, hand_geometry(bases, phis, s_i), cand['yaw'], z), slack): break
    return worst


def _clear_s(sh, lay, cand, z_palm, i, s_contact, R, gap):
    lo, hi = 0.0, float(s_contact)
    for _ in range(24):
        mid = 0.5 * (lo + hi)
        if sh.sdf(_dome_world(lay, cand, z_palm, i, mid)[None])[0] > R + gap: lo = mid
        else: hi = mid
    return lo


def _clear_s_all(sh, lay, cand, z_palm, s_contact, R, gap):
    """_clear_s for all fingers at once (24 batched distance queries instead of 24 per finger)."""
    lo, hi = np.zeros(len(lay)), np.array([float(s) for s in s_contact])
    for _ in range(24):
        mid = 0.5 * (lo + hi)
        d = sh.sdf(np.array([_dome_world(lay, cand, z_palm, i, float(m)) for i, m in enumerate(mid)]))
        free = d > R + gap
        lo, hi = np.where(free, mid, lo), np.where(free, hi, mid)
    return [float(x) for x in lo]


def _subsets(n, lay):
    import itertools
    out = []
    for k in range(n, 1, -1):
        sets = [c for c in itertools.combinations(range(n), k) if len({lay[i]['group'] for i in c}) == 2]
        out.append(sets)
    return out


def _z_palm_guess(sh, z_c, R):
    """Palm height that puts the dome centre at z_c when it touches the part side (ray from the hand axis along +x / -x)."""
    xs = np.linspace(0.0, 0.12, 241)
    best = []
    for sgn in (1.0, -1.0):
        d = sh.sdf(np.stack([sgn * xs, np.zeros_like(xs), np.full_like(xs, z_c)], -1))
        idx = np.where(d <= R)[0]
        best.append(float(xs[idx.max()]) if len(idx) else 0.03)
    r_c = 0.5 * (best[0] + best[1])
    ss = np.linspace(0.0, s_limit(), 121)
    rr = np.array([dome_center(float(x))[0] for x in ss])
    s_g = float(np.interp(r_c, rr[::-1], ss[::-1]))
    return z_c - dome_center(s_g)[1]


def _search1(sh, o, lay_all, idx, z_palm0, h, R, hh, z_hover, z_c_target, reasons, z_start):
    lay = [lay_all[i] for i in idx]
    cands = _cands(lay, [0] if sh.is_round() else None)   # round parts: yaw is irrelevant
    z_palm = max(z_start, hh + h['palm_clear'])
    nom = [c for c in cands if c['Q'] == ((0.0, 0.0), (0.0, 0.0)) and c['yaw'] == 0.0][:1] or cands[:1]
    for _ in range(5):
        s0, ok0 = _find_contacts(sh, lay, nom, z_palm, R)
        if not ok0.all(): break
        zmean = float(np.mean([dome_center(float(sc))[1] for sc in s0[0]]))
        z_new = max(z_c_target - zmean, hh + h['palm_clear'])
        if abs(z_new - z_palm) < 2e-4: break
        z_palm = z_new
    s_c, ok = _find_contacts(sh, lay, cands, z_palm, R)
    reasons['no contact within reach'] = reasons.get('no contact within reach', 0) + int((~ok.all(1)).sum())
    scored = []
    okc = np.where(ok.all(1))[0]
    if len(okc):                                                  # vectorised necessary friction-cone test
        S_, rc_, zc_ = _dome_tab()
        sc_ = s_c[okc]
        phi_ = np.array([cands[c]['phi'] for c in okc]); yaw_ = np.array([cands[c]['yaw'] for c in okc])
        base_ = np.array([l['base'] for l in lay])
        rr = np.interp(sc_, S_, rc_) - HAND['r_base']
        px = base_[None, :, 0] + rr * np.cos(phi_); py = base_[None, :, 1] + rr * np.sin(phi_)
        cy_, sy_ = np.cos(yaw_)[:, None], np.sin(yaw_)[:, None]
        cen_ = np.stack([cy_ * px - sy_ * py, sy_ * px + cy_ * py, z_palm + np.interp(sc_, S_, zc_)], -1)
        nrm_ = sh.normal(cen_.reshape(-1, 3)).reshape(cen_.shape)
        u_ = np.stack([cy_ * np.cos(phi_) - sy_ * np.sin(phi_), sy_ * np.cos(phi_) + cy_ * np.sin(phi_), np.zeros_like(phi_)], -1)
        cos_ = (nrm_ * u_).sum(-1)
        good = (cos_ >= math.cos(math.atan(o['mu'])) - 1e-3).all(1)
        reasons['contact normal outside friction cone'] = reasons.get('contact normal outside friction cone', 0) + int((~good).sum())
        okc = okc[good]
    for ci in okc:
        ev, why = _evaluate(sh, o, lay, cands[ci], z_palm, s_c[ci], R)
        if ev is None:
            reasons[why] = reasons.get(why, 0) + 1
            continue
        robust = (1.0 + 0.4 * max(ev['edge'] * 1e3 - 0.8, 0.0)) * (1.0 + ev['com_dz'] / 0.02)     # edge / rim contacts and contacts far from the CoM are less robust
        scored.append((ev['F_req'] * (1.0 + float(ev['tilt'].max()) / 90.0) * robust + 1e-3 * abs(cands[ci]['yaw']), ci, ev))
    scored.sort(key=lambda t: t[0])
    alts = []
    tried = 0
    for cost, ci, ev in scored[:200]:
        if tried >= 400 or len(alts) >= 2: break
        tried += 1
        cand, sc = cands[ci], [float(x) for x in s_c[ci]]
        s_clear = _clear_s_all(sh, lay, cand, z_palm, sc, R, h['approach_gap'])
        z_floor = max(h['table_clear'] - lowest_z(s) for s in s_clear + sc)
        zp = max(z_palm, z_floor)
        pc = _path_clearance(sh, lay_all, idx, cand, zp, max(z_hover, zp + 0.02), s_clear, sc)
        if pc['rigid'] < 0.0008 or pc['dome'] < -0.0005 or pc['table'] < 0.0005 or pc['ff'] < 0.0008:
            why = ('finger would hit the part' if (pc['rigid'] < 0.0008 or pc['dome'] < -0.0005) else
                   'finger would hit the table' if pc['table'] < 0.0005 else 'fingers would collide with each other')
            reasons[why] = reasons.get(why, 0) + 1
            continue
        alts.append(dict(cand=cand, ev=ev, s_c=sc, s_clear=s_clear, z_palm=zp, clear=pc, cost=cost, idx=list(idx)))
    return alts


def _search(sh, o, lay_all, idx, z_palm0, h, R, hh, z_hover, z_c_target, reasons):
    alts = []
    zg = _z_palm_guess(sh, z_c_target, R)
    for z_start in (zg, z_palm0, zg + 0.012, zg - 0.008, zg + 0.025):  # geometric guess first, nominal-contact start second, then a height ladder
        alts = _search1(sh, o, lay_all, idx, z_palm0, h, R, hh, z_hover, z_c_target, reasons, z_start)
        if alts: break
    return alts


def plan_key(o, h=HAND):
    return (o.get('label'), o.get('mesh'), tuple(o['dims']), o['m'], o['mu'], int(h['n_fingers']), tuple(o.get('com') or ()), h['r_base'])


def plan_grasp(o, h=HAND, phi0=None):
    key = plan_key(o, h)
    if key in _PLAN_CACHE: return _PLAN_CACHE[key]
    _RELAX[0] = False
    plan = _plan_grasp(o, h, phi0)
    if not plan['ok'] and get_shape(o).h < 0.06:                              # retry with the tighter (dome-accurate) table clearance
        first = _PLAN_CACHE.pop(key, plan)
        _RELAX[0] = True
        try: plan2 = _plan_grasp(o, h, phi0)
        finally: _RELAX[0] = False
        if plan2['ok']: return plan2
        _PLAN_CACHE[key] = first
        return first
    return plan


def _plan_grasp(o, h=HAND, phi0=None):
    n = int(h['n_fingers'])
    key = plan_key(o, h)
    if key in _PLAN_CACHE: return dict(_PLAN_CACHE[key])
    sh = get_shape(o)
    lay = layout(n)
    hh = sh.h
    R = h['tip_r']
    msgs = []
    ro = r_open(h)
    z_pref = o['grasp_z'] if o.get('grasp_z') is not None else (0.5 * hh if sh.kind == 'sphere' else 0.55 * hh)
    z_floor_c = R + h['table_clear'] + 0.0015                   # lowest dome centre that keeps the dome off the table
    z_c_target = max(min(z_pref, hh - 0.010), z_floor_c)
    h_min = z_floor_c + 0.004
    z_palm0 = max(z_c_target - dome_center(0.35)[1], hh + h['palm_clear'])
    z_hover = hh + drop_open(h) + h['hover_margin']
    reasons = {}
    alts = []
    done_size = None
    for sets in _subsets(n, lay):                       # all fingers first; park fingers only if that is better / needed
        k = len(sets[0]) if sets else 0
        if done_size is not None and k > 2 and min(alts, key=lambda a: a['cost'])['ev']['edge'] <= 0.002: continue   # first feasible size is robust -> only 2-finger pinches are compared; if its best grasp sits on a rim / edge, every smaller finger set is tried too
        for idx in sets:
            alts += _search(sh, o, lay, list(idx), z_palm0, h, R, hh, z_hover, z_c_target, reasons)
        if alts and done_size is None: done_size = k
    if not alts:
        if hh < h_min: msgs.append('part too low: %.0f mm tall, the dome fingertips need at least %.0f mm to reach its side above the table' % (1000 * hh, 1000 * h_min))
        msgs.append('no collision-free, friction-feasible grasp found (' + '; '.join('%s x%d' % (k, v) for k, v in sorted(reasons.items(), key=lambda kv: -kv[1]) if v) + ')')
        if n >= 5 and hh >= h_min: msgs.append('hint: parts smaller than ~35 mm wide leave no room for five 22 mm fingertips - try FINGERS=3 or 4')
        if sh.kind != 'mesh' and 2 * float(o['dims'][0]) > 2 * ro: msgs.append('object too wide: %.1f mm > hand opening %.1f mm' % (2000 * o['dims'][0], 2000 * ro))
        plan = dict(n=n, phis=[l['phi0'] for l in lay], sigmas=[0.0] * n, bases=[l['base'] for l in lay], yaw=0.0,
                    rhos=[0.0] * n, s_contact=[0.0] * n, s_mean=0.0, z_contact=0.0, z_palm=z_palm0, psi=0.0, ok=False, msgs=msgs,
                    r_open=ro, s_clear=[0.0] * n, z_floor=0.0, grasp_raised=False, z_hover=z_hover, g=[1.0] * n,
                    cos_t=[1.0] * n, tilt_deg=[0.0] * n, fric_gain=float(n), F_req=float('nan'), xc=[0.002] * n, alts=[],
                    active=[True] * n)
        _PLAN_CACHE[key] = plan
        return dict(plan)
    from .materials import GRIPPER as _GR
    f_lim = 0.8 * min(float(o.get('F_allow', 1e9)), float(_GR['F_max']))
    alts.sort(key=lambda a: (a['ev']['F_req'] > f_lim, a['cost'] * (1.0 + 0.12 * (n - len(a['idx'])))))
    plan = plan_from_alt(alts[0], lay, o, hh, z_hover, n, ro, msgs)
    if alts[0]['ev']['F_req'] > f_lim:
        plan['ok'] = False
        plan['msgs'] = ['NO SAFE GRIP: needs %.1f N per reference finger, limit %.1f N (friction too low for this part)' % (alts[0]['ev']['F_req'], f_lim)]
    plan['alts'] = alts
    _PLAN_CACHE[key] = plan
    return dict(plan)


def plan_from_alt(a, lay, o, hh, z_hover, n, ro, msgs):
    cand, ev, idx = a['cand'], a['ev'], a['idx']
    xtab, stab, _ = curl_tables()
    act = [i in idx for i in range(n)]
    F = lambda vals, idle: _full(lay, idx, vals, lambda i: idle)
    xc = F([max(0.002, float(np.interp(s, stab, xtab))) for s in a['s_c']], 0.0)
    W = o['m'] * G
    fric_gain = W / (o['mu'] * ev['F_req']) if ev['F_req'] > 0 else float(len(idx))
    zc_mean = a['z_palm'] + sum(dome_center(s)[1] for s in a['s_c']) / len(idx)
    z_floor = max(HAND['table_clear'] - lowest_z(s) for s in list(a['s_clear']) + list(a['s_c']))
    return dict(n=n, active=act, phis=_full(lay, idx, list(cand['phi']), lambda i: lay[i]['phi0']),
                sigmas=F(list(cand['sig']), 0.0), bases=[l['base'] for l in lay], yaw=cand['yaw'], aim=cand['Q'],
                rhos=F([dome_center(s)[0] for s in a['s_c']], 0.0), s_contact=F(list(a['s_c']), 0.0),
                s_mean=sum(a['s_c']) / len(idx), z_contact=zc_mean, z_palm=a['z_palm'],
                psi=sum(dome_center(s)[2] for s in a['s_c']) / len(idx), ok=True, msgs=list(msgs), r_open=ro,
                s_clear=F(list(a['s_clear']), 0.0), z_floor=z_floor, grasp_raised=a['z_palm'] < z_floor + 1e-9,
                z_hover=max(z_hover, a['z_palm'] + 0.02), g=F([float(x) for x in ev['g']], 0.0),
                cos_t=F([float(x) for x in ev['cos_t']], 1.0), tilt_deg=F([float(x) for x in ev['tilt']], 0.0),
                fric_gain=float(fric_gain), F_req=float(ev['F_req']), xc=xc,
                contact_pts=ev['pts'].tolist(), clearance=a['clear'])
