"""physics_check.py - independent rigid-body contact test of a planned grasp in MuJoCo.

The planner/controller work on an analytic model.  This module re-creates the same hand (same joint layout, same
phalanx lengths, dome fingertips, spread angles, wrist yaw) as REAL colliding MuJoCo bodies, puts the part next to
it as colliding geometry (primitive or convex-decomposed prism), applies finger forces through the joint torques
J^T f, descends, closes, lifts and reports:
  * lifted / slipped (does the part really come up with the hand?)
  * number of contact steps between the part and any NON-fingertip hand geometry (palm, phalanx links)
  * worst fingertip penetration depth (contact distance) and peak contact force
Run:  python -m grip_gz.physics_check [object_key|all] [n_fingers]
"""
import math
import sys
import numpy as np
import mujoco

from . import hand as H
from . import materials as M
from .shapes import get_shape
from . import meshprep as MP

DT = 0.001


def _ear_clip(P):
    """Triangulate a simple polygon (list of (x, y)); returns index triples.  O(n^2) with numpy point-in-triangle tests."""
    P = np.asarray(P, dtype=float)
    idx = list(range(len(P)))
    if _area2(P.tolist()) < 0: idx.reverse()
    def cr(a, b, c): return (P[b, 0] - P[a, 0]) * (P[c, 1] - P[a, 1]) - (P[b, 1] - P[a, 1]) * (P[c, 0] - P[a, 0])
    tris, guard = [], 0
    while len(idx) > 3 and guard < 4 * len(P) + 50:
        guard += 1
        ia = np.array(idx)
        for k in range(len(idx)):
            a, b, c = idx[k - 1], idx[k], idx[(k + 1) % len(idx)]
            if cr(a, b, c) <= 1e-14: continue
            oth = ia[(ia != a) & (ia != b) & (ia != c)]
            if len(oth):
                Q = P[oth]
                d1 = (P[b, 0] - P[a, 0]) * (Q[:, 1] - P[a, 1]) - (P[b, 1] - P[a, 1]) * (Q[:, 0] - P[a, 0])
                d2 = (P[c, 0] - P[b, 0]) * (Q[:, 1] - P[b, 1]) - (P[c, 1] - P[b, 1]) * (Q[:, 0] - P[b, 0])
                d3 = (P[a, 0] - P[c, 0]) * (Q[:, 1] - P[c, 1]) - (P[a, 1] - P[c, 1]) * (Q[:, 0] - P[c, 0])
                if np.any((d1 >= -1e-14) & (d2 >= -1e-14) & (d3 >= -1e-14)): continue
            tris.append((a, b, c)); idx.pop(k); break
        else: break
    if len(idx) == 3: tris.append(tuple(idx))
    return tris


def _area2(P):
    return sum(P[i][0] * P[(i + 1) % len(P)][1] - P[(i + 1) % len(P)][0] * P[i][1] for i in range(len(P)))


def _hull(pts):
    pts = sorted(set(pts))
    if len(pts) < 3: return pts
    def half(seq):
        h = []
        for q in seq:
            while len(h) >= 2 and (h[-1][0] - h[-2][0]) * (q[1] - h[-2][1]) - (h[-1][1] - h[-2][1]) * (q[0] - h[-2][0]) <= 0: h.pop()
            h.append(q)
        return h[:-1]
    return half(pts) + half(pts[::-1])


def _convex_pieces(P):
    """Simple polygon (n,2) -> list of CONVEX vertex-index loops (ear clipping + Hertel-Mehlhorn merging).  Falls back to the hull if clipping loses area."""
    P = np.asarray(P, dtype=float)
    tris = _ear_clip(P)
    ar = abs(_area2(P.tolist())) / 2.0
    cover = sum(abs(_area2([tuple(P[i]) for i in t])) / 2.0 for t in tris)
    if not tris or cover < 0.97 * ar: return None
    def cr(a, b, c): return (P[b, 0] - P[a, 0]) * (P[c, 1] - P[a, 1]) - (P[b, 1] - P[a, 1]) * (P[c, 0] - P[a, 0])
    pieces = [list(t) for t in tris]
    ccw = _area2(P.tolist()) > 0
    pieces = [t if cr(*t) > 0 else t[::-1] for t in pieces]
    merged = True
    while merged:
        merged = False
        edge = {}
        for pi, pc in enumerate(pieces):
            for k in range(len(pc)): edge.setdefault((pc[k], pc[(k + 1) % len(pc)]), pi)
        for (u, v), pi in list(edge.items()):
            pj = edge.get((v, u))
            if pj is None or pj == pi or pieces[pi] is None or pieces[pj] is None: continue
            A, B = pieces[pi], pieces[pj]
            ia, ib = A.index(u), B.index(v)
            A2 = A[ia + 1:] + A[:ia + 1]                    # ends with u ... starts after u: v first
            B2 = B[ib + 1:] + B[:ib + 1]                    # ends with v, starts after v: u first
            M = A2[:-1] + B2[:-1]                           # A: v..(before u) ; B: u..(before v)  -> loop without the shared edge
            M = A[ia + 1:] + A[:ia] if False else M
            # rebuild explicitly: A walk from v around to u, then B walk from u around to v
            M = []
            k = A.index(v)
            for t in range(len(A)): M.append(A[(k + t) % len(A)])
            k = B.index(u)
            for t in range(1, len(B) - 1): M.append(B[(k + t) % len(B)])
            n = len(M)
            if all(cr(M[i - 1], M[i], M[(i + 1) % n]) >= -1e-12 for i in range(n)):
                pieces[pi], pieces[pj] = M, None; merged = True
                break
        pieces = [p for p in pieces if p is not None]
    return pieces


def _simplify(P, tol=1.5e-4):
    """Drop vertices closer than tol (m) to the chord of their neighbours; keeps the O(n^3) ear clipper fast on dense CAD slices."""
    P = list(P)
    i = 0
    while len(P) > 4 and i < len(P):
        a, b, c = P[i - 1], P[i], P[(i + 1) % len(P)]
        L = math.hypot(c[0] - a[0], c[1] - a[1])
        dev = abs((c[0] - a[0]) * (a[1] - b[1]) - (a[0] - b[0]) * (c[1] - a[1])) / L if L > 1e-12 else 0.0
        if dev < tol: P.pop(i)
        else: i += 1
    return P


def _bands(o):
    """Part proxy for rigid-body physics: a stack of convex-decomposed prisms, one per height band whose cross-section differs from the next
    (a hull, a funnel, a mast ... are NOT one extruded mid-slice).  Returns [(z0, z1, polygon (n,2)), ...] in object coordinates (z up from the table)."""
    sh = get_shape(o)
    K = len(sh.polys)
    info = []
    for k in range(K):
        pl = sh.polys[k]
        P = pl[0] if len(pl) else None
        if P is None: info.append(None); continue
        info.append((abs(_area2(P.tolist())) / 2.0, P.min(0), P.max(0)))
    zobj = sh.zl - sh.zmid + sh.h / 2.0
    edges = np.concatenate([[0.0], 0.5 * (zobj[1:] + zobj[:-1]), [sh.h]])
    tol = 0.04
    while True:
        groups, cur = [], [0]
        for k in range(1, K):
            r = info[cur[0]]; q = info[k]
            same = r is not None and q is not None and abs(q[0] - r[0]) <= tol * max(r[0], 1e-9) and np.abs(q[1] - r[1]).max() < 8e-4 + tol * 0.02 and np.abs(q[2] - r[2]).max() < 8e-4 + tol * 0.02
            if same: cur.append(k)
            else: groups.append(cur); cur = [k]
        groups.append(cur)
        if len(groups) <= 8 or tol > 2.0: break
        tol *= 1.8
    out = []
    for g in groups:
        kk = g[len(g) // 2]
        pl = sh.polys[kk]
        if not len(pl): continue
        out.append((float(edges[g[0]]), float(edges[g[-1] + 1]), np.asarray(pl[0], dtype=float)))
    return out


def _prism_geoms(P, z0, z1, tag, fr):
    """Convex prisms covering polygon P between z0 and z1 -> (asset xml, geom xml list)."""
    P = MP.douglas_peucker(np.asarray(P, dtype=float), 2.5e-4)
    if _area2(P.tolist()) < 0: P = P[::-1]
    pcs = _convex_pieces(P) if len(P) >= 3 else None
    loops = [[tuple(P[i]) for i in pc] for pc in pcs] if pcs else [_hull([tuple(q) for q in P])]
    asset, geoms = [], []
    for j, L in enumerate(loops):
        if len(L) < 3 or abs(_area2(L)) < 2e-8: continue
        v = []
        for z in (z0, z1):
            for q in L: v += [q[0], q[1], z]
        nm = '%s_%d' % (tag, j)
        asset.append('<mesh name="%s" vertex="%s"/>' % (nm, ' '.join('%.6f' % x for x in v)))
        geoms.append('<geom name="part_%s" type="mesh" mesh="%s" friction="%s"/>' % (nm, nm, fr))
    return ''.join(asset), geoms


def _object_xml(o, mu):
    """returns (asset_xml, [geom xml])"""
    sh = get_shape(o)
    h = sh.h
    fr = '%g 0.003 0.0001' % mu
    if sh.kind == 'box':
        d = o['dims']
        return '', ['<geom name="part" type="box" pos="0 0 %g" size="%g %g %g" friction="%s"/>' % (h / 2, d[0] / 2, d[1] / 2, d[2] / 2, fr)]
    if sh.kind == 'sphere':
        return '', ['<geom name="part" type="sphere" pos="0 0 %g" size="%g" friction="%s"/>' % (o['dims'][0], o['dims'][0], fr)]
    if sh.kind == 'cyl':
        return '', ['<geom name="part" type="cylinder" pos="0 0 %g" size="%g %g" friction="%s"/>' % (h / 2, o['dims'][0], h / 2, fr)]
    asset, geoms = [], []
    for bi, (z0, z1, P) in enumerate(_bands(o)):
        if z1 - z0 < 1e-4: continue
        a_, g_ = _prism_geoms(P, z0, z1, 'b%d' % bi, fr)
        asset.append(a_); geoms += g_
    return ''.join(asset), geoms


def build_model(o, plan, n):
    mu, h = o['mu'], H.HAND
    lay = H.layout(n)
    asset, geoms = _object_xml(o, mu)
    L, caps = h['L'], h['cap_r']
    X = ['<mujoco><compiler angle="radian"/><option timestep="%g" gravity="0 0 -9.81" cone="elliptic" impratio="10"/>' % DT,
         '<asset>%s</asset><worldbody>' % asset,
         '<geom name="table" type="plane" size="1 1 0.1" contype="4" conaffinity="6" friction="0.8 0.005 0.0001"/>']
    com = list(o.get('com') or (0.0, 0.0, get_shape(o).h / 2.0))
    X.append('<body name="part" pos="0 0 0.0005"><freejoint name="pj"/>')
    X.append('<inertial pos="%g %g %g" mass="%g" diaginertia="%g %g %g"/>' % (com[0], com[1], com[2], o['m'], *(o['m'] * 4e-4 * np.ones(3))))
    X += [g.replace('<geom ', '<geom contype="5" conaffinity="6" condim="4" ') for g in geoms]
    X.append('</body>')
    X.append('<body name="hand" pos="0 0 0" quat="%g 0 0 %g" gravcomp="1"><joint name="hz" type="slide" axis="0 0 1" damping="400"/><inertial pos="0 0 0" mass="3" diaginertia="0.01 0.01 0.01"/>' % (math.cos(plan['yaw'] / 2), math.sin(plan['yaw'] / 2)))
    X.append('<geom name="palm" type="box" pos="0 0 %g" size="%g %g %g" contype="2" conaffinity="0"/>' % (h['palm_thick'] / 2, h['palm_x'], h['palm_y'], h['palm_thick'] / 2))
    for i, f in enumerate(lay):
        bx, by = f['base']
        phi = plan['phis'][i]
        X.append('<body name="f%d" pos="%g %g 0" quat="%g 0 0 %g" gravcomp="1">' % (i, bx, by, math.cos(phi / 2), math.sin(phi / 2)))
        X.append('<body name="f%d_p1" quat="%g 0 %g 0" gravcomp="1">' % (i, math.cos(math.pi / 4), math.sin(math.pi / 4)))
        X.append('<joint name="f%d_q1" type="hinge" axis="0 1 0" damping="0.02" armature="0.004"/>' % i)
        X.append('<geom name="f%d_c1" type="capsule" fromto="0 0 0 %g 0 0" size="%g" mass="0.02" contype="2" conaffinity="0"/>' % (i, L[0], caps[0]))
        X.append('<body name="f%d_p2" pos="%g 0 0" gravcomp="1"><joint name="f%d_q2" type="hinge" axis="0 1 0" damping="0.02" armature="0.004"/>' % (i, L[0], i))
        X.append('<geom name="f%d_c2" type="capsule" fromto="0 0 0 %g 0 0" size="%g" mass="0.015" contype="2" conaffinity="0"/>' % (i, L[1], caps[1]))
        X.append('<body name="f%d_p3" pos="%g 0 0" gravcomp="1"><joint name="f%d_q3" type="hinge" axis="0 1 0" damping="0.02" armature="0.004"/>' % (i, L[1], i))
        X.append('<geom name="f%d_c3" type="capsule" fromto="0 0 0 %g 0 0" size="%g" mass="0.01" contype="2" conaffinity="0"/>' % (i, L[2] - h['pad_back'] - 0.012, caps[2]))
        X.append('<geom name="dome%d" type="sphere" pos="%g 0 0" size="%g" mass="0.01" contype="2" conaffinity="0" condim="4" friction="%g 0.003 0.0001" solref="0.004 1"/>' % (i, L[2] - h['pad_back'], h['tip_r'], mu))
        X.append('</body></body></body></body>')
    X.append('</body></worldbody>')
    X.append('<actuator><position name="az" joint="hz" kp="30000" kv="0"/></actuator>')
    X.append('<equality>')
    for i in range(n):
        for k in (2, 3):
            X.append('<joint joint1="f%d_q%d" joint2="f%d_q1" polycoef="0 %g 0 0 0" solref="0.002 1"/>' % (i, k, i, h['syn'][k - 1] / h['syn'][0]))
    X.append('</equality></mujoco>')
    return '\n'.join(X)


def min_jerk(x):
    x = min(max(x, 0.0), 1.0)
    return x ** 3 * (10 - 15 * x + 6 * x * x)


def run(obj_key, n, F_scale=2.0, verbose=False, plan=None):
    o = M.OBJECTS[obj_key]
    H.HAND['n_fingers'] = n
    plan = plan or H.plan_grasp(o)
    if not plan['ok']: return dict(obj=obj_key, n=n, result='NO PLAN', note=plan['msgs'][0][:90])
    m = mujoco.MjModel.from_xml_string(build_model(o, plan, n))
    d = mujoco.MjData(m)
    act = plan['active']
    qa = [[m.joint('f%d_q%d' % (i, k)).qposadr[0] for k in (1, 2, 3)] for i in range(n)]
    va = [[m.joint('f%d_q%d' % (i, k)).dofadr[0] for k in (1, 2, 3)] for i in range(n)]
    dome_id = {m.geom('dome%d' % i).id for i in range(n)}
    part_geoms = {g for g in range(m.ngeom) if m.geom_bodyid[g] == m.body('part').id}
    pq0 = m.joint('pj').qposadr[0]
    yaw, zp, zh = plan['yaw'], plan['z_palm'], plan['z_hover']
    hz = m.joint('hz').qposadr[0]
    F_hold = min(F_scale * plan['F_req'], 0.8 * min(o['F_allow'], M.GRIPPER['F_max']))
    s_cl = [H.preshape_curl(zh, plan['s_clear'][i]) if act[i] else 0.0 for i in range(n)]
    sc = plan['s_contact']
    for i in range(n):
        for k, q in enumerate(H.joint_angles(s_cl[i])): d.qpos[qa[i][k]] = q
    d.qpos[hz] = zh
    d.ctrl[0] = zh
    mujoco.mj_forward(m, d)
    T_desc, T_close, T_ramp, T_lift, T_hold, LIFT = 1.0, 0.8, 1.2, 1.5, 1.0, 0.10
    stats = dict(rigid_steps=0, rigid_depth=0.0, dome_depth=0.0, table_hand=0)
    kp, kd = np.array([60.0, 40.0, 24.0]), np.array([0.8, 0.5, 0.3])
    nstep = int((T_desc + T_close + T_ramp + T_lift + T_hold) / DT)
    z_part0 = rel0 = None
    z = zh
    for st in range(nstep):
        t = st * DT
        d.qfrc_applied[:] = 0
        mode = 'servo'
        if t < T_desc:
            z = zh + (zp - zh) * min_jerk(t / T_desc)
            tgt = [H.preshape_curl(z, plan['s_clear'][i]) if act[i] else 0.0 for i in range(n)]
        elif t < T_desc + T_close:
            z = zp
            a = min_jerk((t - T_desc) / T_close)
            tgt = [(s_cl[i] + (max(sc[i] - 0.015, 0.0) - s_cl[i]) * a) if act[i] else 0.0 for i in range(n)]
        else:
            tt = t - T_desc - T_close
            z = zp + (LIFT * min_jerk((tt - T_ramp) / T_lift) if tt > T_ramp else 0.0)
            mode = 'force'
        d.ctrl[0] = z
        for i in range(n):
            q1, qd1 = float(d.qpos[qa[i][0]]), float(d.qvel[va[i][0]])
            if mode == 'servo' or not act[i]:
                st_ = tgt[i] if mode == 'servo' else 0.0
                tau = 120.0 * (st_ * H.HAND['syn'][0] * H.HAND['q_max'] - q1) - 2.0 * qd1
            else:
                s_now = float(np.clip(q1 / (H.HAND['syn'][0] * H.HAND['q_max']), 0, 0.999))
                ds = 1e-3
                lever = abs(H.contact_point(s_now + ds)[0] - H.contact_point(s_now)[0]) / (ds * H.HAND['syn'][0] * H.HAND['q_max'])
                tau = lever * F_hold * min(1.0, tt / (T_ramp * 0.8)) * plan['g'][i]
            cc = np.array(H.HAND['syn']) / H.HAND['syn'][0]
            for k in range(3): d.qfrc_applied[va[i][k]] = cc[k] * tau / float((cc * cc).sum()) if (mode == 'force' and act[i]) else (tau if k == 0 else 0.0)
        mujoco.mj_step(m, d)
        if st % 5 == 0:
            for c in d.contact[:d.ncon]:
                g1, g2 = int(c.geom1), int(c.geom2)
                pg = g1 if g1 in part_geoms else (g2 if g2 in part_geoms else None)
                if pg is None:
                    if 'table' in (m.geom(g1).name, m.geom(g2).name): stats['table_hand'] += 1
                    continue
                og = g2 if pg == g1 else g1
                if m.geom(og).name == 'table': continue
                if og in dome_id: stats['dome_depth'] = max(stats['dome_depth'], -min(c.dist, 0.0))
                else: (verbose and print('RIGID t=%.3f %s dist=%.2fmm pos=%s part=%s' % (t, m.geom(og).name, 1000*c.dist, np.round(c.pos,4), np.round(d.qpos[pq0:pq0+3],4)))); stats.setdefault('names', set()).add(m.geom(og).name); stats['rigid_steps'] += 1; stats['rigid_depth'] = max(stats['rigid_depth'], -min(c.dist, 0.0))
        if verbose and st % 250 == 0:
            cs = []
            for ci in range(d.ncon):
                c = d.contact[ci]
                f6 = np.zeros(6); mujoco.mj_contactForce(m, d, ci, f6)
                cs.append('%s-%s %.1fmm N=%.1f ft=%.1f,%.1f dz=%.3f' % (m.geom(int(c.geom1)).name, m.geom(int(c.geom2)).name, 1000 * c.dist, f6[0], f6[1], f6[2], d.geom_xpos[m.geom('dome0').id][2]))
            print('t=%.2f z=%.3f part=%s ncon=%d %s' % (t, z, np.round(d.qpos[pq0:pq0 + 3], 4), d.ncon, cs[:8]))
        if z_part0 is None and t >= T_desc + T_close + T_ramp:
            z_part0 = float(d.qpos[pq0 + 2])
            rel0 = np.array(d.qpos[pq0:pq0 + 3]) - np.array([0, 0, z])
    pq = np.array(d.qpos[pq0:pq0 + 3])
    rise = float(pq[2]) - z_part0
    slip = float(np.linalg.norm((pq - np.array([0, 0, z])) - rel0))
    lifted = rise > 0.9 * LIFT and slip < 0.004
    res = 'LIFTED' if lifted and stats['rigid_steps'] == 0 else ('RIGID CONTACT' if lifted else 'SLIPPED')
    return dict(obj=obj_key, n=n, result=res, rise_mm=1000 * rise, slip_mm=1000 * slip, F_hold=F_hold, active=sum(act),
                rigid_steps=stats['rigid_steps'], rigid_depth_mm=1000 * stats['rigid_depth'], dome_depth_mm=1000 * stats['dome_depth'],
                table_hand=stats['table_hand'], names=sorted(stats.get('names', [])))


def verified_plan(obj_key, n, max_alts=4, F_scale=2.0):
    """Planner result that has also passed the MuJoCo contact test (lift 100 mm, <4 mm slip, no non-fingertip contact).
    Tries the best planner alternatives in cost order; the first one that really lifts is stored as THE plan for this part."""
    o = M.OBJECTS[obj_key]
    H.HAND['n_fingers'] = n
    base = H.plan_grasp(o)
    if not base['ok'] or base.get('physics'): return base
    lay, hh = H.layout(n), get_shape(o).h
    z_hover = hh + H.drop_open() + H.HAND['hover_margin']
    cands = [base] + [H.plan_from_alt(a, lay, o, hh, z_hover, n, base['r_open'], []) for a in base['alts'][1:max_alts]]
    f_lim = 0.8 * min(o['F_allow'], M.GRIPPER['F_max'])
    last = None
    for pl in cands:
        pl['alts'] = base['alts']
        if pl['F_req'] * 1.3 > f_lim: continue                       # even a 30 % margin would exceed the allowed grip force
        prev = -1.0
        for fs in (F_scale, 1.5 * F_scale, 2.0 * F_scale):
            hold = min(fs * pl['F_req'], f_lim)
            if hold <= prev + 1e-9: break
            prev = hold
            r = run(obj_key, n, fs, plan=pl)
            last = r
            if r['result'] == 'LIFTED':
                pl['physics'] = r
                pl['hold_factor'] = fs
                H._PLAN_CACHE[H.plan_key(o)] = pl
                return dict(pl)
    base = dict(base)
    base['ok'] = False
    base['msgs'] = ['physics check: no grasp of this part held in rigid-contact simulation (%s)' % (last['result'] if last else 'needs more force than the limit allows')]
    try:                                                              # would a hand with fewer fingers hold it? (small parts leave no room for five fingertips)
        for m_ in range(n - 1, 2, -1):
            H.HAND['n_fingers'] = m_
            alt = H.plan_grasp(o)
            if alt['ok'] and run(obj_key, m_, F_scale, plan=alt)['result'] == 'LIFTED':
                base['msgs'].append('hint: a %d-finger hand does hold this part - restart with FINGERS=%d (or the fingers selector).' % (m_, m_)); break
    finally:
        H.HAND['n_fingers'] = n
    H._PLAN_CACHE[H.plan_key(o)] = base
    return dict(base)


def best_pose(spec, n, log=None):
    """make_object + planning + MuJoCo proof, for a CAD spec.  With up='auto' and no holdable grasp in the first pose (thin tilted back-rest,
    legs the fingertips cannot span ...) the part is laid down in the other axis-aligned resting poses and the first one that is really
    lifted in rigid-contact physics wins (a chair is picked up lying on its side).  Returns (key, o, rep) with the result registered."""
    from . import cad, shapes
    def fresh(sp):
        shapes._CACHE.clear(); H._PLAN_CACHE.clear()
        key, o, rep = cad.make_object(sp)
        cad.register(key, o)
        return key, o, rep
    H.HAND['n_fingers'] = n
    key, o, rep = fresh(spec)
    pl = verified_plan(key, n)
    if pl['ok'] or str(spec.get('up') or 'auto').lower() != 'auto': return key, o, rep
    tried = [rep['up']]
    for up in ('z', 'y', 'x', '-x', '-y', '-z'):
        if up in tried: continue
        tried.append(up)
        try:
            k2, o2, r2 = fresh(dict(spec, up=up, _pose_search=True, rotz=None) if spec.get('rotz') in (None, '', 'auto') else dict(spec, up=up, _pose_search=True))
            if log: log('first pose not holdable; trying up=%s' % up)
            p2 = verified_plan(k2, n)
        except Exception:
            continue
        if p2['ok']:
            r2['warnings'].insert(0, 'POSE: the part cannot be held in its original pose (%s up); it is placed in another resting pose (%s up) where MuJoCo contact physics lifts it.' % (rep['up'], up))
            return k2, o2, r2
    key, o, rep = fresh(spec)                       # nothing worked: restore the first pose so the report explains why
    verified_plan(key, n)
    rep['warnings'].insert(0, 'Tried all 6 resting poses: no pose can be held by this hand.')
    return key, o, rep


def plan_summary(obj_key, n):
    """One-paragraph human-readable result of planning + physics verification (shown in the CAD report)."""
    pl = verified_plan(obj_key, n)
    if pl['ok']:
        ph = pl.get('physics') or {}
        return ('Grasp plan (%d-finger hand): %d fingers used, wrist yaw %.0f deg, min holding force %.1f N per reference finger.\n'
                'MuJoCo contact test: %s (rise %.0f mm, slip %.1f mm, palm/link contacts: %d).' % (
                    n, sum(pl['active']), math.degrees(pl['yaw']), pl['F_req'], ph.get('result', 'not run'), ph.get('rise_mm', 0.0), ph.get('slip_mm', 0.0), ph.get('rigid_steps', 0)))
    return 'Grasp plan (%d-finger hand): NO SAFE GRIP\n  ' % n + '\n  '.join(m for m in pl['msgs'] if m)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    key = argv[0] if argv else 'all'
    ns = [int(argv[1])] if len(argv) > 1 else [3, 5]
    keys = list(M.OBJECTS) if key == 'all' else [key]
    print('%-14s %2s %-13s %7s %7s %7s %5s %10s %10s' % ('object', 'n', 'result', 'rise mm', 'slip mm', 'F_hold', 'rigid', 'rigid pen', 'dome pen'))
    for n in ns:
        for k in keys:
            r = run(k, n)
            if r['result'] == 'NO PLAN':
                print('%-14s %2d %-13s %s' % (k, n, r['result'], r['note'])); continue
            print('%-14s %2d %-13s %7.1f %7.2f %7.2f %5d %8.2f mm %8.2f mm' % (k, n, r['result'], r['rise_mm'], r['slip_mm'], r['F_hold'], r['rigid_steps'], r['rigid_depth_mm'], r['dome_depth_mm']), flush=True)


if __name__ == '__main__':
    main()
