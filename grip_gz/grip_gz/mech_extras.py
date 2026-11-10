"""mech_extras.py - Closed-form solid-mechanics helpers used by the analysis windows (tensors, strain energy, failure
theories, pressure vessels, beams, Castigliano, buckling, torsion, Hertz verification, plane strain).

Every function is a closed-form textbook result evaluated at the CURRENT grasp (force N, friction fy, object o, Hertz state ms),
so each number in the dashboard can be derived on paper.  Assumed hardware data (arm tubes, phalanx section, wrist shaft, shell
thickness) are listed in ASSUME and are meant to be edited.
"""
import math
import numpy as np

G = 9.81

ASSUME = dict(
    # arm links: thin-walled aluminium tube (6061-T6)
    arm_OD=0.040, arm_ID=0.034, arm_E=69e9, arm_rho=2700.0, arm_sy=240e6, hand_mass=0.60, arm_fos=2.0,
    # finger phalanx: solid aluminium block, bending about the joint axis
    ph_E=69e9, ph_b=0.020, ph_h=0.022,
    # wrist shaft: hollow steel tube
    sh_OD=0.020, sh_ID=0.014, sh_L=0.080, sh_G=79e9, sh_sy=250e6, com_offset=0.015,
    # thin-walled shells (wall thickness t, internal gauge pressure p) - egg shell, PET bottle, glass tumbler
    shell={'egg': dict(t=0.35e-3, p=0.0, sy=25e6), 'bottle_empty': dict(t=0.30e-3, p=0.0, sy=50e6),
           'bottle_full': dict(t=0.30e-3, p=2.0e3, sy=50e6), 'glass': dict(t=2.0e-3, p=0.0, sy=40e6),
           'wet_glass': dict(t=2.0e-3, p=0.0, sy=40e6)},
    # brittle strengths (tension, compression) for Mohr-Coulomb / Rankine
    brittle={'glass': (40e6, 1000e6), 'wet_glass': (40e6, 1000e6), 'egg': (25e6, 80e6)},
    pad_width=0.020,                      # finger pad width: length of the line contact for the plane-strain comparison
)


# ---------------------------------------------------------------------------------------------------------------------------
def _yield_strength(o, key, ms):
    """(sy, brittle?, sig_t, sig_c) used by the failure criteria."""
    br = ASSUME['brittle'].get(key)
    if br is not None and ms.get('sig_y_allow') is None:
        return br[0], True, br[0], br[1]
    sy = ms.get('sig_y_allow')
    if sy is None:
        sh = ASSUME['shell'].get(key)
        sy = sh['sy'] if sh else (0.62 * o['p_allow'] if not math.isinf(o['p_allow']) else 40e6)
    return sy, False, sy, sy


# 1 ---- full Cauchy tensor, principal stresses, Mohr circles ------------------------------------------------------------------
def stress_tensor(ms):
    """Cauchy tensor at the critical point z = 0.48a in (r, theta, z): axisymmetric normal stresses plus the friction shear
    tau_rz = mu p0 at the patch edge.  Returns tensor, principal stresses (descending) and the three Mohr-circle shear stresses."""
    sr, st, sz = ms['sig1'], ms['sig2'], ms['sig3']
    tr = ms['tau_edge']
    S = np.array([[sr, 0.0, tr], [0.0, st, 0.0], [tr, 0.0, sz]])
    w = np.sort(np.linalg.eigvalsh(S))[::-1]
    return dict(S=S, p=w, tau12=abs(w[0] - w[1]) / 2, tau23=abs(w[1] - w[2]) / 2, tau13=abs(w[0] - w[2]) / 2,
                tau_max=abs(w[0] - w[2]) / 2, tresca=abs(w[0] - w[2]),
                theta_p=0.5 * math.degrees(math.atan2(2 * tr, sr - sz)))


# 2 ---- strain tensor, strain energy density, distortion-energy check -------------------------------------------------------
def strain_energy(p, E, nu):
    s1, s2, s3 = p
    e = [(a - nu * (b + c)) / E for a, b, c in ((s1, s2, s3), (s2, s1, s3), (s3, s1, s2))]
    U = 0.5 * (s1 * e[0] + s2 * e[1] + s3 * e[2])
    sm = (s1 + s2 + s3) / 3.0
    U_dil = 3.0 * (1 - 2 * nu) / (2 * E) * sm ** 2                     # = (1-2nu)/(6E) (s1+s2+s3)^2
    U_dist = (1 + nu) / (6 * E) * ((s1 - s2) ** 2 + (s2 - s3) ** 2 + (s3 - s1) ** 2)
    svm = math.sqrt(0.5 * ((s1 - s2) ** 2 + (s2 - s3) ** 2 + (s3 - s1) ** 2))
    return dict(eps=e, vol=sum(e), U=U, U_dil=U_dil, U_dist=U_dist, svm=svm, U_vm=(1 + nu) / (3 * E) * svm ** 2,
                E=E, nu=nu)


# 3 ---- failure envelopes ---------------------------------------------------------------------------------------------------
def envelopes(sy, nu, st, sc):
    """Plane-stress (sigma_1, sigma_2) loci for Tresca, Von Mises, Rankine, St Venant (max normal strain) and Mohr-Coulomb."""
    tresca = np.array([(sy, 0), (sy, sy), (0, sy), (-sy, 0), (-sy, -sy), (0, -sy), (sy, 0)], float)
    t = np.linspace(0, 2 * math.pi, 181)
    # sigma1^2 - sigma1 sigma2 + sigma2^2 = sy^2   ->  param by rotation of the principal axes
    a, b = math.sqrt(2) * sy, math.sqrt(2.0 / 3.0) * sy
    u, v = a * np.cos(t), b * np.sin(t)
    vm = np.c_[(u / math.sqrt(2) - v / math.sqrt(2)) * 1.0, (u / math.sqrt(2) + v / math.sqrt(2)) * 1.0]
    rank = np.array([(sy, sy), (-sy, sy), (-sy, -sy), (sy, -sy), (sy, sy)], float)
    A = np.array([[1, -nu], [-nu, 1]])
    sv = []
    for e1, e2 in ((1, 1), (1, -1), (-1, -1), (-1, 1), (1, 1)):          # intersections of the four strain lines
        if (e1, e2) == (1, -1) or (e1, e2) == (-1, 1):
            sv.append(tuple(np.linalg.solve(A, [e1 * sy, e2 * sy])))
        else:
            sv.append(tuple(np.linalg.solve(A, [e1 * sy, e2 * sy])))
    sv = np.array(sv)
    mc = np.array([(st, st), (0, st), (-sc, 0), (-sc, -sc), (0, -sc), (st, 0), (st, st)], float)
    return dict(tresca=tresca, vm=vm, rankine=rank, stvenant=sv, mc=mc)


# 4 ---- thin-walled shells: hoop / longitudinal stress and thin-ring compression -------------------------------------------
def thin_shell(key, o, N):
    sh = ASSUME['shell'].get(key)
    if sh is None or math.isinf(o['R']):
        return None
    R, t, p, sy = o['R'], sh['t'], sh['p'], sh['sy']
    E, nu, b = o['E'], o['nu'], ASSUME['pad_width']
    hoop, longi = p * R / t, p * R / (2 * t)                         # sigma_theta = pR/t, sigma_L = pR/2t
    I_t = t ** 3 / 12.0                                              # ring wall, per unit width
    P = N / b                                                        # diametral line load (N/m)
    M = 0.318 * P * R                                                # |M| at load point of a ring pressed by a diametral load
    sig_b = 6.0 * M / t ** 2                                         # bending stress in the wall
    delta = 0.149 * P * R ** 3 / (E / (1 - nu ** 2) * I_t)           # change of diameter along the load (thin ring)
    sig_tot = sig_b + hoop                                           # pressure is tensile; superposed with the bending at the load line
    sig_net = abs(sig_b - hoop)
    N_allow = sy / (6 * 0.318 * R / (b * t ** 2)) if sy else math.inf   # N giving sig_b = sy (empty shell)
    N_allow_p = (sy + hoop) / (6 * 0.318 * R / (b * t ** 2))             # tension side limited by sy, pressure shifts the mean
    return dict(R=R, t=t, p=p, hoop=hoop, longi=longi, sig_b=sig_b, delta=delta, sy=sy, N_allow=N_allow, N_allow_p=N_allow_p,
                fos=sy / max(max(sig_b + hoop, abs(sig_b - hoop)), 1e-9), P=P, M=M)


# 5 ---- arm link as a cantilever beam: SFD / BMD, bending stress, deflection ------------------------------------------------
def arm_beams(m_obj, n_f=5, mu=0.6, F_max=40.0, L1=0.40, L2=0.40):
    A = ASSUME
    OD, ID = A['arm_OD'], A['arm_ID']
    I = math.pi / 64 * (OD ** 4 - ID ** 4)
    Ar = math.pi / 4 * (OD ** 2 - ID ** 2)
    c = OD / 2
    E, w = A['arm_E'], A['arm_rho'] * Ar * G
    P = (m_obj + A['hand_mass']) * G                                   # payload at the wrist (link-2 tip), arm fully stretched
    x2 = np.linspace(0, L2, 60)
    V2 = P + w * (L2 - x2)
    M2 = P * (L2 - x2) + w * (L2 - x2) ** 2 / 2
    F1 = P + w * L2                                                     # reaction passed from link 2 to link 1
    Mt = P * L2 + w * L2 ** 2 / 2
    x1 = np.linspace(0, L1, 60)
    V1 = F1 + w * (L1 - x1)
    M1 = F1 * (L1 - x1) + Mt + w * (L1 - x1) ** 2 / 2
    sig2, sig1 = M2.max() * c / I, M1.max() * c / I
    d2 = P * L2 ** 3 / (3 * E * I) + w * L2 ** 4 / (8 * E * I)
    d1 = F1 * L1 ** 3 / (3 * E * I) + Mt * L1 ** 2 / (2 * E * I) + w * L1 ** 4 / (8 * E * I)
    M_self = w * L1 ** 2 / 2 + w * L2 * (L1 + L2 / 2) + A['hand_mass'] * G * (L1 + L2)
    M_allow = A['arm_sy'] / A['arm_fos'] * I / c
    m_beam = max(0.0, (M_allow - M_self) / (L1 + L2) / G)             # payload that brings link 1 to sy / FoS
    m_grip = n_f * mu * F_max / G
    return dict(I=I, c=c, w=w, P=P, x1=x1, x2=x2, V1=V1, V2=V2, M1=M1, M2=M2, sig1=sig1, sig2=sig2,
                tip_defl=d1 + d2 + 0.0, d1=d1, d2=d2, fos=A['arm_sy'] / max(sig1, 1e-9),
                m_beam=m_beam, m_grip=m_grip, limit='bending of link 1' if m_beam < m_grip else 'grip friction')


# 6 ---- finger phalanx as a beam: Castigliano vs the spring-joint compliance in hand.py ----------------------------------------
def phalanx_castigliano(F, L, joint_compliance_mm_per_N, k_joint=None):
    """Straight finger (s = 0), unit force F normal to the distal phalanx at the tip.  M(x) = F (Ltot - x);
    dM/dF = (Ltot - x);  delta = integral M (dM/dF) / EI dx, evaluated segment by segment (exact)."""
    A = ASSUME
    EI = A['ph_E'] * A['ph_b'] * A['ph_h'] ** 3 / 12.0
    Lt = sum(L)
    d, x0 = 0.0, 0.0
    for Lk in L:
        a, b = Lt - x0 - Lk, Lt - x0                                  # lever arms at the two ends of the segment
        d += F * (b ** 3 - a ** 3) / (3 * EI)
        x0 += Lk
    d_joint = 0.0
    if k_joint:
        lev = [Lt, Lt - L[0], Lt - L[0] - L[1]]                       # perpendicular distance of the tip from joint k
        d_joint = F * sum(l ** 2 / k for l, k in zip(lev, k_joint))
    return dict(EI=EI, delta_beam=d, delta_joint=d_joint, ratio=(d / d_joint if d_joint > 0 else math.nan), Lt=Lt)


# 7 ---- Euler buckling of the thin tube / classical shell buckling ------------------------------------------------------------
def buckling(key, o, N_total, lift_load):
    sh = ASSUME['shell'].get(key)
    if sh is None or math.isinf(o['R']) or o['shape'] != 'cyl':
        return None
    R, t, E, nu, L = o['R'], sh['t'], o['E'], o['nu'], o['dims'][1]
    I = math.pi * R ** 3 * t                                          # thin tube, I = pi R^3 t
    out = {}
    for name, Kf in (('pinned-pinned (K=1)', 1.0), ('fixed-fixed (K=0.5)', 0.5), ('cantilever (K=2)', 2.0)):
        out[name] = math.pi ** 2 * E * I / (Kf * L) ** 2
    P_shell = 2 * math.pi * E * t ** 2 / math.sqrt(3 * (1 - nu ** 2)) * 0.2      # classical axial shell load x 0.2 knock-down
    return dict(I=I, L=L, euler=out, P_cr=out['pinned-pinned (K=1)'], P_shell=P_shell, squeeze=N_total, lift=lift_load,
                ratio=out['pinned-pinned (K=1)'] / max(N_total, 1e-9))


# 8 ---- torsion of the wrist shaft -------------------------------------------------------------------------------------------
def wrist_torsion(m_obj, e=None):
    A = ASSUME
    e = A['com_offset'] if e is None else e
    T = m_obj * G * e             # moment from the off-centre object weight
    J = math.pi / 32 * (A['sh_OD'] ** 4 - A['sh_ID'] ** 4)
    r = A['sh_OD'] / 2
    tau = T * r / J
    phi = T * A['sh_L'] / (A['sh_G'] * J)
    return dict(T=T, J=J, tau=tau, twist_deg=math.degrees(phi), tau_allow=A['sh_sy'] / 2, fos=(A['sh_sy'] / 2) / max(tau, 1e-9), e=e)


# 9 ---- numerical verification of the Hertz solution (Boussinesq superposition) -----------------------------------------------
def hertz_check(p0, a, Ep, n=4000):
    """Superpose the Boussinesq point-load solution over the Hertz pressure p(r) = p0 sqrt(1 - r^2/a^2) and compare with the
    closed-form axis stress sigma_z = -p0/(1+zeta^2) and the centre displacement u_z = (1-nu^2) pi p0 a / (2E)."""
    r = (np.arange(n) + 0.5) * (a / n)
    dr = a / n
    p = p0 * np.sqrt(1 - (r / a) ** 2)
    z = np.linspace(0.02, 3.0, 60) * a
    sz = np.array([-3 * zz ** 3 * np.sum(p * r / (r * r + zz * zz) ** 2.5) * dr for zz in z])
    closed = -p0 / (1 + (z / a) ** 2)
    uz_num = 2.0 / Ep * np.sum(p) * dr                      # Ep = E/(1-nu^2) plane-strain modulus
    uz_cl = math.pi * p0 * a / (2.0 * Ep)
    z48 = 0.48 * a
    s48 = -3 * z48 ** 3 * np.sum(p * r / (r * r + z48 * z48) ** 2.5) * dr
    return dict(zeta=z / a, num=sz / p0, closed=closed / p0, err48=abs(s48 - (-p0 / (1 + 0.48 ** 2))) / (p0 / (1 + 0.48 ** 2)),
                uz_num=uz_num, uz_cl=uz_cl, err_u=abs(uz_num - uz_cl) / uz_cl)


# 10 --- plane strain: 2-D line contact (Flamant / Airy) vs the 3-D point contact ------------------------------------------------
def plane_strain(N, Es, Re, p0_3d, nu):
    Lc = ASSUME['pad_width']
    Fp = N / Lc
    b = math.sqrt(4 * Fp * Re / (math.pi * Es))                       # half-width of the line contact
    p0 = 2 * Fp / (math.pi * b)
    zeta = np.linspace(0, 3, 61)
    sz = -p0 / np.sqrt(1 + zeta ** 2)
    sx = -p0 * ((1 + 2 * zeta ** 2) / np.sqrt(1 + zeta ** 2) - 2 * zeta)
    sy_ = nu * (sx + sz)                                              # plane strain: eps_y = 0
    tau = np.abs(sx - sz) / 2
    return dict(b=b, p0=p0, zeta=zeta, sz=sz, sx=sx, sy=sy_, tau=tau, tau_max=tau.max(), z_tau=zeta[int(np.argmax(tau))] * b,
                p0_3d=p0_3d, ratio=p0 / max(p0_3d, 1e-9), Fp=Fp)


# ---------------------------------------------------------------------------------------------------------------------------
_HC = {}








# ---------------------------------------------------------------------------------------------------------------------------


ASSUME.update(
    channel=dict(h=0.022, b=0.020, t=0.002),               # C-channel finger housing (web h, flange b, wall t)
    angle=dict(a=0.020, t=0.003),                          # equal-leg angle section (principal-axis demo)
    load_angle_deg=20.0,                                   # direction of the tip force relative to the section's principal axis
    tube_demo={'glass': 50e3, 'wet_glass': 50e3, 'bottle_empty': 300e3, 'bottle_full': 300e3},   # sealed / pressurised case (Pa)
    plane_deg=30.0,                                        # user-chosen cut plane for Cauchy's formula
)


# ---- traction vector, Cauchy's formula, tetrahedron equilibrium, symmetry ---------------------------------------
def traction(S, ang_deg=None):
    ang = ASSUME['plane_deg'] if ang_deg is None else ang_deg
    planes = [('contact plane (n = z)', (0.0, 0.0, 1.0)),
              ('45 deg plane in r-z', (math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4))),
              (f'{ang:.0f} deg plane in r-z', (math.sin(math.radians(ang)), 0.0, math.cos(math.radians(ang))))]
    rows = []
    for name, n in planes:
        n = np.array(n)
        t = S @ n
        sn = float(t @ n)
        tau = float(np.linalg.norm(t - sn * n))
        # Cauchy tetrahedron: traction on the slanted face minus the three coordinate-face forces (each area = |n_j| dA)
        res = t - sum(-(-np.sign(n[j]) * S[:, j]) * abs(n[j]) for j in range(3) if n[j] != 0)
        rows.append(dict(name=name, n=n, t=t, sn=sn, tau=tau, resid=float(np.linalg.norm(res))))
    th = np.radians(np.linspace(0, 180, 181))
    N = np.stack([np.sin(th), np.zeros_like(th), np.cos(th)], 1)
    T = N @ S.T
    sn_c = np.einsum('ij,ij->i', T, N)
    tau_c = np.linalg.norm(T - sn_c[:, None] * N, axis=1)
    return dict(rows=rows, theta=np.degrees(th), sn=sn_c, tau=tau_c, sym=float(np.abs(S - S.T).max()),
                tau_peak=float(tau_c.max()), th_peak=float(np.degrees(th[int(np.argmax(tau_c))])))


# ---- invariants, octahedral plane, hydrostatic + deviatoric split ---------------------------------------------------
def invariants(S):
    I1 = float(np.trace(S))
    I2 = float(0.5 * (I1 ** 2 - np.trace(S @ S)))
    I3 = float(np.linalg.det(S))
    sm = I1 / 3.0
    dev = S - sm * np.eye(3)
    J2 = float(0.5 * np.sum(dev * dev))
    p = np.sort(np.linalg.eigvalsh(S))[::-1]
    tau_oct = math.sqrt(2.0 * J2 / 3.0)
    svm = math.sqrt(3.0 * J2)
    return dict(I1=I1, I2=I2, I3=I3, sig_oct=sm, tau_oct=tau_oct, J2=J2, svm=svm, dev=dev,
                chk_vm=abs(tau_oct - math.sqrt(2) / 3 * svm), chk_I3=abs(I3 - float(np.prod(p))) / max(abs(I3), 1e-30))


# ---- isotropic constants and their relations -----------------------------------------------------------------------
def constants(E, nu):
    G = E / (2 * (1 + nu))
    K = E / (3 * (1 - 2 * nu))
    lam = E * nu / ((1 + nu) * (1 - 2 * nu))
    return dict(E=E, nu=nu, G=G, K=K, lam=lam, chkE=abs(E - 9 * K * G / (3 * K + G)) / E,
                chkNu=abs(nu - (3 * K - 2 * G) / (2 * (3 * K + G))), M=lam + 2 * G)


# ---- reciprocity (Maxwell-Betti) and Castigliano for the finger springs; curved beam (thin ring) --------------------
def reciprocity(s, k_joint):
    from .hand import jacobian
    J = np.array([[c[0] for c in jacobian(s)], [c[1] for c in jacobian(s)]])          # tip displacement per joint rotation (2x3)
    Kinv = np.diag([1.0 / k for k in k_joint])
    C = J @ Kinv @ J.T                                                                 # tip flexibility matrix
    a, b = np.array([1.0, 0.0]), np.array([math.cos(1.0), math.sin(1.0)])
    d_ab, d_ba = float(b @ C @ a), float(a @ C @ b)                                    # displacement along b due to unit force a, and vice versa
    F = np.array([3.0, -2.0])
    U = lambda f: 0.5 * float(f @ C @ f)                                               # strain energy of the joint springs
    h = 1e-6
    grad = np.array([(U(F + h * e) - U(F - h * e)) / (2 * h) for e in np.eye(2)])      # Castigliano: u = dU/dF
    return dict(C=C, d_ab=d_ab, d_ba=d_ba, sym_err=abs(d_ab - d_ba) / max(abs(d_ab), 1e-30),
                cast_err=float(np.linalg.norm(grad - C @ F) / np.linalg.norm(C @ F)))


def ring_castigliano(n=4000):
    """Thin ring under diametral load P (R, EI = 1).  Quarter ring measured from the horizontal section:
    M(phi) = M0 - (P R / 2)(1 - cos phi);  dU/dM0 = 0  ->  M0 = P R (1/2 - 1/pi).  Diametral shortening: delta = 4 int M dM/dP R dphi / EI."""
    phi = (np.arange(n) + 0.5) * (math.pi / 2) / n
    dphi = (math.pi / 2) / n
    M0 = 0.5 - 1.0 / math.pi
    M = M0 - 0.5 * (1 - np.cos(phi))
    chk = float(np.sum(M) * dphi)                                  # dU/dM0 ~ int M dphi must vanish
    dM_dP = -0.5 * (1 - np.cos(phi))
    delta = 4.0 * float(np.sum(M * dM_dP) * dphi)
    return dict(M_load=abs(M0 - 0.5), M_side=M0, delta_coef=abs(delta), dU_dM0=chk,
                err_M=abs(abs(M0 - 0.5) - 1 / math.pi), err_d=abs(abs(delta) - (math.pi / 4 - 2 / math.pi)))


# ---- unsymmetric bending ------------------------------------------------------------------------------------------
def unsymmetric_bending(F, L, E, b, h, psi_deg):
    Iy, Iz = b * h ** 3 / 12.0, h * b ** 3 / 12.0                  # y across the width b, z along the depth h
    psi = math.radians(psi_deg)
    Fz, Fy = F * math.cos(psi), F * math.sin(psi)
    My, Mz = Fz * L, Fy * L
    corners = [(y, z) for y in (-b / 2, b / 2) for z in (-h / 2, h / 2)]
    sig = [My * z / Iy - Mz * y / Iz for y, z in corners]
    alpha = math.degrees(math.atan2(Iy * math.sin(psi), Iz * math.cos(psi))) if abs(psi_deg) > 1e-9 else 0.0
    dz, dy = Fz * L ** 3 / (3 * E * Iy), Fy * L ** 3 / (3 * E * Iz)
    return dict(Iy=Iy, Iz=Iz, My=My, Mz=Mz, corners=corners, sig=sig, sig_max=max(abs(s) for s in sig),
                neutral_deg=alpha, defl_dir_deg=math.degrees(math.atan2(dy, dz)), delta=math.hypot(dy, dz), psi=psi_deg,
                sig_sym=abs(My) * (h / 2) / Iy, b=b, h=h)


def angle_section(a, t):
    parts = [(a, t, a / 2, t / 2), (t, a - t, t / 2, t + (a - t) / 2)]           # (width, height, cx, cy)
    A = sum(w * hh for w, hh, _, _ in parts)
    cx = sum(w * hh * x for w, hh, x, _ in parts) / A
    cy = sum(w * hh * y for w, hh, _, y in parts) / A
    Ixx = sum(w * hh ** 3 / 12 + w * hh * (y - cy) ** 2 for w, hh, _, y in parts)
    Iyy = sum(hh * w ** 3 / 12 + w * hh * (x - cx) ** 2 for w, hh, x, _ in parts)
    Ixy = sum(w * hh * (x - cx) * (y - cy) for w, hh, x, y in parts)
    avg, R = (Ixx + Iyy) / 2, math.hypot((Ixx - Iyy) / 2, Ixy)
    return dict(A=A, cx=cx, cy=cy, Ixx=Ixx, Iyy=Iyy, Ixy=Ixy, I1=avg + R, I2=avg - R,
                theta_p=0.5 * math.degrees(math.atan2(-2 * Ixy, Ixx - Iyy)))


# ---- shear stress, shear flow, shear centre, Timoshenko vs Euler-Bernoulli ----------------------------------------
def shear_beam(V, P_tip, L, nu=0.33):
    A = ASSUME
    Ro, Ri = A['arm_OD'] / 2, A['arm_ID'] / 2
    I = math.pi / 4 * (Ro ** 4 - Ri ** 4)
    Ar = math.pi * (Ro ** 2 - Ri ** 2)
    Q = 2.0 / 3.0 * (Ro ** 3 - Ri ** 3)
    bw = 2 * (Ro - Ri)
    tau_max = V * Q / (I * bw)
    G = A['arm_E'] / (2 * (1 + nu))
    kappa = 2 * (1 + nu) / (4 + 3 * nu)                         # Timoshenko shear coefficient, thin circular tube
    d_b = P_tip * L ** 3 / (3 * A['arm_E'] * I)
    d_s = P_tip * L / (kappa * G * Ar)
    LD = np.linspace(2, 30, 40)
    ratio = (P_tip * (LD * A['arm_OD']) / (kappa * G * Ar)) / (P_tip * (LD * A['arm_OD']) ** 3 / (3 * A['arm_E'] * I))
    c = A.get('channel')
    h, b, t = c['h'], c['b'], c['t']
    Ic = t * h ** 3 / 12 + 2 * (b * t * (h / 2) ** 2)
    s = np.linspace(0, b, 60)
    q_fl = V * (t * s) * (h / 2) / Ic                           # shear flow in a flange, s from the free tip
    F_fl = float(np.sum(q_fl) * (s[1] - s[0]))
    e_num = F_fl * h / max(V, 1e-12)
    e_cl = 3 * b ** 2 / (h + 6 * b)
    return dict(tau_max=tau_max, tau_avg=V / Ar, ratio_tau=tau_max / (V / Ar), delta_b=d_b, delta_s=d_s,
                shear_share=d_s / (d_b + d_s), kappa=kappa, LD=LD, ratio=ratio, s=s, q=q_fl, q_max=float(q_fl.max()),
                e_num=e_num, e_cl=e_cl, Ic=Ic, ch=(h, b, t))


# ---- tube under inflation (Lame), extension and torsion --------------------------------------------------------
def tube_combined(key, o, m_obj, e=None):
    sh = ASSUME['shell'].get(key)
    if sh is None or key in ('egg',) or o['shape'] != 'cyl' or math.isinf(o['R']):
        return None
    ro, t = o['R'], sh['t']
    ri = ro - t
    A = ASSUME
    out = {}
    e = A['com_offset'] if e is None else e
    Ar = math.pi * (ro ** 2 - ri ** 2)
    J = math.pi / 2 * (ro ** 4 - ri ** 4)
    W, T = m_obj * G, m_obj * G * e
    for lab, p in (('as filled', sh['p']), ('pressurised', A['tube_demo'].get(key, 0.0))):
        a = p * ri ** 2 / (ro ** 2 - ri ** 2)
        b_ = p * ri ** 2 * ro ** 2 / (ro ** 2 - ri ** 2)
        sz_infl = p * ri ** 2 / (ro ** 2 - ri ** 2)                          # closed ends
        rows = []
        for rr, nm in ((ri, 'inner'), (ro, 'outer')):
            sr = a - b_ / rr ** 2
            sth = a + b_ / rr ** 2
            sz = sz_infl + W / Ar
            tau = T * rr / J
            S = np.array([[sr, 0, 0], [0, sth, tau], [0, tau, sz]])
            w = np.sort(np.linalg.eigvalsh(S))[::-1]
            vm = math.sqrt(0.5 * ((w[0] - w[1]) ** 2 + (w[1] - w[2]) ** 2 + (w[2] - w[0]) ** 2))
            rows.append(dict(where=nm, sr=sr, sth=sth, sz=sz, tau=tau, p=w, vm=vm, tresca=w[0] - w[2]))
        rr = np.linspace(ri, ro, 40)
        out[lab] = dict(p=p, rows=rows, r=rr, sr=a - b_ / rr ** 2, sth=a + b_ / rr ** 2)
    return dict(ri=ri, ro=ro, t=t, W=W, T=T, Ar=Ar, J=J, e=e, sy=sh['sy'], cases=out,
                thin_hoop=sh['p'] * o['R'] / t if sh['p'] else 0.0)


# ---- displacement-based check of the elasticity solution, cylindrical equilibrium, compatibility ------------------
def _pt_stress(rho, z, Q, nu):
    """Boussinesq point load Q (compressive, into the half-space), tension positive; cylindrical about the load axis."""
    R = np.sqrt(rho ** 2 + z ** 2)
    sz = -3 * Q * z ** 3 / (2 * np.pi * R ** 5)
    trz = -3 * Q * rho * z ** 2 / (2 * np.pi * R ** 5)
    sr = Q / (2 * np.pi) * ((1 - 2 * nu) / (R * (R + z)) - 3 * rho ** 2 * z / R ** 5)
    st = Q / (2 * np.pi) * (1 - 2 * nu) * (z / R ** 3 - 1.0 / (R * (R + z)))
    return sr, st, sz, trz


def _pt_disp(r, z, Q, nu, G):
    R = math.sqrt(r * r + z * z)
    ur = Q / (4 * math.pi * G) * (r * z / R ** 3 - (1 - 2 * nu) * r / (R * (R + z)))
    uz = Q / (4 * math.pi * G) * (z * z / R ** 3 + 2 * (1 - nu) / R)
    return ur, uz


def displacement_check(nu=0.3, G=1.0, Q=1.0, r=0.7, z=0.9, h=1e-5):
    """Differentiate the displacement field -> strains, apply Hooke, compare with the stress formulas
    and evaluate the local rotation omega = 1/2 (du_r/dz - du_z/dr)."""
    ur = lambda rr, zz: _pt_disp(rr, zz, Q, nu, G)[0]
    uz = lambda rr, zz: _pt_disp(rr, zz, Q, nu, G)[1]
    d = lambda f, i: (f(r + h, z) - f(r - h, z)) / (2 * h) if i == 0 else (f(r, z + h) - f(r, z - h)) / (2 * h)
    er, ez, eth = d(ur, 0), d(uz, 1), ur(r, z) / r
    g = d(ur, 1) + d(uz, 0)
    lam = 2 * G * nu / (1 - 2 * nu)
    e = er + ez + eth
    s_h = (lam * e + 2 * G * er, lam * e + 2 * G * eth, lam * e + 2 * G * ez, G * g)
    s_f = tuple(float(v) for v in _pt_stress(r, z, Q, nu))
    err = max(abs(a - b) / max(abs(b), 1e-12) for a, b in zip(s_h, s_f))
    return dict(er=er, eth=eth, ez=ez, gam=g, vol=e, omega=0.5 * (d(ur, 1) - d(uz, 0)), err=err)


_EQ = {}


def _hertz_field(r, z, nu, nr=48, nphi=72):
    """Full stress at (r, z) under the Hertz pressure p0 = 1, a = 1 by superposing point loads on a polar grid."""
    rs = (np.arange(nr) + 0.5) / nr
    ph = (np.arange(nphi) + 0.5) * 2 * np.pi / nphi
    RS, PH = np.meshgrid(rs, ph, indexing='ij')
    Q = np.sqrt(np.clip(1 - RS ** 2, 0, None)) * RS * (1.0 / nr) * (2 * np.pi / nphi)
    dx, dy = r - RS * np.cos(PH), -RS * np.sin(PH)
    rho = np.hypot(dx, dy)
    al = np.arctan2(dy, dx)
    sr, st, sz, tz = _pt_stress(rho, z, Q, nu)
    c, s = np.cos(al), np.sin(al)
    sxx = sr * c ** 2 + st * s ** 2
    syy = sr * s ** 2 + st * c ** 2
    txz = tz * c
    return float(sxx.sum()), float(syy.sum()), float(sz.sum()), float(txz.sum())     # (sigma_r, sigma_theta, sigma_z, tau_rz)


def equilibrium_check(nu):
    key = round(nu, 4)
    if key in _EQ: return _EQ[key]
    h = 0.02
    rows = []
    for r, z in ((0.5, 0.5), (0.8, 0.3), (1.2, 0.6), (0.3, 1.0)):
        f0 = _hertz_field(r, z, nu)
        fr1, fr0 = _hertz_field(r + h, z, nu), _hertz_field(r - h, z, nu)
        fz1, fz0 = _hertz_field(r, z + h, nu), _hertz_field(r, z - h, nu)
        d_r = [(a - b) / (2 * h) for a, b in zip(fr1, fr0)]
        d_z = [(a - b) / (2 * h) for a, b in zip(fz1, fz0)]
        e1 = d_r[0] + d_z[3] + (f0[0] - f0[1]) / r                  # d sr/dr + d trz/dz + (sr - sth)/r
        e2 = d_r[3] + d_z[2] + f0[3] / r                            # d trz/dr + d sz/dz + trz/r
        sc = max(abs(d_r[0]), abs(d_z[3]), abs(d_z[2]), abs(d_r[3]), abs((f0[0] - f0[1]) / r), 1e-9)
        rows.append(dict(r=r, z=z, e1=e1, e2=e2, rel=max(abs(e1), abs(e2)) / sc, sig=f0))
    # also compare with the closed-form axis solution: sigma_z and sigma_r at z = 0.48a on the axis
    zeta = 0.48
    fa = _hertz_field(1e-3, zeta, nu)
    sz_cl = -1 / (1 + zeta ** 2)
    sr_cl = -((1 + nu) * (1 - zeta * math.atan(1 / zeta)) - 0.5 / (1 + zeta ** 2))
    _EQ.clear()
    _EQ[key] = dict(rows=rows, worst=max(r['rel'] for r in rows), err_sz=abs(fa[2] - sz_cl) / abs(sz_cl),
                    err_sr=abs(fa[0] - sr_cl) / abs(sr_cl))
    return _EQ[key]


def compatibility_check(nu=0.3, E=1.0, P=1.0, h=2e-3):
    """Plane strain, Flamant line load.  Equilibrium, Laplacian of (sx+sz) (Airy: biharmonic phi) and the St Venant compatibility
    condition d2ex/dz2 + d2ez/dx2 = d2gxz/dxdz evaluated by finite differences."""
    def sig(x, z):
        R2 = x * x + z * z
        return (-2 * P * x * x * z / (math.pi * R2 ** 2), -2 * P * z ** 3 / (math.pi * R2 ** 2), -2 * P * x * z * z / (math.pi * R2 ** 2))
    def eps(x, z):
        sx, sz, t = sig(x, z)
        return ((1 + nu) / E * ((1 - nu) * sx - nu * sz), (1 + nu) / E * ((1 - nu) * sz - nu * sx), 2 * (1 + nu) / E * t)
    rows = []
    for x, z in ((0.5, 0.8), (1.0, 0.6), (0.3, 1.1)):
        d2 = lambda f, i, dx, dz: (f(x + dx, z + dz) - 2 * f(x, z) + f(x - dx, z - dz)) / (h * h)
        ex = lambda a, b: eps(a, b)[0]; ez = lambda a, b: eps(a, b)[1]
        lap = lambda f: d2(f, 0, h, 0) + d2(f, 0, 0, h)
        s_sum = lambda a, b: sig(a, b)[0] + sig(a, b)[1]
        g = lambda a, b: eps(a, b)[2]
        mix = (g(x + h, z + h) - g(x + h, z - h) - g(x - h, z + h) + g(x - h, z - h)) / (4 * h * h)
        comp = d2(ex, 0, 0, h) + d2(ez, 0, h, 0) - mix
        scale = abs(d2(ex, 0, 0, h)) + abs(d2(ez, 0, h, 0)) + abs(mix)
        sc2 = abs(d2(lambda a, b: sig(a, b)[0], 0, h, 0)) + abs(d2(lambda a, b: sig(a, b)[1], 0, 0, h)) + 1e-12
        dx_ = lambda f: (f(x + h, z) - f(x - h, z)) / (2 * h)
        dz_ = lambda f: (f(x, z + h) - f(x, z - h)) / (2 * h)
        eq1 = dx_(lambda a, b: sig(a, b)[0]) + dz_(lambda a, b: sig(a, b)[2])
        eq2 = dx_(lambda a, b: sig(a, b)[2]) + dz_(lambda a, b: sig(a, b)[1])
        rows.append(dict(x=x, z=z, lap=lap(s_sum) / sc2, comp=comp / max(scale, 1e-12), eq=max(abs(eq1), abs(eq2)) / (abs(dx_(lambda a, b: sig(a, b)[0])) + 1e-12)))
    return dict(rows=rows, worst=max(max(abs(r['lap']), abs(r['comp']), abs(r['eq'])) for r in rows))


# ---- buckling of arm link and finger column ---------------------------------------------------------------------
def column_buckling(m_obj):
    A = ASSUME
    I_arm = math.pi / 64 * (A['arm_OD'] ** 4 - A['arm_ID'] ** 4)
    L_arm = 0.40
    I_f = min(A['ph_b'] * A['ph_h'] ** 3, A['ph_h'] * A['ph_b'] ** 3) / 12.0
    L_f = 0.143
    out = {}
    for nm, E, I, L, load, lab in (('arm link', A['arm_E'], I_arm, L_arm, (m_obj + A['hand_mass']) * G, 'payload weight'),
                                    ('finger', A['ph_E'], I_f, L_f, 40.0, 'F_max per finger')):
        out[nm] = dict(E=E, I=I, L=L, load=load, lab=lab,
                       Pcr={k: math.pi ** 2 * E * I / (K * L) ** 2 for k in (0.5, 0.7, 1.0, 2.0) for K in (k,)},
                       slender=L / math.sqrt(I / (A['arm_OD'] ** 2 if nm == 'arm link' else A['ph_b'] * A['ph_h'])))
    return out


# ---------------------------------------------------------------------------------------------------------------------------












ASSUME.update(
    composite=dict(b=0.020, layers=[('ABS body', 0.020, 2.3e9), ('steel strip', 0.002, 200e9)]),   # finger with a stiffening plate
    curved=dict(ri=0.040, h=0.022, b=0.020),              # curved finger link, Winkler-Bach
    ecc=0.005,                                            # load eccentricity for the secant formula (m)
    dT=30.0,                                              # temperature difference used for thermal stress (K)
    wood=dict(EL=11e9, ET=0.9e9, G=0.7e9, nuLT=0.37),     # orthotropic wood (assumed handbook values)
    sphere_demo=dict(ro=0.022, ri=0.015, p=100e3),        # thick-walled sphere demo
    notch=dict(d_over_w=0.30), fatigue=dict(Su=310e6, S_inf=96e6, N_inf=5e8, cycles=1e5),
)


# ---- 2-D transformation of stress, stress at many points, location of the maximum ---------------------------------
def stress_transform_2d(sx, sz, t, thetas_deg=(0, 30, 45, 60, 90)):
    S = np.array([[sx, 0, t], [0, 0, 0], [t, 0, sz]])
    rows = []
    for th in thetas_deg:
        a = math.radians(th)
        sxp = (sx + sz) / 2 + (sx - sz) / 2 * math.cos(2 * a) + t * math.sin(2 * a)
        szp = (sx + sz) / 2 - (sx - sz) / 2 * math.cos(2 * a) - t * math.sin(2 * a)
        tp = -(sx - sz) / 2 * math.sin(2 * a) + t * math.cos(2 * a)
        c, s = math.cos(a), math.sin(a)
        Rm = np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])            # axes rotated by a in the r-z plane
        Sr = Rm.T @ S @ Rm
        rows.append(dict(th=th, sxp=sxp, szp=szp, tp=tp, err=max(abs(Sr[0, 0] - sxp), abs(Sr[2, 2] - szp), abs(Sr[0, 2] - tp)),
                         inv=abs((sxp + szp) - (sx + sz))))
    R = math.hypot((sx - sz) / 2, t)
    return dict(rows=rows, centre=(sx + sz) / 2, radius=R, theta_p=0.5 * math.degrees(math.atan2(2 * t, sx - sz)),
                s1=(sx + sz) / 2 + R, s2=(sx + sz) / 2 - R)


def _hertz_cart(x, y, z, nu, nr=36, nphi=60):
    """Full Cartesian stress tensor at (x, y, z) under Hertz p0 = 1, a = 1 (point-load superposition)."""
    rs = (np.arange(nr) + 0.5) / nr
    ph = (np.arange(nphi) + 0.5) * 2 * np.pi / nphi
    RS, PH = np.meshgrid(rs, ph, indexing='ij')
    Q = np.sqrt(np.clip(1 - RS ** 2, 0, None)) * RS * (1.0 / nr) * (2 * np.pi / nphi)
    dx, dy = x - RS * np.cos(PH), y - RS * np.sin(PH)
    rho = np.hypot(dx, dy)
    al = np.arctan2(dy, dx)
    sr, st, sz, tz = _pt_stress(rho, z, Q, nu)
    c, s = np.cos(al), np.sin(al)
    return np.array([[(sr * c * c + st * s * s).sum(), ((sr - st) * s * c).sum(), (tz * c).sum()],
                     [((sr - st) * s * c).sum(), (sr * s * s + st * c * c).sum(), (tz * s).sum()],
                     [(tz * c).sum(), (tz * s).sum(), sz.sum()]])


_FLD = {}


def stress_map(nu):
    """tau_max/p0 and Von Mises/p0 on an (r, z) grid; location of their maxima."""
    k = round(nu, 4)
    if k in _FLD: return _FLD[k]
    rr, zz = np.linspace(0.0, 1.6, 17), np.linspace(0.1, 2.0, 20)
    T = np.zeros((len(zz), len(rr)))
    V = np.zeros_like(T)
    for i, z in enumerate(zz):
        for j, r in enumerate(rr):
            S = _hertz_cart(max(r, 1e-3), 0.0, z, nu)
            w = np.sort(np.linalg.eigvalsh(S))
            T[i, j] = (w[2] - w[0]) / 2
            V[i, j] = math.sqrt(0.5 * ((w[0] - w[1]) ** 2 + (w[1] - w[2]) ** 2 + (w[2] - w[0]) ** 2))
    it = np.unravel_index(np.argmax(T), T.shape)
    iv = np.unravel_index(np.argmax(V), V.shape)
    _FLD.clear()
    _FLD[k] = dict(r=rr, z=zz, T=T, V=V, tmax=float(T[it]), t_at=(float(rr[it[1]]), float(zz[it[0]])),
                   vmax=float(V[iv]), v_at=(float(rr[iv[1]]), float(zz[iv[0]])))
    return _FLD[k]


# ---- Cartesian equilibrium and body force ---------------------------------------------------------------------------
def cartesian_equilibrium(nu, rho_g):
    h = 0.02
    pts = ((0.5, 0.4, 0.5), (0.9, -0.3, 0.7), (0.2, 0.6, 1.0))
    rows = []
    for (x, y, z) in pts:
        f = lambda a, b, c: _hertz_cart(a, b, c, nu)
        d = lambda i, j, ax: ((f(x + h, y, z) if ax == 0 else f(x, y + h, z) if ax == 1 else f(x, y, z + h))[i, j]
                              - (f(x - h, y, z) if ax == 0 else f(x, y - h, z) if ax == 1 else f(x, y, z - h))[i, j]) / (2 * h)
        e = [d(i, 0, 0) + d(i, 1, 1) + d(i, 2, 2) for i in range(3)]     # div(sigma)_i, body force = 0 in the half-space
        sc = max(abs(d(0, 0, 0)), abs(d(2, 2, 2)), abs(d(0, 2, 2)), 1e-9)
        rows.append(dict(p=(x, y, z), res=max(abs(v) for v in e) / sc))
    # body force case: bar hanging by its top, z down, free bottom: sigma_z = rho g (L - z)
    L = 1.0
    sz = lambda z: rho_g * (L - z)
    dsz = (sz(0.5 + 1e-4) - sz(0.5 - 1e-4)) / 2e-4
    return dict(rows=rows, worst=max(r['res'] for r in rows), body_res=abs(dsz + rho_g) / abs(rho_g))


# ---- full small-strain / rotation split and cylindrical strain-displacement relations --------------------------------
def strain_rotation_split(nu, G=1.0, Q=1.0, x=0.5, y=0.5, z=0.9, h=1e-5):
    def u(px, py, pz):
        r = math.hypot(px, py)
        ur, uz = _pt_disp(r, pz, Q, nu, G)
        return np.array([ur * px / r, ur * py / r, uz])
    Gd = np.zeros((3, 3))
    for j, e in enumerate(np.eye(3)):
        Gd[:, j] = (u(x + h * e[0], y + h * e[1], z + h * e[2]) - u(x - h * e[0], y - h * e[1], z - h * e[2])) / (2 * h)
    eps, Om = 0.5 * (Gd + Gd.T), 0.5 * (Gd - Gd.T)
    omega = np.array([Om[2, 1], Om[0, 2], Om[1, 0]])
    lam = 2 * G * nu / (1 - 2 * nu)
    sig = lam * np.trace(eps) * np.eye(3) + 2 * G * eps
    r = math.hypot(x, y)
    sr, st, sz_, tz = _pt_stress(r, z, Q, nu)
    c, s = x / r, y / r
    sc = np.array([[sr * c * c + st * s * s, (sr - st) * s * c, tz * c], [(sr - st) * s * c, sr * s * s + st * c * c, tz * s], [tz * c, tz * s, sz_]])
    # cylindrical strains from the cylindrical strain-displacement relations
    ur_ = lambda rr, zz: _pt_disp(rr, zz, Q, nu, G)[0]
    uz_ = lambda rr, zz: _pt_disp(rr, zz, Q, nu, G)[1]
    er = (ur_(r + h, z) - ur_(r - h, z)) / (2 * h)
    et = ur_(r, z) / r
    ez = (uz_(r, z + h) - uz_(r, z - h)) / (2 * h)
    grz = (ur_(r, z + h) - ur_(r, z - h)) / (2 * h) + (uz_(r + h, z) - uz_(r - h, z)) / (2 * h)
    Rm = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
    ec = Rm @ eps @ Rm.T
    return dict(Gd=Gd, eps=eps, Om=Om, omega=omega, w_mag=float(np.linalg.norm(omega)), p_eps=np.sort(np.linalg.eigvalsh(eps))[::-1],
                err_hooke=float(np.abs(sig - sc).max() / np.abs(sc).max()),
                err_cyl=max(abs(ec[0, 0] - er), abs(ec[1, 1] - et), abs(ec[2, 2] - ez), abs(2 * ec[0, 2] - grz)) / max(abs(er), abs(et), abs(ez)),
                cyl=(er, et, ez, grz))


# ---- torsion of solid circular, rectangular (warping), thin-walled closed and open sections ---------------------------
def torsion_sections(T):
    A = ASSUME
    D, d = A['sh_OD'], A['sh_ID']
    Gm = A['sh_G']
    out = {}
    J = math.pi / 32 * D ** 4
    out['solid circle'] = dict(J=J, tau=T * (D / 2) / J, warp='none (plane sections stay plane)')
    Jh = math.pi / 32 * (D ** 4 - d ** 4)
    out['hollow tube (closed)'] = dict(J=Jh, tau=T * (D / 2) / Jh, warp='none')
    tw = (D - d) / 2
    Rm = (D + d) / 4
    Jc = 2 * math.pi * Rm ** 3 * tw                                   # Bredt-Batho, J = 4 Am^2 t / L_m
    Am = math.pi * Rm ** 2
    out['thin tube (Bredt-Batho)'] = dict(J=4 * Am ** 2 * tw / (2 * math.pi * Rm), tau=T / (2 * Am * tw), warp='none')
    Jo = 2 * math.pi * Rm * tw ** 3 / 3                                # same tube slit open: J = sum b t^3 / 3
    out['same tube slit open'] = dict(J=Jo, tau=T * tw / Jo, warp='out-of-plane warping')
    b, t = D, 0.7 * D                                                  # rectangle of similar size
    s = sum(math.tanh(n * math.pi * b / (2 * t)) / n ** 5 for n in range(1, 60, 2))
    Jr = b * t ** 3 / 3 * (1 - 192 * t / (math.pi ** 5 * b) * s)
    out['rectangle (St Venant)'] = dict(J=Jr, tau=T * (3 + 1.8 * t / b) / (b * t ** 2), warp='warps; max shear at mid of long side')
    for v in out.values(): v['twist_deg'] = math.degrees(T * A['sh_L'] / (Gm * v['J']))
    out['_ratio_open_closed'] = out['thin tube (Bredt-Batho)']['J'] / out['same tube slit open']['J']
    out['_chk_bredt'] = abs(out['thin tube (Bredt-Batho)']['J'] - out['hollow tube (closed)']['J']) / out['hollow tube (closed)']['J']
    return out


# ---- composite (transformed section) and curved (Winkler-Bach) bending -----------------------------------------------
def composite_bending(M):
    c = ASSUME['composite']
    b, layers = c['b'], c['layers']
    y0, rows = 0.0, []
    for nm, t, E in layers:
        rows.append((nm, t, E, y0 + t / 2)); y0 += t
    EA = sum(E * b * t for _, t, E, _ in rows)
    yn = sum(E * b * t * yc for _, t, E, yc in rows) / EA
    EI = sum(E * (b * t ** 3 / 12 + b * t * (yc - yn) ** 2) for _, t, E, yc in rows)
    H = y0
    out = []
    for nm, t, E, yc in rows:
        ytop, ybot = yc + t / 2 - yn, yc - t / 2 - yn
        out.append(dict(name=nm, E=E, s_top=-E * M * ytop / EI, s_bot=-E * M * ybot / EI))
    E1 = rows[0][2]
    EI_hom = E1 * b * H ** 3 / 12
    ok = abs(sum(E * b * t * (yc - yn) for _, t, E, yc in rows)) / (EA * H)          # zero net axial force
    return dict(yn=yn, EI=EI, EI_hom=EI_hom, gain=EI / EI_hom, layers=out, H=H, chk=ok, kappa=M / EI)


def winkler_bach(M):
    c = ASSUME['curved']
    ri, h, b = c['ri'], c['h'], c['b']
    ro, rc, A = ri + h, ri + h / 2, b * h
    rn = h / math.log(ro / ri)
    e = rc - rn
    s = lambda r: M * (r - rn) / (A * e * r)
    s_str = 6 * M / (b * h ** 2)
    far = ASSUME['curved'].copy(); far['ri'] = 100.0
    rn2 = h / math.log((100 + h) / 100.0); e2 = (100 + h / 2) - rn2
    s_far = M * (100 - rn2) / (A * e2 * 100)
    return dict(ri=ri, ro=ro, rn=rn, rc=rc, e=e, s_in=s(ri), s_out=s(ro), s_straight=s_str, ratio_in=abs(s(ri)) / s_str,
                far_ratio=abs(s_far) / s_str, shift=rc - rn)


# ---- thin-walled open sections: shear flow, shear centre (numerical), Timoshenko ---------------------------------------
def thin_section_shear_centre(nodes, t):
    """Open thin-walled section given as a chain of nodes (free end to free end), constant thickness t.  Shear flow from
    q(s) = -(Vx Ixx - Vy Ixy)/D int t x ds - (Vy Iyy - Vx Ixy)/D int t y ds, shear centre from the moment of the flow resultant."""
    P = np.array(nodes, float)
    seg = P[1:] - P[:-1]
    ln = np.linalg.norm(seg, axis=1)
    A = float(np.sum(t * ln))
    mid = (P[1:] + P[:-1]) / 2
    xc, yc = float(np.sum(t * ln * mid[:, 0]) / A), float(np.sum(t * ln * mid[:, 1]) / A)
    P = P - np.array([xc, yc])
    seg = P[1:] - P[:-1]
    mid = (P[1:] + P[:-1]) / 2
    u = seg / ln[:, None]
    Ixx = float(np.sum(t * ln ** 3 * u[:, 1] ** 2 / 12 + t * ln * mid[:, 1] ** 2))
    Iyy = float(np.sum(t * ln ** 3 * u[:, 0] ** 2 / 12 + t * ln * mid[:, 0] ** 2))
    Ixy = float(np.sum(t * ln ** 3 * u[:, 0] * u[:, 1] / 12 + t * ln * mid[:, 0] * mid[:, 1]))
    D = Ixx * Iyy - Ixy ** 2
    def resultants(Vx, Vy, n=200):
        cx, cy, F, M = 0.0, 0.0, np.zeros(2), 0.0
        for i in range(len(ln)):
            s = (np.arange(n) + 0.5) * ln[i] / n
            ds = ln[i] / n
            x = P[i, 0] + u[i, 0] * s
            y = P[i, 1] + u[i, 1] * s
            ix = cx + t * (P[i, 0] * s + u[i, 0] * s ** 2 / 2)          # int t x ds from the free end
            iy = cy + t * (P[i, 1] * s + u[i, 1] * s ** 2 / 2)
            q = -((Vx * Ixx - Vy * Ixy) / D * ix + (Vy * Iyy - Vx * Ixy) / D * iy)
            F += np.array([np.sum(q * u[i, 0]) * ds, np.sum(q * u[i, 1]) * ds])
            M += float(np.sum(q * (x * u[i, 1] - y * u[i, 0])) * ds)
            cx += t * (P[i, 0] * ln[i] + u[i, 0] * ln[i] ** 2 / 2)
            cy += t * (P[i, 1] * ln[i] + u[i, 1] * ln[i] ** 2 / 2)
        return F, M
    Fy, My_ = resultants(0.0, 1.0)
    Fx, Mx_ = resultants(1.0, 0.0)
    sgn_y = 1.0 if Fy[1] > 0 else -1.0
    sgn_x = 1.0 if Fx[0] > 0 else -1.0
    xs = sgn_y * My_ / (abs(Fy[1]) if abs(Fy[1]) > 0 else 1.0)
    ys = -sgn_x * Mx_ / (abs(Fx[0]) if abs(Fx[0]) > 0 else 1.0)
    return dict(xc=xc, yc=yc, xs=xs, ys=ys, Ixx=Ixx, Iyy=Iyy, Ixy=Ixy, F_y=float(abs(Fy[1])), F_x=float(abs(Fx[0])))


def shear_centres():
    c = ASSUME['channel']; h, b, t = c['h'], c['b'], c['t']
    ch = thin_section_shear_centre([(b, h / 2), (0, h / 2), (0, -h / 2), (b, -h / 2)], t)
    web_x = 0.0 - ch['xc']
    ch['e'] = web_x - ch['xs']
    ch['e_cl'] = 3 * b ** 2 / (h + 6 * b)
    a, tt = ASSUME['angle']['a'], ASSUME['angle']['t']
    an = thin_section_shear_centre([(a, 0), (0, 0), (0, a)], tt)
    an['dist_to_corner'] = math.hypot(an['xs'] - (0 - an['xc']), an['ys'] - (0 - an['yc']))
    an['J'] = 2 * a * tt ** 3 / 3
    ch['J'] = (h + 2 * b) * t ** 3 / 3
    return dict(channel=ch, angle=an)


def timoshenko_beam(P, L, E, b, h, nu=0.33, n=2000):
    """Cantilever, tip load.  Equations:  M = EI dθ/dx,  V = κ G A (dw/dx - θ),  M' = V.  Integrated numerically and compared with
    the closed form w = P L³/3EI + P L/(κ G A)."""
    I, A = b * h ** 3 / 12, b * h
    G = E / (2 * (1 + nu))
    kap = 10 * (1 + nu) / (12 + 11 * nu)                          # solid rectangle
    x = (np.arange(n) + 0.5) * L / n
    dx = L / n
    M = P * (L - x)
    th = np.cumsum(M / (E * I)) * dx
    dw = th + P / (kap * G * A)
    w = float(np.sum(dw) * dx)
    w_cl = P * L ** 3 / (3 * E * I) + P * L / (kap * G * A)
    w_eb = P * L ** 3 / (3 * E * I)
    return dict(w=w, w_cl=w_cl, w_eb=w_eb, err=abs(w - w_cl) / w_cl, shear_share=(w_cl - w_eb) / w_cl, kappa=kap, LH=L / h)


# ---- buckling beyond Euler: secant formula and Johnson parabola ------------------------------------------------------------
def column_regimes():
    A = ASSUME
    out = {}
    cases = {'arm link (cantilever K=2)': (A['arm_E'], A['arm_sy'], math.pi / 64 * (A['arm_OD'] ** 4 - A['arm_ID'] ** 4),
                                            math.pi / 4 * (A['arm_OD'] ** 2 - A['arm_ID'] ** 2), 0.40, 2.0, A['arm_OD'] / 2),
             'finger (pinned K=1)': (A['ph_E'], A['arm_sy'], min(A['ph_b'] * A['ph_h'] ** 3, A['ph_h'] * A['ph_b'] ** 3) / 12,
                                      A['ph_b'] * A['ph_h'], 0.143, 1.0, min(A['ph_b'], A['ph_h']) / 2)}
    for nm, (E, sy, I, Ar, L, K, c) in cases.items():
        r = math.sqrt(I / Ar)
        Le = K * L
        sl = Le / r
        Cc = math.sqrt(2 * math.pi ** 2 * E / sy)
        P_euler = math.pi ** 2 * E * I / Le ** 2
        P_john = Ar * (sy - sy ** 2 / (4 * math.pi ** 2 * E) * sl ** 2)
        e = A_ecc = ASSUME['ecc']
        def smax(P):
            return P / Ar * (1 + e * c / r ** 2 / math.cos(Le / (2 * r) * math.sqrt(P / (E * Ar))))
        lo, hi = 0.0, min(P_euler, P_john if sl < Cc else P_euler) * 0.999999
        for _ in range(80):
            mid = (lo + hi) / 2
            if smax(mid) > sy: hi = mid
            else: lo = mid
        out[nm] = dict(r=r, sl=sl, Cc=Cc, regime='inelastic (Johnson)' if sl < Cc else 'elastic (Euler)', P_euler=P_euler,
                       P_john=P_john if sl < Cc else None, P_secant=lo, e=e, sy=sy, E=E, Ar=Ar)
    return out


# ---- unit-load method, Betti-Maxwell for two load systems, statically indeterminate beam -------------------------------
def energy_extra():
    A = ASSUME
    I = math.pi / 64 * (A['arm_OD'] ** 4 - A['arm_ID'] ** 4)
    E, L = A['arm_E'], 0.40
    Ar = math.pi / 4 * (A['arm_OD'] ** 2 - A['arm_ID'] ** 2)
    w = A['arm_rho'] * Ar * G
    P = 10.0
    n = 4000
    x = (np.arange(n) + 0.5) * L / n
    dx = L / n
    M = P * (L - x) + w * (L - x) ** 2 / 2
    m = (L - x)                                                              # unit load at the tip
    d_unit = float(np.sum(M * m) * dx / (E * I))
    d_cl = P * L ** 3 / (3 * E * I) + w * L ** 4 / (8 * E * I)
    f = lambda a, xx: (lambda a_, x_: (x_ ** 2 * (3 * a_ - x_) / 6 if x_ <= a_ else a_ ** 2 * (3 * x_ - a_) / 6))(a, xx) / (E * I)
    dBA, dAB = f(L, L / 2), f(L / 2, L)                                      # deflection at L/2 due to unit tip load vs at tip due to unit load at L/2
    PA, QB = 7.0, 3.0
    betti = (PA * QB * dBA, QB * PA * dAB)
    M0 = np.where(x < L / 2, 1.0 * (L / 2 - x), 0.0)                         # propped cantilever, mid-span load P = 1, prop at the tip
    R = float(np.sum(M0 * m) / np.sum(m * m))
    Mwall = 0.5 * L - R * L
    return dict(d_unit=d_unit, d_cl=d_cl, err_unit=abs(d_unit - d_cl) / d_cl, betti=betti, err_betti=abs(betti[0] - betti[1]) / betti[0],
                R_over_P=R, Mwall=Mwall / L, chkR=abs(R - 5 / 16), chkM=abs(Mwall / L - 3 / 16))


# ---- plasticity ----------------------------------------------------------------------------------------
def plasticity(sy, Y_hertz, Es, Re, N):
    A = ASSUME
    sy_b = A['arm_sy']
    Ro, Ri = A['arm_OD'] / 2, A['arm_ID'] / 2
    I = math.pi / 4 * (Ro ** 4 - Ri ** 4)
    Zp = 4.0 / 3.0 * (Ro ** 3 - Ri ** 3)
    My, Mp = sy_b * I / Ro, sy_b * Zp
    b, h = A['ph_b'], A['ph_h']
    f_rect = (b * h ** 2 / 4) / (b * h ** 2 / 6)
    resid = sy_b - 1.5 * sy_b                                              # surface residual stress after unloading from Mp (rectangle)
    p_y = 1.60 * Y_hertz
    F_y = (p_y ** 3 * math.pi ** 3 * Re ** 2) / (6 * Es ** 2)
    L1 = L2 = 0.40
    M_self = A['arm_rho'] * math.pi * (Ro ** 2 - Ri ** 2) * G * (L1 ** 2 / 2 + L2 * (L1 + L2 / 2)) + A['hand_mass'] * G * (L1 + L2)
    return dict(My=My, Mp=Mp, shape=Mp / My, shape_rect=f_rect, resid=resid, p_y=p_y, F_y=F_y, ratio=N / F_y,
                m_yield=(My - M_self) / (L1 + L2) / G, m_collapse=(Mp - M_self) / (L1 + L2) / G, shake=2.7 * Y_hertz)


# ---- Thermal stress --------------------------------------------------------------------------------------------------------------
def _alpha(key, o):
    tab = {'steel': 12e-6, 'steel_2kg': 12e-6, 'glass': 9e-6, 'wet_glass': 9e-6, 'egg': 12e-6, 'bottle_empty': 70e-6,
           'bottle_full': 70e-6, 'apple': 60e-6, 'foam': 100e-6}
    if key in tab: return tab[key], 'table'
    lab = (o.get('label', '') or '').lower()
    for w, a in (('titan', 8.6e-6), ('alumin', 23e-6), ('steel', 12e-6), ('abs', 90e-6), ('wood', 5e-6), ('rubber', 150e-6), ('glass', 9e-6)):
        if w in lab: return a, 'label'
    E = o['E']
    return (12e-6 if E > 1.5e11 else 23e-6 if E > 5e10 else 90e-6 if E > 5e8 else 150e-6), 'estimated from E'


def thermal(key, o):
    a, src = _alpha(key, o)
    dT = ASSUME['dT']
    E, nu = o['E'], o['nu']
    L = o['dims'][1] if o.get('dims') else 0.05
    arm = ASSUME['arm_E']
    return dict(alpha=a, src=src, dT=dT, s_uni=E * a * dT, s_bi=E * a * dT / (1 - nu), dL=a * L * dT, arm_dL=23e-6 * 0.8 * dT,
                mismatch=(150e-6 - a) * dT)


# ---- Fatigue and stress concentration -----------------------------------------------------------------------------------------------
def fatigue(sig_nom):
    f = ASSUME['fatigue']
    dw = ASSUME['notch']['d_over_w']
    Kt = 3.00 - 3.14 * dw + 3.667 * dw ** 2 - 1.527 * dw ** 3                 # Peterson, plate with a central hole, net-section stress based on gross width
    Kt_net = Kt * (1 - dw)
    Su, Sinf, Ninf = f['Su'], f['S_inf'], f['N_inf']
    S1, N1 = 0.9 * Su, 1e3
    bexp = math.log(Sinf / S1) / math.log(Ninf / N1)
    sa = Kt * sig_nom / 2.0                                                     # repeated 0 -> max loading: amplitude = mean = max/2
    sm = sa
    s_eq = sa / (1 - sm / Su) if sm < Su else float('inf')
    N = N1 * (s_eq / S1) ** (1.0 / bexp) if s_eq > Sinf else float('inf')
    sig_allow_1e6 = S1 * (1e6 / N1) ** bexp
    return dict(Kt=Kt, Kt_net=Kt_net, sa=sa, s_eq=s_eq, N=N, b=bexp, goodman_fos=(1.0 / (sa / Sinf + sm / Su)),
                F_scale=(sig_allow_1e6 * (1 - sm / Su) * 2.0 / Kt) / max(sig_nom, 1e-12), cycles=f['cycles'], miner=(f['cycles'] / N if N != float('inf') else 0.0),
                S_1e6=sig_allow_1e6, Su=Su, Sinf=Sinf, S1=S1, Ninf=Ninf, N1=N1)


# ---- Contact of two curved bodies (general Hertz, elliptical contact) ----------------------------------------------------------------------
def ellipse_contact(W, R1x, R1y, R2x, R2y, E1, nu1, E2, nu2):
    inv = lambda r: 0.0 if math.isinf(r) else 1.0 / r
    cx, cy = inv(R1x) + inv(R2x), inv(R1y) + inv(R2y)
    Rx, Ry = 1.0 / cx, 1.0 / cy
    if Rx > Ry: Rx, Ry = Ry, Rx
    alpha = Ry / Rx
    k = 1.0339 * alpha ** 0.636
    Ee = 1.0003 + 0.5968 / alpha
    R = 1.0 / (cx + cy)
    Ep = 2.0 / ((1 - nu1 ** 2) / E1 + (1 - nu2 ** 2) / E2)
    a = (6 * k * k * Ee * W * R / (math.pi * Ep)) ** (1 / 3)
    b = (6 * Ee * W * R / (math.pi * k * Ep)) ** (1 / 3)
    return dict(a=a, b=b, k=k, p0=3 * W / (2 * math.pi * a * b), alpha=alpha, Rx=Rx, Ry=Ry)


# ---- Anisotropic Hooke's law -----------------------------------------------------------------------------------------------------------------
def orthotropic(Es_pad, nu_pad, N, Re, p0_iso, Es_iso):
    w = ASSUME['wood']
    EL, ET, Gm, nu = w['EL'], w['ET'], w['G'], w['nuLT']
    S = np.array([[1 / EL, -nu / EL, 0], [-nu / EL, 1 / ET, 0], [0, 0, 1 / Gm]])
    def Ex(th):
        c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
        return 1.0 / (c ** 4 / EL + (1 / Gm - 2 * nu / EL) * s * s * c * c + s ** 4 / ET)
    def Sbar(th):
        c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
        T = np.array([[c * c, s * s, 2 * s * c], [s * s, c * c, -2 * s * c], [-s * c, s * c, c * c - s * s]])
        Rr = np.diag([1, 1, 2.0])
        return Rr @ np.linalg.inv(T) @ np.linalg.inv(Rr) @ S @ T
    th = np.linspace(0, 90, 91)
    E_th = np.array([Ex(t) for t in th])
    chk = max(abs(Sbar(t)[0, 0] - 1 / Ex(t)) / (1 / Ex(t)) for t in (0, 30, 45, 60, 90))
    def p0_for(Eo):
        Es = 1.0 / ((1 - ASSUME_PAD_NU ** 2) / ASSUME_PAD_E + (1 - 0.3 ** 2) / Eo)
        a = (3 * N * Re / (4 * Es)) ** (1 / 3)
        return 1.5 * N / (math.pi * a * a)
    return dict(theta=th, E=E_th, EL=EL, ET=ET, chk=chk, S45=Sbar(45), p0_L=p0_for(EL), p0_T=p0_for(ET),
                E45=Ex(45), eta=Sbar(45)[0, 2] * Ex(45))


ASSUME_PAD_E, ASSUME_PAD_NU = 2.0e6, 0.49


# ---- Thick-walled sphere (Lame) ------------------------------------------------------------------------------------------------------------------
def thick_sphere(ro, ri, p_i, p_o=0.0):
    A = (p_i * ri ** 3 - p_o * ro ** 3) / (ro ** 3 - ri ** 3)
    B = (p_i - p_o) * ri ** 3 * ro ** 3 / (2 * (ro ** 3 - ri ** 3))
    r = np.linspace(ri, ro, 40)
    sr, st = A - 2 * B / r ** 3, A + B / r ** 3
    return dict(A=A, B=B, r=r, sr=sr, st=st, st_in=A + B / ri ** 3, st_out=A + B / ro ** 3, sr_in=A - 2 * B / ri ** 3,
                thin=p_i * (ro + ri) / 2 / (2 * (ro - ri)), chk_bc=abs((A - 2 * B / ri ** 3) + p_i) + abs(A - 2 * B / ro ** 3 + p_o))


# ---------------------------------------------------------------------------------------------------------------------------








