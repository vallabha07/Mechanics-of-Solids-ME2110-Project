"""cad.py - Turn ANY CAD mesh (STL / OBJ) into a grip_gz specimen with automatic curvature & MoS analysis."""
import argparse
import json
import math
import os
import re
import struct
import numpy as np

from . import meshprep as MP
from .materials import GRIPPER, INF, OBJECTS, PADS, truth

UNITS = {'m': 1.0, 'cm': 0.01, 'mm': 0.001, 'in': 0.0254, 'inch': 0.0254, 'ft': 0.3048}

MATERIALS = {
    'steel':         dict(rho=7850, mu=0.60, E=200e9, nu=0.30, strength=250e6, kind='ductile',  rgba=(0.62, 0.64, 0.68, 1)),
    'stainless':     dict(rho=8000, mu=0.50, E=193e9, nu=0.30, strength=215e6, kind='ductile',  rgba=(0.75, 0.77, 0.80, 1)),
    'aluminium':     dict(rho=2700, mu=0.50, E=69e9,  nu=0.33, strength=240e6, kind='ductile',  rgba=(0.78, 0.80, 0.83, 1)),
    'brass':         dict(rho=8500, mu=0.40, E=100e9, nu=0.34, strength=200e6, kind='ductile',  rgba=(0.80, 0.65, 0.25, 1)),
    'titanium':      dict(rho=4500, mu=0.40, E=110e9, nu=0.34, strength=880e6, kind='ductile',  rgba=(0.60, 0.62, 0.66, 1)),
    'abs':           dict(rho=1050, mu=0.50, E=2.3e9, nu=0.35, strength=40e6,  kind='ductile',  rgba=(0.90, 0.90, 0.85, 1)),
    'pla':           dict(rho=1250, mu=0.45, E=3.5e9, nu=0.36, strength=50e6,  kind='brittle',  rgba=(0.30, 0.75, 0.45, 1)),
    'nylon':         dict(rho=1150, mu=0.35, E=2.7e9, nu=0.39, strength=70e6,  kind='ductile',  rgba=(0.92, 0.92, 0.88, 1)),
    'polycarbonate': dict(rho=1200, mu=0.40, E=2.4e9, nu=0.37, strength=62e6,  kind='ductile',  rgba=(0.80, 0.90, 0.95, 1)),
    'glass':         dict(rho=2500, mu=0.60, E=70e9,  nu=0.22, strength=40e6,  kind='brittle',  rgba=(0.60, 0.85, 0.95, 1)),
    'ceramic':       dict(rho=3900, mu=0.50, E=300e9, nu=0.22, strength=250e6, kind='brittle',  rgba=(0.95, 0.95, 0.92, 1)),
    'wood':          dict(rho=600,  mu=0.50, E=10e9,  nu=0.30, strength=6e6,   kind='ductile',  rgba=(0.65, 0.48, 0.30, 1)),
    'rubber':        dict(rho=1100, mu=1.00, E=10e6,  nu=0.49, strength=15e6,  kind='ductile',  rgba=(0.15, 0.15, 0.15, 1)),
    'foam':          dict(rho=40,   mu=0.80, E=50e3,  nu=0.30, strength=12e3,  kind='cellular', rgba=(0.95, 0.85, 0.20, 1)),
    'fruit':         dict(rho=800,  mu=0.70, E=5e6,   nu=0.35, strength=0.19e6, kind='cellular', rgba=(0.85, 0.15, 0.15, 1)),
    'thin_shell':    dict(rho=900,  mu=0.50, E=2e9,   nu=0.40, strength=30e6,  kind='shell',    rgba=(0.30, 0.70, 0.90, 1)),
}
KIND_INFO = {'ductile': ('Von Mises (Ductile Yield)', 1.5), 'cellular': ('Von Mises (Cellular Crush)', 1.5), 'brittle': ('Rankine (Brittle Fracture)', 5.0), 'shell': ('Thin-Shell Force Limit', 1.0)}
CAD_DIR = os.path.expanduser('~/.grip_gz_cad')

class CadError(Exception): pass

def load_mesh(path):
    """Raw triangle soup (V, F) of an STL / OBJ (/ PLY, OFF, 3MF, GLB with trimesh). Raises CadError."""
    try: return MP.read_mesh(path)
    except MP.MeshError as e: raise CadError(str(e))

slice_plane = MP.slice_segments

def _rot_up(up):
    c = {'z': (0, 0), 'y': (90, 0), '-y': (-90, 0), 'x': (0, -90), '-x': (0, 90), '-z': (180, 0)}
    ax, ay = [math.radians(v) for v in c[up.lower().replace('+', '')]]
    Rx = np.array([[1, 0, 0], [0, math.cos(ax), -math.sin(ax)], [0, math.sin(ax), math.cos(ax)]])
    Ry = np.array([[math.cos(ay), 0, math.sin(ay)], [0, 1, 0], [-math.sin(ay), 0, math.cos(ay)]])
    return Ry @ Rx

def _rot_z(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])

def _fit_radius(pts2, rw):
    if len(pts2) < 5: return None
    q = pts2 - pts2.mean(axis=0)
    if np.abs(q @ np.linalg.svd(q, full_matrices=False)[2][1]).max() < max(2e-4, 0.01 * rw): return INF
    try: sol = np.linalg.lstsq(np.c_[2 * q[:, 0], 2 * q[:, 1], np.ones(len(q))], (q ** 2).sum(axis=1), rcond=None)[0]
    except np.linalg.LinAlgError: return None
    r2 = sol[2] + sol[0] ** 2 + sol[1] ** 2
    return INF if r2 <= 0 or math.sqrt(r2) > 40.0 * rw else math.sqrt(r2)

def _grasp_geometry(V, F, poly, zl):
    """Width across y and local contact radii at both y-extremes of the grasp-height section `poly` ((n,2) closed loop).
    Radii are fitted to the outline (horizontal) and to the vertical profile (plane x = x of the extreme point); failures give None."""
    ymin, ymax = float(poly[:, 1].min()), float(poly[:, 1].max())
    res = dict(w=ymax - ymin, ymin=ymin, ymax=ymax, zl=zl, R_h=[], R_v=[])
    rw = max(0.35 * res['w'], 0.004)
    for yext in (ymax, ymin):
        try:
            near = poly[np.abs(poly[:, 1] - yext) < 0.05 * rw]
            xe = float(near[:, 0].mean())
            Rh = _fit_radius(poly[np.linalg.norm(poly - np.array([xe, yext]), axis=1) < rw], rw)
            seg = MP.slice_segments(V, F, 0, xe)
            Rv = None
            if len(seg):
                pt = seg.reshape(-1, 3)[:, 1:3]                              # (y, z)
                pt = pt[np.abs(pt[:, 1] - zl) < rw]
                if len(pt) >= 5:
                    zb = np.linspace(pt[:, 1].min(), pt[:, 1].max(), 14)
                    idx = np.clip(np.digitize(pt[:, 1], zb) - 1, 0, 12)
                    prof = np.array([(pt[idx == i][:, 0].max() if yext > 0 or yext == ymax and abs(ymax) >= abs(ymin) else pt[idx == i][:, 0].min(), pt[idx == i][:, 1].mean()) for i in range(13) if (idx == i).any()])
                    Rv = _fit_radius(prof, rw) if len(prof) >= 5 else None
        except Exception:
            Rh, Rv = None, None
        res['R_h'].append(Rh); res['R_v'].append(Rv)
    return res

def _combine_radius(Rh, Rv):
    vals = [r for r in (Rh, Rv) if r is not None]
    fin = [r for r in vals if not math.isinf(r)]
    return INF if not fin else (math.sqrt(fin[0] * fin[1]) if len(fin) == 2 else fin[0]) if vals else None

def _write_stl(path, V, F):
    tri = V[F].astype('<f4')
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    nl = np.linalg.norm(n, axis=1, keepdims=True)
    with open(path, 'wb') as fh:
        fh.write(b'grip_gz normalised part'.ljust(80, b' '))
        fh.write(struct.pack('<I', len(F)))
        rec = np.zeros(len(F), dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
        rec['n'], rec['v'] = (n / np.where(nl == 0, 1, nl)).astype('<f4'), tri
        fh.write(rec.tobytes())

def _truthy(x, default=True):
    if x is None or x == '': return default
    return str(x).strip().lower() not in ('0', 'false', 'no', 'off', 'n')

def _levels_K(h): return int(min(80, max(10, h / 0.0015)))

def make_object(spec):
    s = dict(spec)
    path = os.path.expanduser(str(s.get('file') or '').strip().strip('"').strip("'"))
    if not path: raise CadError('No CAD file given.')
    warn, unit = [], str(s.get('unit') or 'mm').lower()
    if unit not in UNITS: raise CadError('Unknown unit %r. Use one of: %s' % (unit, ', '.join(UNITS)))
    mname = str(s.get('material') or 'aluminium').lower()
    if mname not in MATERIALS: raise CadError('Unknown material %r. Use one of: %s' % (mname, ', '.join(MATERIALS)))
    mat = dict(MATERIALS[mname])
    fit = _truthy(s.get('fit'), True)
    notes = []

    # ---- 1. read + clean + bring under the MuJoCo face limit ------------------------------------------------------------------------------
    V, F = load_mesh(path)
    V = V * (UNITS[unit] * float(s.get('scale') or 1.0))
    try: V, F, info = MP.clean_mesh(V, F)
    except MP.MeshError as e: raise CadError(str(e))
    if info['bad']: warn.append('Dropped %d NaN / zero-area triangles.' % info['bad'])
    if info['debris']: warn.append('Removed %d loose fragments (specks / slivers) that were not part of the main body.' % info['debris'])
    nraw = int(len(F))
    V, F = MP.decimate(V, F, MP.WORK_FACES)
    if len(F) < nraw: notes.append('Mesh simplified from %d to %d triangles (MuJoCo accepts at most %d per mesh).' % (nraw, len(F), MP.MJ_MAX_FACES))
    F, nflip = MP.orient_consistent(F)
    F, nout = MP.make_outward(V, F)
    if nflip or nout: warn.append('Repaired inverted / inconsistent triangle winding (%d faces).' % (nflip + nout))
    wt, nbad = MP.is_closed(F)
    ext_file = V.max(axis=0) - V.min(axis=0)                                 # size as stored in the file (before any re-orientation)

    # ---- 2. orientation: up axis, then turn the narrowest width onto the finger axis (y) -------------------------------------------------
    up = str(s.get('up') or 'auto').lower().replace('+', '')
    if up == 'auto':
        up, why = MP.detect_up(V, F)
        notes.append('Orientation (auto): ' + why + ('' if up == 'z' else '.  Override with --up z|y|-y|x|-x|-z.'))
    elif up not in ('z', 'y', '-y', 'x', '-x', '-z'): raise CadError('Unknown up axis %r (use auto, z, y, -y, x, -x, -z).' % up)
    else: notes.append('Orientation: %s axis up (%s).' % (up, 'automatic pose search' if s.get('_pose_search') else 'as requested'))
    V = V @ _rot_up(up).T
    rz = s.get('rotz')
    if rz in (None, '', 'auto'):
        wmin, ang, wmax = MP.footprint_width(V[:, :2])
        if wmax > 1.03 * wmin:
            rz = 90.0 - ang
            notes.append('Turned %.0f deg about Z so the narrowest width (%.1f mm) lies across the fingers.' % (rz, wmin * 1000))
        else: rz = 0.0
    V = V @ _rot_z(float(rz)).T

    # ---- 3. size sanity (unit mistakes are the commonest reason a random download fails) -----------------------------------------------
    lo, hi = V.min(axis=0), V.max(axis=0)
    ext0 = hi - lo
    opening = 2.0 * GRIPPER['open_half_gap']
    big, kfit, ktot = float(ext0.max()), 1.0, 1.0
    MAXDIM, FPMAX = 0.22, 0.90 * opening                                # tallest part / largest footprint (when BOTH footprint sides are big) the hand envelope is sized for
    fp = sorted(float(x) for x in ext0[:2])
    if fit and big < 0.012: kfit = 0.040 / big
    elif fit: kfit = min(1.0, MAXDIM / float(ext0[2]), FPMAX / fp[1] if fp[0] > FPMAX else 1.0)
    if kfit > 1.0:
        warn.append('Part was only %.2f mm in size - auto-scaled x%.3g to %.0f mm. If that is wrong, pick the right UNIT (m / cm / inch) or pass --scale.' % (big * 1000, kfit, big * kfit * 1000))
    elif kfit < 1.0: warn.append('Part is bigger than the hand envelope (%.0f x %.0f x %.0f mm): scaled down so it fits the hand.' % tuple(1000 * ext0))
    elif big < 0.012: warn.append('Largest dimension is only %.1f mm - if the file is in another unit (cm / inch / m) choose that unit.' % (big * 1000))
    elif big > 0.45: warn.append('Largest dimension is %.0f mm - check the unit (the file may be in mm while %s is selected).' % (big * 1000, unit))
    V = V * kfit; ktot *= kfit

    # ---- 4. solid cross-sections (leak-proof: works for open, hollow, self-overlapping meshes) -----------------------------------------
    for attempt in range(2):
        lo, hi = V.min(axis=0), V.max(axis=0)
        ctr0 = np.array([0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]), 0.5 * (lo[2] + hi[2])])
        V = V - ctr0
        lo, hi = V.min(axis=0), V.max(axis=0)
        h = float(hi[2] - lo[2])
        K = _levels_K(h)
        sec = MP.build_sections(V, F, K)
        if not sec['area'].sum(): raise CadError('Could not slice mesh (no solid cross-section found).')
        zg0 = h / 2.0
        dz = float(sec['zs'][1] - sec['zs'][0])
        vol_s = float(sec['area'].sum() * dz) if K > 1 else 0.0
        com_s = np.array([(sec['area'] * sec['cen'][:, 0]).sum(), (sec['area'] * sec['cen'][:, 1]).sum(), (sec['area'] * sec['zs']).sum()]) / sec['area'].sum()
        if wt:
            vol, com = MP.signed_volume_com(V, F)
            if vol < 0: F, vol = F[:, [0, 2, 1]], -vol
            if vol < 1e-12 or not (0.3 * vol_s < vol < 3.0 * vol_s): vol, com = vol_s, com_s       # inside-out / broken closed mesh: trust the sections
        else:
            vol, com = vol_s, com_s
        com_rel = float(com[2] + h / 2.0)
        zg = min(max(com_rel, 0.4 * h), 0.75 * h)
        zc = max(0.0, min(h, 0.5 * (max(0.0, zg - GRIPPER['pad_h'] / 2.0) + min(h, zg + GRIPPER['pad_h'] / 2.0))))
        k = int(np.argmin(np.abs(sec['zs'] - (zc - h / 2.0))))
        while k > 0 and not sec['polys'][k]: k -= 1
        while k < K - 1 and not sec['polys'][k]: k += 1
        if not sec['polys'][k]: raise CadError('Could not find a cross-section at grasp height.')
        poly = sec['polys'][k][0]
        wmin = MP.footprint_width(poly)[0]                                          # narrowest width of the main body at grasp height
        if attempt == 0 and fit and wmin > 0.70 * opening:
            kf = 0.55 * opening / wmin
            V = V * kf; ktot *= kf
            warn.append('Part is %.0f mm across at grasp height but the hand opens only %.0f mm - scaled down further by x%.3g.' % (wmin * 1000, opening * 1000, kf))
            continue
        break
    if wmin > opening - 0.004: raise CadError('Part too wide: narrowest width %.0f mm exceeds the hand opening (%.0f mm).' % (wmin * 1000, opening * 1000))
    if wmin < 0.0015: raise CadError('Part too thin at grasp height (%.1f mm): the fingertips cannot pinch it. Try another UP axis, a larger scale, or a different part.' % (wmin * 1000))
    if not wt: warn.append('Mesh is not watertight (%d open edges): treated as a SOLID using its filled cross-sections (volume %.0f cm3, mass from that).' % (nbad, vol * 1e6))
    if abs(com[2] + h / 2.0 - zc) > 0.5 * h: warn.append('Centre of mass is far from the grasp height.')
    warn = notes + warn

    # ---- 5. hand axis through the centre of mass, grasp geometry, material numbers ---------------------------------------------------------
    ctr = np.array([com[0], com[1], 0.0])
    def _inside(pt, P):
        x, y = pt; xs, ys = P[:, 0], P[:, 1]; x2, y2 = np.roll(xs, -1), np.roll(ys, -1)
        c = ((ys > y) != (y2 > y)) & (x < xs + (y - ys) * (x2 - xs) / np.where(y2 == ys, 1e-18, y2 - ys))
        return bool(c.sum() % 2)
    if not any(_inside(com[:2], P) for P in sec['polys'][k]):          # chairs, frames, rings ...: the centre of mass floats in air at grasp height
        A2 = 0.5 * float(np.dot(poly[:, 0], np.roll(poly[:, 1], -1)) - np.dot(poly[:, 1], np.roll(poly[:, 0], -1)))
        x, y, x2, y2 = poly[:, 0], poly[:, 1], np.roll(poly[:, 0], -1), np.roll(poly[:, 1], -1)
        cr = x * y2 - x2 * y
        ctr = np.array([((x + x2) * cr).sum() / (6 * A2), ((y + y2) * cr).sum() / (6 * A2), 0.0])
        warn.append('Centre of mass lies outside the material at grasp height, so the hand is placed on the main body there and the grasp is balanced for the off-axis mass.')
    V, com = V - ctr, com - ctr
    poly = poly - ctr[:2]
    ext = V.max(axis=0) - V.min(axis=0)
    if abs(ktot - 1.0) > 1e-6:
        warn.insert(0, 'SCALE: %s - the part is simulated at %.4g x its original size (%.1f x %.1f x %.1f mm -> %.1f x %.1f x %.1f mm).' % (('dimensions of the STL were REDUCED %.3g times' % (1.0 / ktot)) if ktot < 1 else ('dimensions of the STL were ENLARGED %.3g times' % ktot), ktot, *(1000 * ext_file), *(1000 * ext)))
    gg = _grasp_geometry(V, F, poly, float(sec['zs'][k] - ctr[2]))
    Rs = [r for r in (_combine_radius(gg['R_h'][i], gg['R_v'][i]) for i in (0, 1)) if r is not None]
    if s.get('R_contact') not in (None, ''): R_obj = float(s['R_contact'])
    elif not Rs: R_obj, _ = INF, warn.append('Flat contact assumed.')
    else: R_obj = max(0.0015, min(Rs))

    def ov(k, d): return d if s.get(k) in (None, '') else float(s[k])
    rho = ov('density', mat['rho'])
    mass = float(s['mass']) if s.get('mass') not in (None, '', 0) else rho * vol
    kind = str(s.get('kind') or mat['kind']).lower()
    if kind not in KIND_INFO: raise CadError('Unknown failure mode %r. Use one of: %s' % (kind, ', '.join(KIND_INFO)))

    p_allow = ov('p_allow', ov('strength', mat['strength']) / (0.62 * ov('safety', KIND_INFO[kind][1])) if kind in ('ductile', 'cellular') else ov('strength', mat['strength']) / (max((1.0 - 2.0 * ov('nu', mat['nu'])) / 3.0, 0.05) * ov('safety', KIND_INFO[kind][1])) if kind == 'brittle' else INF)
    F_allow = ov('F_allow', 10.0 if kind == 'shell' else INF)

    key = 'cad_' + re.sub(r'[^A-Za-z0-9_]+', '_', str(s.get('name') or os.path.splitext(os.path.basename(path))[0])).strip('_')
    os.makedirs(CAD_DIR, exist_ok=True)
    V = V - np.array([0.0, 0.0, 0.0])
    _write_stl(os.path.join(CAD_DIR, key + '.stl'), V, F)
    zmid = 0.5 * float(V[:, 2].max() + V[:, 2].min())
    com_rel = float(com[2] - zmid + h / 2.0)

    o = dict(label='CAD: ' + key[4:], shape='mesh', dims=(gg['w'] / 2.0, h), vis='mesh', m=mass, mu=ov('mu', mat['mu']), E=ov('E', mat['E']), nu=ov('nu', mat['nu']), R=R_obj, p_allow=p_allow, F_allow=F_allow, mode=KIND_INFO[kind][0], rgba=tuple(s.get('color') or mat['rgba']), mesh=os.path.join(CAD_DIR, key + '.stl'), grasp_z=zg, com=(float(com[0]), float(com[1]), com_rel), cad=True)
    rep = dict(key=key, name=key[4:], file=path, unit=unit, material=str(s.get('material') or 'aluminium').lower(), kind=kind, volume=vol, mass=mass, density=mass / vol, extents=tuple(float(x) for x in ext), height=h, width=gg['w'], grasp_z=zg, com=tuple(float(x) for x in com), R_contact=R_obj, mu=ov('mu', mat['mu']), E=ov('E', mat['E']), nu=ov('nu', mat['nu']), p_allow=p_allow, F_allow=F_allow, strength=ov('strength', mat['strength']), watertight=wt, faces=int(len(F)), mesh_out=os.path.join(CAD_DIR, key + '.stl'), warnings=warn, up=up, scale=ktot, orig_extents=tuple(float(x) for x in ext_file))
    return key, o, rep

def register(key, o): OBJECTS[key] = o
def _f(x, u='', s=1.0): return 'inf' if math.isinf(x) else ('%.4g%s' % (x * s, u))
def format_report(rep):
    lines = ['CAD part: %s (%s, %d faces)' % (rep['name'], rep['material'], rep['faces']), 'Size: %.1f x %.1f x %.1f mm' % tuple(1000 * v for v in rep['extents']), 'Mass: %.1f g (density %.0f kg/m3)' % (rep['mass'] * 1000, rep['density']), 'Grasp width: %.1f mm at height %.1f mm' % (rep['width'] * 1000, rep['grasp_z'] * 1000), 'Contact R: %s mm' % _f(rep['R_contact'], '', 1000.0), 'Limits: p_allow %s Pa, F_allow %s N' % (_f(rep['p_allow']), _f(rep['F_allow']))]
    for p in ('small', 'large'):
        t = truth(rep['key'], p)
        lines.append('  %-5s pad: N_slip %.2f N, N_max %.2f N -> %s' % (p, t['N_slip'], t['N_max'], 'GRIPPABLE' if t['N_slip'] < t['N_max'] else 'NO SAFE GRIP'))
    if abs(rep.get('scale', 1.0) - 1.0) > 1e-6:
        k = rep['scale']
        lines.insert(2, '*** SCALED: STL dimensions %s %.3g times (original %.1f x %.1f x %.1f mm) ***' % ('reduced' if k < 1 else 'enlarged', (1.0 / k) if k < 1 else k, *(1000 * v for v in rep['orig_extents'])))
    if rep.get('plan_text'): lines.extend(['', rep['plan_text']])
    if rep['warnings']: lines.extend(['', *(f'WARNING: {w}' for w in rep['warnings'])])
    return '\n'.join(lines)
def spec_from_json(path):
    with open(os.path.expanduser(path)) as fh: d = json.load(fh)
    if 'file' not in d and 'cad' in d: d['file'] = d['cad']
    return d
def build_parser():
    ap = argparse.ArgumentParser(description='Analyse a CAD mesh (STL/OBJ) as a grip_gz specimen. SI units: E, strength, p_allow in Pa; mass in kg; density in kg/m3; F_allow in N; R_contact in m.')
    ap.add_argument('file', nargs='?'); ap.add_argument('--spec'); ap.add_argument('--unit', default='mm'); ap.add_argument('--material', default='aluminium')
    ap.add_argument('--mass', type=float); ap.add_argument('--density', type=float); ap.add_argument('--up', default='auto', help='auto (default) | z | y | -y | x | -x | -z'); ap.add_argument('--rotz', type=float, default=None, help='degrees about Z; default = auto-align narrowest width across the fingers')
    ap.add_argument('--scale', type=float, help='extra uniform scale on top of the unit'); ap.add_argument('--no-fit', dest='fit', action='store_false', default=True, help='do not auto-scale parts that are too big / small for the hand')
    ap.add_argument('--mu', type=float); ap.add_argument('--E', type=float); ap.add_argument('--nu', type=float); ap.add_argument('--strength', type=float)
    ap.add_argument('--F-allow', dest='F_allow', type=float); ap.add_argument('--R-contact', dest='R_contact', type=float); ap.add_argument('--kind'); ap.add_argument('--name')
    return ap

def spec_from_args(argv):
    """['part.stl', '--material', 'foam', '--mu', '0.7'] -> spec dict for make_object()."""
    a = build_parser().parse_args(list(argv))
    spec = {k: v for k, v in vars(a).items() if v is not None and k != 'spec'}
    if a.spec: spec.update(spec_from_json(a.spec))
    return spec

def main(argv=None):
    try: spec = spec_from_args(argv if argv is not None else __import__('sys').argv[1:])
    except SystemExit as e: return int(e.code or 0)
    try: key, o, rep = make_object(spec)
    except CadError as e: return print('ERROR:', e) or 1
    register(key, o)
    print(format_report(rep))
    return 0

if __name__ == '__main__': raise SystemExit(main())
