"""cad.py - Turn ANY CAD mesh (STL / OBJ) into a grip_gz specimen with automatic curvature & MoS analysis."""
import argparse
import json
import math
import os
import re
import struct
import numpy as np

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

def _read_stl(path):
    with open(path, 'rb') as fh: data = fh.read()
    if len(data) >= 84:
        n = struct.unpack('<I', data[80:84])[0]
        if len(data) == 84 + 50 * n and n > 0:
            arr = np.frombuffer(data, dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]), count=n, offset=84)
            return arr['v'].reshape(-1, 3).astype(np.float64), np.arange(len(arr['v'].reshape(-1, 3))).reshape(-1, 3)
    nums = np.array(re.findall(r'vertex\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)', data.decode('utf-8', errors='ignore')), dtype=np.float64)
    if not len(nums) or len(nums) % 3: raise CadError('Could not read STL file')
    return nums, np.arange(len(nums)).reshape(-1, 3)

def _read_obj(path):
    verts, faces = [], []
    with open(path, 'r', errors='ignore') as fh:
        for line in fh:
            if line.startswith('v '): verts.append([float(x) for x in line.split()[1:4]])
            elif line.startswith('f '):
                idx = [int(tok.split('/')[0]) for tok in line.split()[1:]]
                idx = [i - 1 if i > 0 else len(verts) + i for i in idx]
                for k in range(1, len(idx) - 1): faces.append([idx[0], idx[k], idx[k + 1]])
    if not verts or not faces: raise CadError('OBJ file has no geometry')
    return np.array(verts, dtype=np.float64), np.array(faces, dtype=np.int64)

def load_mesh(path):
    path = os.path.expanduser(path)
    if not os.path.isfile(path): raise CadError('File not found')
    ext = os.path.splitext(path)[1].lower()
    if ext == '.stl': return _read_stl(path)
    if ext == '.obj': return _read_obj(path)
    raise CadError('Use STL or OBJ format')

def _weld(V, F, tol=1e-7):
    _, first, inv = np.unique(np.round(V / tol).astype(np.int64), axis=0, return_index=True, return_inverse=True)
    F2 = inv[F]
    return V[first], F2[(F2[:, 0] != F2[:, 1]) & (F2[:, 1] != F2[:, 2]) & (F2[:, 0] != F2[:, 2])]

def _is_watertight(F):
    e = np.sort(np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]), axis=1)
    cnt = np.unique(e, axis=0, return_counts=True)[1]
    return bool(np.all(cnt == 2)), int(np.sum(cnt != 2))

def _volume_com(V, F):
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    vol6 = np.einsum('ij,ij->i', a, np.cross(b, c))
    vol = vol6.sum() / 6.0
    if abs(vol) < 1e-18: return 0.0, np.zeros(3)
    return vol, ((a + b + c) / 4.0 * (vol6[:, None] / 6.0)).sum(axis=0) / vol

def _rot_up(up):
    c = {'z': (0, 0), 'y': (90, 0), '-y': (-90, 0), 'x': (0, -90), '-x': (0, 90), '-z': (180, 0)}
    ax, ay = [math.radians(v) for v in c[up.lower().replace('+', '')]]
    Rx = np.array([[1, 0, 0], [0, math.cos(ax), -math.sin(ax)], [0, math.sin(ax), math.cos(ax)]])
    Ry = np.array([[math.cos(ay), 0, math.sin(ay)], [0, 1, 0], [-math.sin(ay), 0, math.cos(ay)]])
    return Ry @ Rx

def _rot_z(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])

def slice_plane(V, F, axis, val):
    tri, cross, P = V[F], np.zeros((3, len(F)), dtype=bool), np.zeros((3, len(F), 3))
    d = np.where(np.abs(tri[:, :, axis] - val) < 1e-12, 1e-12, tri[:, :, axis] - val)
    for e, (i, j) in enumerate(((0, 1), (1, 2), (2, 0))):
        cross[e] = d[:, i] * d[:, j] < 0
        P[e] = tri[:, i, :] + np.where(cross[e], d[:, i] / np.where(cross[e], d[:, i] - d[:, j], 1.0), 0.0)[:, None] * (tri[:, j, :] - tri[:, i, :])
    sel = cross.sum(axis=0) == 2
    if not sel.any(): return np.zeros((0, 2, 3))
    order = np.argsort(~cross[:, sel], axis=0, kind='stable')
    return np.stack([P[order[0], np.where(sel)[0], :], P[order[1], np.where(sel)[0], :]], axis=1)

def _fit_radius(pts2, rw):
    if len(pts2) < 5: return None
    q = pts2 - pts2.mean(axis=0)
    if np.abs(q @ np.linalg.svd(q, full_matrices=False)[2][1]).max() < max(2e-4, 0.01 * rw): return INF
    try: sol = np.linalg.lstsq(np.c_[2 * q[:, 0], 2 * q[:, 1], np.ones(len(q))], (q ** 2).sum(axis=1), rcond=None)[0]
    except np.linalg.LinAlgError: return None
    r2 = sol[2] + sol[0] ** 2 + sol[1] ** 2
    return INF if r2 <= 0 or math.sqrt(r2) > 40.0 * rw else math.sqrt(r2)

def _grasp_geometry(V, F, h, zc_abs):
    zl = min(max(zc_abs - h / 2.0, -h / 2.0 + 1e-4), h / 2.0 - 1e-4)
    seg = slice_plane(V, F, 2, zl)
    if len(seg) == 0: return None
    pts = seg.reshape(-1, 3)
    ymin, ymax = pts[:, 1].min(), pts[:, 1].max()
    res = dict(w=ymax - ymin, ymin=ymin, ymax=ymax, zl=zl, R_h=[], R_v=[])
    rw = max(0.35 * res['w'], 0.004)
    for yext in (ymax, ymin):
        near = pts[np.abs(pts[:, 1] - yext) < 1e-5]
        xe = 0.5 * (near[:, 0].min() + near[:, 0].max())
        res['R_h'].append(INF if (near[:, 0].max() - near[:, 0].min()) >= 0.5 * rw else _fit_radius(pts[np.linalg.norm(pts[:, :2] - np.array([xe, yext]), axis=1) < rw][:, :2], rw))
        seg2 = slice_plane(V, F, 0, xe)
        Rv = None
        if len(seg2):
            near2 = seg2.reshape(-1, 3)[:, 1:3][np.abs(seg2.reshape(-1, 3)[:, 1] - yext) < 1e-5]
            Rv = INF if len(near2) >= 2 and (near2[:, 1].max() - near2[:, 1].min()) >= 0.5 * rw else _fit_radius(seg2.reshape(-1, 3)[:, 1:3][np.linalg.norm(seg2.reshape(-1, 3)[:, 1:3] - np.array([yext, zl]), axis=1) < rw], rw)
        res['R_v'].append(Rv)
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

def make_object(spec):
    s = dict(spec)
    path = os.path.expanduser(str(s.get('file') or '').strip().strip('"').strip("'"))
    if not path: raise CadError('No CAD file given.')
    warn, unit = [], str(s.get('unit') or 'mm').lower()
    if unit not in UNITS: raise CadError('Unknown unit %r. Use one of: %s' % (unit, ', '.join(UNITS)))
    mname = str(s.get('material') or 'aluminium').lower()
    if mname not in MATERIALS: raise CadError('Unknown material %r. Use one of: %s' % (mname, ', '.join(MATERIALS)))
    mat = dict(MATERIALS[mname])

    V, F = load_mesh(path)
    V, F = _weld(V * UNITS[unit], F)
    V = V @ _rot_up(str(s.get('up') or 'z')).T @ _rot_z(float(s.get('rotz') or 0.0)).T
    wt, nbad = _is_watertight(F)
    if not wt: warn.append('Mesh is not watertight (%d open edges).' % nbad)
    vol, com = _volume_com(V, F)
    if vol < 0: F, vol = F[:, [0, 2, 1]], -vol
    if vol < 1e-12: raise CadError('Mesh volume is ~0.')

    lo, hi = V.min(axis=0), V.max(axis=0)
    V, com, ext, h = V - 0.5 * (lo + hi), com - 0.5 * (lo + hi), hi - lo, float(hi[2] - lo[2])
    com_rel, zg = float(com[2] + h / 2.0), min(max(float(com[2] + h / 2.0), 0.032), max(h - 0.002, 0.032))
    zc = max(0.0, min(h, 0.5 * (max(0.0, zg - GRIPPER['pad_h'] / 2.0) + min(h, zg + GRIPPER['pad_h'] / 2.0))))
    
    gg = _grasp_geometry(V, F, h, zc)
    if gg and gg['w'] > 2.0 * GRIPPER['open_half_gap'] - 0.004:
        gg2 = _grasp_geometry(V @ _rot_z(90).T, F, h, zc)
        if gg2 and gg2['w'] < gg['w'] and gg2['w'] <= 2.0 * GRIPPER['open_half_gap'] - 0.004:
            V, com, gg = V @ _rot_z(90).T, _rot_z(90) @ com, gg2
            warn.append('Turned 90 deg about Z to fit.')
    if not gg: raise CadError('Could not slice mesh.')
    if gg['w'] > 2.0 * GRIPPER['open_half_gap'] - 0.004: raise CadError('Part too wide.')
    if gg['w'] < 0.004: raise CadError('Part too narrow.')

    y_mid = 0.5 * (gg['ymin'] + gg['ymax'])
    if max(abs(y_mid), abs(com[1])) > 0.05 * gg['w']: warn.append('Part is not symmetric about Y.')
    if abs(com[0]) > 0.10 * ext[0]: warn.append('Center of mass is off-axis in X.')

    Rs = [r for r in (_combine_radius(gg['R_h'][k], gg['R_v'][k]) for k in (0,1)) if r is not None]
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
    _write_stl(os.path.join(CAD_DIR, key + '.stl'), V, F)

    o = dict(label='CAD: ' + key[4:], shape='mesh', dims=(gg['w'] / 2.0, h), vis='mesh', m=mass, mu=ov('mu', mat['mu']), E=ov('E', mat['E']), nu=ov('nu', mat['nu']), R=R_obj, p_allow=p_allow, F_allow=F_allow, mode=KIND_INFO[kind][0], rgba=tuple(s.get('color') or mat['rgba']), mesh=os.path.join(CAD_DIR, key + '.stl'), grasp_z=zg, cad=True)
    rep = dict(key=key, name=key[4:], file=path, unit=unit, material=str(s.get('material') or 'aluminium').lower(), kind=kind, volume=vol, mass=mass, density=mass / vol, extents=tuple(float(x) for x in ext), height=h, width=gg['w'], grasp_z=zg, com=tuple(float(x) for x in com), R_contact=R_obj, mu=ov('mu', mat['mu']), E=ov('E', mat['E']), nu=ov('nu', mat['nu']), p_allow=p_allow, F_allow=F_allow, strength=ov('strength', mat['strength']), watertight=wt, faces=int(len(F)), mesh_out=os.path.join(CAD_DIR, key + '.stl'), warnings=warn)
    return key, o, rep

def register(key, o): OBJECTS[key] = o
def _f(x, u='', s=1.0): return 'inf' if math.isinf(x) else ('%.4g%s' % (x * s, u))
def format_report(rep):
    lines = ['CAD part: %s (%s, %d faces)' % (rep['name'], rep['material'], rep['faces']), 'Size: %.1f x %.1f x %.1f mm' % tuple(1000 * v for v in rep['extents']), 'Mass: %.1f g (density %.0f kg/m3)' % (rep['mass'] * 1000, rep['density']), 'Grasp width: %.1f mm at height %.1f mm' % (rep['width'] * 1000, rep['grasp_z'] * 1000), 'Contact R: %s mm' % _f(rep['R_contact'], '', 1000.0), 'Limits: p_allow %s Pa, F_allow %s N' % (_f(rep['p_allow']), _f(rep['F_allow']))]
    for p in ('small', 'large'):
        t = truth(rep['key'], p)
        lines.append('  %-5s pad: N_slip %.2f N, N_max %.2f N -> %s' % (p, t['N_slip'], t['N_max'], 'GRIPPABLE' if t['N_slip'] < t['N_max'] else 'NO SAFE GRIP'))
    if rep['warnings']: lines.extend(['', *(f'WARNING: {w}' for w in rep['warnings'])])
    return '\n'.join(lines)
def spec_from_json(path):
    with open(os.path.expanduser(path)) as fh: d = json.load(fh)
    if 'file' not in d and 'cad' in d: d['file'] = d['cad']
    return d
def build_parser():
    ap = argparse.ArgumentParser(description='Analyse a CAD mesh (STL/OBJ) as a grip_gz specimen. SI units: E, strength, p_allow in Pa; mass in kg; density in kg/m3; F_allow in N; R_contact in m.')
    ap.add_argument('file', nargs='?'); ap.add_argument('--spec'); ap.add_argument('--unit', default='mm'); ap.add_argument('--material', default='aluminium')
    ap.add_argument('--mass', type=float); ap.add_argument('--density', type=float); ap.add_argument('--up', default='z'); ap.add_argument('--rotz', type=float, default=0.0)
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
