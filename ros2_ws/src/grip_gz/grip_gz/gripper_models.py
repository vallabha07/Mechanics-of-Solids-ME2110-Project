"""gripper_models.py - Procedural multi-material STL mesh generator for the arm and n-finger hand."""
import math
import os
import struct
import numpy as np

from . import hand as H
from .arm_ik import ARM

MESH_DIR = os.path.expanduser('~/.grip_gz_cad/gripper')
COLORS = {'steel': (0.62, 0.65, 0.70, 1.0), 'dark': (0.20, 0.21, 0.25, 1.0), 'hub': (0.36, 0.38, 0.43, 1.0), 'light': (0.80, 0.82, 0.85, 1.0), 'accent': (0.95, 0.50, 0.10, 1.0), 'rubber': (0.07, 0.07, 0.08, 1.0), 'bolt': (0.85, 0.86, 0.88, 1.0), 'black': (0.05, 0.05, 0.06, 1.0), 'white': (0.96, 0.96, 0.94, 1.0), 'charcoal': (0.15, 0.16, 0.18, 1.0)}

def _cat(parts):
    Vs, Fs, off = [], [], 0
    for V, F in parts: Vs.append(V); Fs.append(F + off); off += len(V)
    return np.vstack(Vs), np.vstack(Fs)

def box(cx, cy, cz, sx, sy, sz):
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    V = np.array([[cx + a * hx, cy + b * hy, cz + c * hz] for c in (-1, 1) for b in (-1, 1) for a in (-1, 1)], dtype=np.float64)
    F = np.array([(0, 2, 1), (1, 2, 3), (4, 5, 6), (5, 7, 6), (0, 1, 4), (1, 5, 4), (2, 6, 3), (3, 6, 7), (0, 4, 2), (2, 4, 6), (1, 3, 5), (3, 7, 5)], dtype=np.int64)
    return V, F

def cyl(cx, cy, cz, r, length, axis='y', n=40):
    ang = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    V = np.vstack([np.c_[r * np.cos(ang), r * np.sin(ang), np.full(n, -length / 2.0)], np.c_[r * np.cos(ang), r * np.sin(ang), np.full(n, length / 2.0)], [[0, 0, -length / 2.0]], [[0, 0, length / 2.0]]])
    F = np.array([(i, (i + 1) % n, n + (i + 1) % n) for i in range(n)] + [(i, n + (i + 1) % n, n + i) for i in range(n)] + [(2 * n + 1, n + i, n + (i + 1) % n) for i in range(n)] + [(2 * n, (i + 1) % n, i) for i in range(n)], dtype=np.int64)
    if axis == 'y': V = np.c_[V[:, 0], V[:, 2], -V[:, 1]]
    elif axis == 'x': V = np.c_[V[:, 2], V[:, 1], -V[:, 0]]
    return V + np.array([cx, cy, cz]), F

def ellipsoid(cx, cy, cz, rx, ry, rz, nu=28, nv=16):
    V = [[0, 0, 1]] + [[math.sin(math.pi * i / nv) * math.cos(2.0 * math.pi * j / nu), math.sin(math.pi * i / nv) * math.sin(2.0 * math.pi * j / nu), math.cos(math.pi * i / nv)] for i in range(1, nv) for j in range(nu)] + [[0, 0, -1]]
    F = [(0, 1 + j, 1 + (j + 1) % nu) for j in range(nu)] + [(1 + i * nu + j, 1 + (i + 1) * nu + j, 1 + i * nu + (j + 1) % nu) for i in range(nv - 2) for j in range(nu)] + [(1 + i * nu + (j + 1) % nu, 1 + (i + 1) * nu + j, 1 + (i + 1) * nu + (j + 1) % nu) for i in range(nv - 2) for j in range(nu)] + [(len(V) - 1, 1 + (nv - 2) * nu + (j + 1) % nu, 1 + (nv - 2) * nu + j) for j in range(nu)]
    return np.array(V) * np.array([rx, ry, rz]) + np.array([cx, cy, cz]), np.array(F, dtype=np.int64)

def rotz(V, F, phi): return np.c_[math.cos(phi) * V[:, 0] - math.sin(phi) * V[:, 1], math.sin(phi) * V[:, 0] + math.cos(phi) * V[:, 1], V[:, 2]], F

def screws_y(cx, cz, ring_r, y_face, count=5, r=0.0016, h=0.0015):
    return _cat([cyl(cx + ring_r * math.cos(2.0 * math.pi * k / count), y_face, cz + ring_r * math.sin(2.0 * math.pi * k / count), r, h, 'y', 10) for k in range(count)])

def write_stl(path, V, F):
    tri = V[F].astype('<f4')
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    nl = np.linalg.norm(n, axis=1, keepdims=True)
    with open(path, 'wb') as fh:
        fh.write(b'grip_gz gripper part'.ljust(80, b' '))
        fh.write(struct.pack('<I', len(F)))
        rec = np.zeros(len(F), dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
        rec['n'], rec['v'] = (n / np.where(nl == 0, 1, nl)).astype('<f4'), tri
        fh.write(rec.tobytes())

def part_base(base_h):
    return {'charcoal': _cat([_cat([box(0, 0, 0.010, 0.22, 0.22, 0.020)] + [cyl(sx * 0.09, sy * 0.09, 0.023, 0.008, 0.006, 'z', 16) for sx in (-1, 1) for sy in (-1, 1)]), _cat([cyl(0, 0, base_h, 0.052, 0.090, 'y')])]), 'black': _cat([box(0, 0, 0.020 + (base_h - 0.020) / 2.0, 0.12, 0.12, base_h - 0.020), box(0, 0, 0.060, 0.15, 0.15, 0.030)]), 'bolt': _cat([screws_y(0, base_h, 0.036, 0.0465, 6), screws_y(0, base_h, 0.036, -0.0465, 6)])}

def part_link1(L):
    return {'black': _cat([box(L / 2.0, 0, 0, L - 0.04, 0.040, 0.056), box(L / 2.0, 0, 0.0305, L - 0.10, 0.030, 0.005)]), 'charcoal': _cat([cyl(0, 0, 0, 0.040, 0.066, 'y'), cyl(L, 0, 0, 0.040, 0.066, 'y')]), 'bolt': _cat([screws_y(0, 0, 0.026, 0.0345, 5), screws_y(L, 0, 0.026, 0.0345, 5), screws_y(L, 0, 0.026, -0.0345, 5)]), 'white': cyl(0, 0, 0, 0.020, 0.080, 'y')}

def part_link2(L):
    return {'black': _cat([box(L / 2.0, 0, 0, L - 0.04, 0.040, 0.050), box(L / 2.0, 0, 0.0275, L - 0.10, 0.030, 0.005)]), 'charcoal': _cat([cyl(0, 0, 0, 0.037, 0.092, 'y'), cyl(L, 0, 0, 0.032, 0.070, 'y')]), 'bolt': _cat([screws_y(0, 0, 0.024, 0.0475, 5), screws_y(0, 0, 0.024, -0.0475, 5), screws_y(L, 0, 0.020, 0.0365, 5)]), 'white': cyl(L, 0, 0, 0.016, 0.090, 'y')}

def part_palm(n, phis):
    top, under = -0.040, -ARM['tcp_drop']
    sockets, bolts = [], []
    for phi in phis:
        sockets.append(rotz(*cyl(H.HAND['r_base'], 0.0, under, 0.0155, 0.036, 'y', 28), phi))
        bolts.append(rotz(*_cat([screws_y(H.HAND['r_base'], under, 0.009, 0.0185, 4, 0.0014), screws_y(H.HAND['r_base'], under, 0.009, -0.0185, 4, 0.0014)]), phi))
    return {'black': _cat([cyl(0, 0, 0.5 * (top + under), H.HAND['palm_r'], top - under, 'z', 56)]), 'charcoal': _cat([cyl(0, 0, -0.020, 0.045, 0.040, 'z', 8), cyl(0, 0, top + 0.002, 0.100, 0.004, 'z', 56), cyl(0, 0, 0.004, 0.021, 0.030, 'z', 32)] + sockets), 'bolt': _cat(bolts), 'white': rotz(*cyl(0.082, 0.0, top + 0.0045, 0.015, 0.003, 'z', 32), phis[0] + math.pi / n), 'dark': rotz(*cyl(0.0855, 0.006, top + 0.0055, 0.011, 0.002, 'z', 32), phis[0] + math.pi / n)}

def part_phalanx(k, L):
    hub_len = (0.034, 0.032, 0.030)[k - 1]
    bolts = _cat([screws_y(0, 0, 0.0075, hub_len / 2.0 + 0.0006, 5), screws_y(0, 0, 0.0075, -hub_len / 2.0 - 0.0006, 5)])
    hub = cyl(0, 0, 0, (0.0145, 0.0135, 0.0125)[k - 1], hub_len, 'y', 32)
    if k < 3: return {'black': _cat([box(0.5 * (0.010 + L - 0.010), 0, 0, L - 0.020, 0.020, 0.022), box(L - 0.012, 0, 0, 0.012, 0.024, 0.026)]), 'charcoal': hub, 'bolt': bolts}
    return {'black': box(0.5 * (0.010 + L - 0.024), 0, 0, L - 0.034, 0.019, 0.022), 'white': _cat([box(L - 0.018, 0, 0, 0.012, 0.019, 0.022), ellipsoid(L - 0.012, 0, 0, 0.012, 0.0095, 0.011, 28, 16)]), 'charcoal': hub, 'bolt': bolts}

def ensure_gripper_meshes(z0=0.080, n=None):
    n = int(n or H.HAND['n_fingers'])
    H.HAND['n_fingers'] = n
    os.makedirs(MESH_DIR, exist_ok=True)
    raw_parts = {'base': ('base', part_base(ARM['zs'] + z0)), 'link1': ('link1', part_link1(ARM['L1'])), 'link2': ('link2', part_link2(ARM['L2'])), 'palm': ('palm', part_palm(n, H.azimuths(n))), 'phalanx1': ('phalanx1', part_phalanx(1, H.HAND['L'][0])), 'phalanx2': ('phalanx2', part_phalanx(2, H.HAND['L'][1])), 'phalanx3': ('phalanx3', part_phalanx(3, H.HAND['L'][2]))}
    mesh_map = {}
    for key, (uname, groups) in raw_parts.items():
        entries = []
        for cname, (V, F) in groups.items():
            stl_name = 'grip_%s_%s_n%d.stl' % (key, cname, n) if key == 'palm' else 'grip_%s_%s.stl' % (key, cname)
            fpath = os.path.join(MESH_DIR, stl_name)
            if not os.path.isfile(fpath) or key == 'palm': write_stl(fpath, V, F)
            entries.append((cname, fpath))
        mesh_map[key] = entries
    return mesh_map
