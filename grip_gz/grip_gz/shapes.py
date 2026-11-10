"""shapes.py - exact signed-distance geometry of the specimen (box / cylinder / sphere / CAD mesh).

Frame: object frame = world xy (part centred on the hand axis), z measured up from the table top (z = 0 is the table).
sdf(P) < 0 means the point is inside the part.  Used by the grasp planner, the collision checker and the live
clearance monitor, so the planner, the checker and what MuJoCo draws all agree on the same geometry.
"""
import math
import numpy as np


_CACHE = {}


def height(o): return o['dims'][2] if o['shape'] == 'box' else o['dims'][1]


def _sdf2d(pts, a, b, chunk=3_000_000):
    """Signed 2-D distance of pts (N,2) to a closed outline given as unordered segments a->b (M,2)."""
    N, M = len(pts), len(a)
    out = np.empty(N)
    ab = b - a
    l2 = np.maximum((ab * ab).sum(1), 1e-18)
    step = max(1, chunk // max(M, 1))
    for i in range(0, N, step):
        p = pts[i:i + step]
        t = np.clip(((p[:, None, :] - a[None]) * ab[None]).sum(2) / l2[None], 0.0, 1.0)
        q = a[None] + t[..., None] * ab[None]
        d = np.sqrt(((p[:, None, :] - q) ** 2).sum(2)).min(1)
        ay, by = a[None, :, 1], b[None, :, 1]
        cross = (ay > p[:, None, 1]) != (by > p[:, None, 1])
        xint = a[None, :, 0] + (p[:, None, 1] - ay) * (b[None, :, 0] - a[None, :, 0]) / np.where(by == ay, 1e-18, by - ay)
        inside = (cross & (p[:, None, 0] < xint)).sum(1) % 2 == 1
        out[i:i + step] = np.where(inside, -d, d)
    return out


def _inside_mask(a, b, x0, y0, res, nx, ny):
    """Even-odd fill of the closed outline (segments a->b) on the node grid (x0 + i*res, y0 + j*res). Returns bool (ny, nx)."""
    ys = y0 + res * np.arange(ny)
    ay, by = a[:, 1], b[:, 1]
    lo, hi = np.minimum(ay, by), np.maximum(ay, by)
    cnt = np.zeros((ny, nx + 1), dtype=np.int32)
    j0 = np.clip(np.ceil((lo - y0) / res - 1e-9).astype(int), 0, ny)
    j1 = np.clip(np.ceil((hi - y0) / res - 1e-9).astype(int), 0, ny)          # rows j0 <= j < j1 satisfy lo <= y_j < hi
    num = np.maximum(j1 - j0, 0)
    tot = int(num.sum())
    if tot:
        k = np.repeat(np.arange(len(a)), num)
        rows = j0[k] + (np.arange(tot) - np.repeat(np.cumsum(num) - num, num))
        t = (ys[rows] - ay[k]) / (by[k] - ay[k])
        xi = a[k, 0] + t * (b[k, 0] - a[k, 0])
        ix = np.clip(np.ceil((xi - x0) / res - 1e-9).astype(int), 0, nx)      # first node to the right of the crossing
        np.add.at(cnt, (rows, ix), 1)
    return (np.cumsum(cnt[:, :nx], axis=1) % 2).astype(bool)


def _grid_sdf2d(a, b, x0, y0, res, nx, ny):
    """Signed 2-D distance (m, negative inside) of the outline on a node grid.  Exact distance to the outline (boundary is sampled
    at res/4 and looked up in a KD-tree), sign from an even-odd fill."""
    mask = _inside_mask(a, b, x0, y0, res, nx, ny)
    gx, gy = np.meshgrid(x0 + res * np.arange(nx), y0 + res * np.arange(ny))
    G = np.c_[gx.ravel(), gy.ravel()]
    try:
        from scipy.spatial import cKDTree
        L = np.linalg.norm(b - a, axis=1)
        m = np.maximum(1, np.ceil(L / (0.25 * res)).astype(int))
        seg = np.repeat(np.arange(len(a)), m)
        t = (np.arange(len(seg)) - np.repeat(np.cumsum(m) - m, m) + 0.5) / m[seg]
        pts = a[seg] + t[:, None] * (b - a)[seg]
        d = cKDTree(pts).query(G, workers=-1)[0]
    except ImportError:                                                        # no scipy: slower exact brute force
        d = np.abs(_sdf2d(G, a, b))
    return np.where(mask.ravel(), -d, d).reshape(ny, nx).astype(np.float32)


class Shape:
    def __init__(self, o):
        self.o = o
        self.h = float(height(o))
        self.kind = 'mesh' if (o.get('shape') == 'mesh' and o.get('mesh')) else ('box' if o['shape'] == 'box' else ('sphere' if o.get('vis') == 'sphere' else 'cyl'))
        self.com = np.array(o.get('com') or (0.0, 0.0, self.h / 2.0), dtype=float)
        if self.kind == 'mesh':
            from .cad import load_mesh
            from . import meshprep as MP
            V, F = load_mesh(o['mesh'])
            self.mesh_h = float(V[:, 2].max() - V[:, 2].min())
            zmid = 0.5 * float(V[:, 2].max() + V[:, 2].min())
            K = int(min(80, max(10, self.mesh_h / 0.0015)))
            sec = MP.build_sections(V, F, K, zmid - self.mesh_h / 2.0 + 2e-4, zmid + self.mesh_h / 2.0 - 2e-4)
            self.zl, self.zmid, self.polys = sec['zs'], zmid, sec['polys']
            self.levels = []
            for pl in self.polys:                      # closed solid outlines (leak-proof sections, see meshprep.build_sections)
                if pl:
                    A = np.concatenate(pl); B = np.concatenate([np.roll(q, -1, axis=0) for q in pl])
                    self.levels.append((A, B))
                else: self.levels.append(None)
            if all(lv is None for lv in self.levels): raise ValueError('mesh has no solid cross-section')
            for i in range(K):                                  # fill empty levels from the nearest non-empty one
                if self.levels[i] is None:
                    j = min((k for k in range(K) if self.levels[k] is not None), key=lambda k: abs(k - i))
                    self.levels[i] = self.levels[j]; self.polys[i] = self.polys[j]
            self._build_grid()

    GRID_MARGIN = 0.060        # m of distance field kept around the part (hand reach for the planner / clearance checks)

    def _build_grid(self):
        """Precompute a (levels x ny x nx) signed-distance volume so that sdf() is a trilinear lookup instead of a scan over
        every slice segment (which cost O(points x segments) per call and made CAD parts very slow to plan and simulate)."""
        allp = np.concatenate([np.concatenate([lv[0], lv[1]]) for lv in self.levels])
        lo, hi = allp.min(0) - self.GRID_MARGIN, allp.max(0) + self.GRID_MARGIN
        try:
            import scipy.spatial  # noqa: F401
            res = max(0.0005, float((hi - lo).max()) / 360.0)
            while len(self.levels) * ((hi - lo) / res + 1).prod() > 12e6: res *= 1.15     # cap the distance volume (RAM / build time)
        except ImportError:
            res = max(0.001, float((hi - lo).max()) / 150.0)
        self.g_res = res
        self.g_x0, self.g_y0 = float(lo[0]), float(lo[1])
        self.g_nx, self.g_ny = int(math.ceil((hi[0] - lo[0]) / res)) + 1, int(math.ceil((hi[1] - lo[1]) / res)) + 1
        vol = np.empty((len(self.levels), self.g_ny, self.g_nx), dtype=np.float32)
        cache = {}
        for k, (a, b) in enumerate(self.levels):
            key = id(a)
            if key not in cache: cache[key] = _grid_sdf2d(a, b, self.g_x0, self.g_y0, res, self.g_nx, self.g_ny)
            vol[k] = cache[key]
        self.g_flat = vol.ravel()
        self.g_K, self.g_ires = len(self.levels), 1.0 / res
        self.g_z0, self.g_idz = float(self.zl[0]), 1.0 / max(float(self.zl[1] - self.zl[0]), 1e-9)
        st, nxx = self.g_nx * self.g_ny, self.g_nx
        self.g_off = np.array([0, 1, nxx, nxx + 1, st, st + 1, st + nxx, st + nxx + 1], dtype=np.intp)

    def _dxy_mesh(self, P, zm):
        """Trilinear lookup of the precomputed distance volume (see _build_grid)."""
        K, nx, ny = self.g_K, self.g_nx, self.g_ny
        fz = np.minimum(np.maximum((zm - self.g_z0) * self.g_idz, 0.0), K - 1.0)
        k0 = np.minimum(fz.astype(np.intp), K - 2)
        tz = fz - k0
        fx, fy = (P[:, 0] - self.g_x0) * self.g_ires, (P[:, 1] - self.g_y0) * self.g_ires
        cx, cy = np.minimum(np.maximum(fx, 0.0), nx - 1.0), np.minimum(np.maximum(fy, 0.0), ny - 1.0)
        i0, j0 = np.minimum(cx.astype(np.intp), nx - 2), np.minimum(cy.astype(np.intp), ny - 2)
        tx, ty = cx - i0, cy - j0
        base = k0 * (nx * ny) + j0 * nx + i0
        v = self.g_flat[base[None, :] + self.g_off[:, None]]                    # (8, N) corner values
        wx, wy = np.stack([1.0 - tx, tx]), np.stack([1.0 - ty, ty])
        w2 = (wx[None, :, :] * wy[:, None, :]).reshape(4, -1)                    # (4, N)  order: (0,0) (1,0) (0,1) (1,1)
        lo, hi = (v[:4] * w2).sum(0), (v[4:] * w2).sum(0)
        return lo + tz * (hi - lo) + np.hypot(fx - cx, fy - cy) * self.g_res     # beyond the stored field: distance to its edge added

    def is_round(self):
        """True for parts whose cross-section is a circle about the hand axis (yaw is then irrelevant)."""
        if self.kind in ('sphere', 'cyl'): return True
        if self.kind != 'mesh': return False
        r = []
        for lv in (self.levels[len(self.levels) // 4], self.levels[len(self.levels) // 2], self.levels[3 * len(self.levels) // 4]):
            q = np.linalg.norm(np.concatenate([lv[0], lv[1]]), axis=1)
            r.append((q.max() - q.min()) / max(q.mean(), 1e-9))
        return max(r) < 0.03

    def sdf(self, P):
        P = np.asarray(P, dtype=float).reshape(-1, 3)
        o = self.o
        if self.kind == 'box':
            half = np.array(o['dims']) / 2.0
            q = np.abs(P - np.array([0.0, 0.0, half[2]])) - half
            return np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(q.max(axis=1), 0.0)
        if self.kind == 'sphere':
            r = o['dims'][0]
            return np.linalg.norm(P - np.array([0.0, 0.0, r]), axis=1) - r
        if self.kind == 'cyl':
            r, hh = o['dims'][0], o['dims'][1]
            dxy = np.hypot(P[:, 0], P[:, 1]) - r
            dz = np.abs(P[:, 2] - hh / 2.0) - hh / 2.0
            return np.minimum(np.maximum(dxy, dz), 0.0) + np.hypot(np.maximum(dxy, 0.0), np.maximum(dz, 0.0))
        zm = P[:, 2] - self.h / 2.0 + self.zmid                 # object z -> mesh z
        dxy = self._dxy_mesh(P, zm)
        dz = np.abs(P[:, 2] - self.h / 2.0) - self.h / 2.0
        return np.minimum(np.maximum(dxy, dz), 0.0) + np.hypot(np.maximum(dxy, 0.0), np.maximum(dz, 0.0))

    def normal(self, P, eps=4e-4):
        P = np.asarray(P, dtype=float).reshape(-1, 3)
        n = len(P)
        E = np.eye(3) * eps
        S = np.concatenate([P + e for e in E] + [P - e for e in E])             # one batched distance query instead of six
        v = self.sdf(S).reshape(6, n)
        g = np.stack([v[0] - v[3], v[1] - v[4], v[2] - v[5]], axis=1)
        return g / np.maximum(np.linalg.norm(g, axis=1, keepdims=True), 1e-12)


def get_shape(o):
    key = (o.get('label'), o.get('shape'), o.get('mesh'), tuple(o.get('dims')), tuple(o.get('com') or ()))
    sh = _CACHE.get(key)
    if sh is None:
        sh = _CACHE[key] = Shape(o)
    return sh
