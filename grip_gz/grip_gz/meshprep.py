"""meshprep.py - turn ANY downloaded mesh (STL / OBJ / PLY / OFF / ...) into clean, bounded geometry the rest of grip_gz can use.

Real-world files are messy: 600k triangles, thousands of loose slivers and tiny floating pieces, duplicated or inverted faces, open
(non-watertight) shells, Y-up instead of Z-up, wrong units, binary files whose header says 'solid ...', wrong face counts, NaNs.
Everything here is written to survive that.

  read_mesh()      robust reader (binary / ASCII STL, OBJ; PLY, OFF, 3MF, GLB through trimesh when it is installed)
  clean_mesh()     weld, drop NaN / degenerate / duplicate triangles, drop loose debris, make winding consistent and outward
  decimate()       bring the triangle count under MuJoCo's 200 000-face limit (quadric if fast_simplification exists, else clustering)
  detect_up()      guess Z-up vs Y-up from mirror symmetry (ships, cars, planes, animals are left-right symmetric about a VERTICAL plane)
  section_polys()  watertight-independent cross-section: rasterise the slice, seal pin-holes, flood-fill the outside, trace the outline
"""
import math
import re
import struct
import numpy as np

MJ_MAX_FACES = 200000
WORK_FACES = 120000            # faces kept in the working / displayed mesh (visual quality vs. slicing time)


class MeshError(Exception): pass


# ------------------------------------------------------------------------------------------------------------------ reading
_STL_DT = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])


def _read_stl(data):
    n_hdr = struct.unpack('<I', data[80:84])[0] if len(data) >= 84 else 0
    n_fit = (len(data) - 84) // 50 if len(data) >= 84 else 0
    n = 0
    if n_hdr > 0 and 84 + 50 * n_hdr <= len(data): n = n_hdr                         # header count is consistent (padding after it is tolerated)
    elif n_fit > 0 and (len(data) - 84) % 50 == 0: n = n_fit                         # header count is wrong / zero but the size is an exact multiple
    if n > 0:
        arr = np.frombuffer(data, dtype=_STL_DT, count=n, offset=84)
        V = arr['v'].reshape(-1, 3).astype(np.float64)
        if np.isfinite(V).all() or n_hdr > 0 and not data[:84].lstrip().lower().startswith(b'solid'): return V
        if np.isfinite(V).mean() > 0.9: return V                                      # mostly sane binary file: NaNs are dropped later
    txt = data.decode('utf-8', errors='ignore')
    nums = re.findall(r'vertex\s+([-+]?[0-9.]+(?:[eE][-+]?[0-9]+)?)\s+([-+]?[0-9.]+(?:[eE][-+]?[0-9]+)?)\s+([-+]?[0-9.]+(?:[eE][-+]?[0-9]+)?)', txt)
    if nums:
        V = np.array(nums, dtype=np.float64)
        return V[:len(V) // 3 * 3]
    if n_fit > 0:                                                                     # last resort: truncated binary file - use whole records
        arr = np.frombuffer(data, dtype=_STL_DT, count=n_fit, offset=84)
        return arr['v'].reshape(-1, 3).astype(np.float64)
    raise MeshError('Could not read STL file (neither a valid binary nor an ASCII STL).')


def _read_obj(path):
    verts, faces = [], []
    with open(path, 'r', errors='ignore') as fh:
        for line in fh:
            if line.startswith('v '):
                p = line.split()
                if len(p) >= 4: verts.append([float(x) for x in p[1:4]])
            elif line.startswith('f '):
                idx = [int(tok.split('/')[0]) for tok in line.split()[1:] if tok.split('/')[0]]
                idx = [i - 1 if i > 0 else len(verts) + i for i in idx]
                for k in range(1, len(idx) - 1): faces.append([idx[0], idx[k], idx[k + 1]])
    if not verts or not faces: raise MeshError('OBJ file has no geometry.')
    V, F = np.array(verts, dtype=np.float64), np.array(faces, dtype=np.int64)
    if F.max() >= len(V) or F.min() < 0: raise MeshError('OBJ file references missing vertices.')
    return V, F


def _read_trimesh(path):
    try:
        import trimesh
    except ImportError:
        raise MeshError('This format needs the "trimesh" package (pip3 install --user trimesh). STL and OBJ work without it.')
    m = trimesh.load(path, force='mesh', process=False)
    if hasattr(m, 'dump') and not hasattr(m, 'faces'): m = m.dump(concatenate=True)
    if m is None or not len(getattr(m, 'faces', [])): raise MeshError('File contains no triangles.')
    return np.asarray(m.vertices, dtype=np.float64), np.asarray(m.faces, dtype=np.int64)


def read_mesh(path):
    """-> (V (n,3) float64, F (m,3) int64) with raw, unwelded geometry."""
    import os
    path = os.path.expanduser(path)
    if not os.path.isfile(path): raise MeshError('File not found: %s' % path)
    ext = os.path.splitext(path)[1].lower()
    if ext == '.stl':
        with open(path, 'rb') as fh: data = fh.read()
        V = _read_stl(data)
        if len(V) < 3: raise MeshError('STL file contains no triangles.')
        return V, np.arange(len(V)).reshape(-1, 3)
    if ext == '.obj': return _read_obj(path)
    if ext in ('.ply', '.off', '.3mf', '.glb', '.gltf', '.dae', '.stp_mesh'): return _read_trimesh(path)
    # unknown extension: sniff the content (people rename files)
    with open(path, 'rb') as fh: data = fh.read(256 * 1024 * 1024)
    try: return read_mesh_bytes_stl(data)
    except MeshError: raise MeshError('Unsupported file type "%s". Use STL, OBJ (or PLY / OFF / 3MF / GLB with trimesh installed).' % ext)


def read_mesh_bytes_stl(data):
    V = _read_stl(data)
    return V, np.arange(len(V)).reshape(-1, 3)


# ------------------------------------------------------------------------------------------------------------------ cleaning
def _components(F, nV):
    """Connected-component label per vertex and per face (faces connected through shared vertices)."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    r = np.concatenate([F[:, 0], F[:, 1], F[:, 2]])
    c = np.concatenate([F[:, 1], F[:, 2], F[:, 0]])
    g = coo_matrix((np.ones(len(r), dtype=np.int8), (r, c)), shape=(nV, nV))
    n, lab = connected_components(g, directed=False)
    return n, lab, lab[F[:, 0]]


def weld(V, F, tol=None):
    """Merge coincident vertices (exact duplicates first - cheap - then a KD-tree pass for near duplicates), drop degenerate/duplicate faces."""
    diag = float(np.linalg.norm(V.max(axis=0) - V.min(axis=0))) or 1.0
    tol = tol or max(1e-12, 2e-6 * diag)
    q = np.round(V / tol).astype(np.int64)
    _, first, inv = np.unique(q, axis=0, return_index=True, return_inverse=True)
    inv = np.asarray(inv).reshape(-1)
    Vu, F = V[first], inv[F]
    try:
        from scipy.spatial import cKDTree
        from scipy.sparse import coo_matrix
        from scipy.sparse.csgraph import connected_components
        pr = cKDTree(Vu).query_pairs(tol * 1.5, output_type='ndarray')
        if len(pr):
            g = coo_matrix((np.ones(len(pr), dtype=np.int8), (pr[:, 0], pr[:, 1])), shape=(len(Vu), len(Vu)))
            _, lab = connected_components(g, directed=False)
            rep = np.full(lab.max() + 1, -1, dtype=np.int64)
            rep[lab[::-1]] = np.arange(len(Vu))[::-1]                                  # first vertex of each group
            Vu, F = Vu[rep], lab[F]
    except ImportError:
        pass
    return _compact(Vu, F)


def _compact(V, F):
    F = F[(F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 0] != F[:, 2])]
    if len(F):
        _, keep = np.unique(np.sort(F, axis=1), axis=0, return_index=True)           # duplicate faces (both windings)
        F = F[np.sort(keep)]
    used = np.unique(F)
    remap = -np.ones(len(V), dtype=np.int64)
    remap[used] = np.arange(len(used))
    return V[used], remap[F]


def _face_areas(V, F):
    return 0.5 * np.linalg.norm(np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]]), axis=1)


def drop_debris(V, F):
    """Remove loose specks: tiny components that carry almost no area (stray vertices, slivers, 1-2 mm rivets of a 300 mm model).
    A real part, however small relative to the model, is kept if it has noticeable area; a small model made only of small pieces is untouched."""
    n, _, fl = _components(F, len(V))
    if n <= 1: return V, F, 0
    ar = _face_areas(V, F)
    atot = float(ar.sum()) or 1.0
    carea = np.bincount(fl, weights=ar, minlength=n)
    lo = np.full((n, 3), np.inf); hi = np.full((n, 3), -np.inf)
    vl = np.empty(len(V), dtype=np.int64); vl[F[:, 0]] = fl; vl[F[:, 1]] = fl; vl[F[:, 2]] = fl
    for k in range(3):
        np.minimum.at(lo[:, k], vl, V[:, k]); np.maximum.at(hi[:, k], vl, V[:, k])
    cdiag = np.linalg.norm(hi - lo, axis=1)
    diag = float(np.linalg.norm(V.max(0) - V.min(0))) or 1.0
    keep_c = (carea >= 2e-4 * atot) | (cdiag >= 0.02 * diag)
    keep_c |= carea >= carea.max() * 0.05                                              # never drop a big sibling of the main body
    if keep_c.all(): return V, F, 0
    F2 = F[keep_c[fl]]
    V2, F2 = _compact(V, F2)
    return V2, F2, int(n - keep_c.sum())


def _directed_edge_dups(F):
    N = int(F.max()) + 1
    e = np.concatenate([F[:, 0] * N + F[:, 1], F[:, 1] * N + F[:, 2], F[:, 2] * N + F[:, 0]])
    return len(e) - len(np.unique(e))


def orient_consistent(F):
    """Flip triangles so neighbouring faces traverse their shared edge in opposite directions. Returns (F, n_flipped).  Skipped when already consistent."""
    if _directed_edge_dups(F) == 0: return F, 0
    n = len(F)
    ed = {}
    for f in range(n):
        a, b, c = int(F[f, 0]), int(F[f, 1]), int(F[f, 2])
        for u, v in ((a, b), (b, c), (c, a)): ed.setdefault((u, v) if u < v else (v, u), []).append(f)
    F = F.copy(); seen = np.zeros(n, dtype=bool); flipped = 0
    for s0 in range(n):
        if seen[s0]: continue
        seen[s0] = True; stack = [s0]
        while stack:
            f = stack.pop()
            a0, b0, c0 = int(F[f, 0]), int(F[f, 1]), int(F[f, 2])
            for u, v in ((a0, b0), (b0, c0), (c0, a0)):
                for g in ed[(u, v) if u < v else (v, u)]:
                    if g == f or seen[g]: continue
                    x, y, z = int(F[g, 0]), int(F[g, 1]), int(F[g, 2])
                    if (x, y) == (u, v) or (y, z) == (u, v) or (z, x) == (u, v):         # same direction as f -> g is inverted
                        F[g] = F[g][[0, 2, 1]]; flipped += 1
                    seen[g] = True; stack.append(g)
    return F, flipped


def signed_volume_com(V, F):
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    v6 = np.einsum('ij,ij->i', a, np.cross(b, c))
    vol = v6.sum() / 6.0
    if abs(vol) < 1e-18: return 0.0, np.zeros(3)
    return vol, ((a + b + c) / 4.0 * (v6[:, None] / 6.0)).sum(axis=0) / vol


def is_closed(F):
    e = np.sort(np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]), axis=1)
    cnt = np.unique(e, axis=0, return_counts=True)[1]
    return bool(np.all(cnt == 2)), int(np.sum(cnt != 2))


def make_outward(V, F):
    """Flip every closed component whose enclosed volume is negative (inside-out) so MuJoCo / lighting see outward normals."""
    n, _, fl = _components(F, len(V))
    out = F.copy(); nfl = 0
    for k in range(n):
        idx = np.where(fl == k)[0]
        if len(idx) < 4: continue
        if signed_volume_com(V, F[idx])[0] < 0: out[idx] = F[idx][:, [0, 2, 1]]; nfl += 1
    return out, nfl


def clean_mesh(V, F, log=None):
    """Raw (V, F) -> (V, F, info). info: dict of what was repaired."""
    info = dict(raw_faces=int(len(F)), debris=0, flipped=0, bad=0)
    ok = np.isfinite(V).all(axis=1)
    if not ok.all():
        bad_v = ~ok
        keep = ~bad_v[F].any(axis=1)
        info['bad'] = int((~keep).sum()); F = F[keep]
    if len(F) == 0: raise MeshError('Mesh has no valid triangles.')
    used = np.unique(F); remap = -np.ones(len(V), dtype=np.int64); remap[used] = np.arange(len(used)); V, F = V[used], remap[F]
    V, F = weld(V, F)
    if len(F) < 4: raise MeshError('Mesh collapsed to nothing after cleaning (all triangles degenerate).')
    diag = float(np.linalg.norm(V.max(0) - V.min(0)))
    if diag <= 0: raise MeshError('Mesh has zero size.')
    ar = _face_areas(V, F)
    keep = ar > 1e-12 * diag * diag                                                     # slivers with (numerically) no area
    if not keep.all(): info['bad'] += int((~keep).sum()); V, F = _compact(V, F[keep])
    V, F, nd = drop_debris(V, F); info['debris'] = nd
    return V, F, info


# ------------------------------------------------------------------------------------------------------------------ decimation
def _cluster(V, F, cell, lo):
    key = np.floor((V - lo) / cell).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    inv = np.asarray(inv).reshape(-1)
    cnt = np.bincount(inv).astype(float)
    Vn = np.stack([np.bincount(inv, weights=V[:, k]) / cnt for k in range(3)], axis=1)
    Fn = inv[F]
    Fn = Fn[(Fn[:, 0] != Fn[:, 1]) & (Fn[:, 1] != Fn[:, 2]) & (Fn[:, 0] != Fn[:, 2])]
    return Vn, Fn


def decimate(V, F, target=WORK_FACES):
    """Reduce to <= target triangles.  Quadric edge collapse (best shape) when fast_simplification is available, otherwise vertex clustering."""
    if len(F) <= target: return V, F
    try:
        import fast_simplification
        Vq, Fq = fast_simplification.simplify(V.astype(np.float32), F.astype(np.int32), target_count=int(target), agg=5)
        Vq, Fq = _compact(Vq.astype(np.float64), Fq.astype(np.int64))
        if 0 < len(Fq) <= MJ_MAX_FACES: return Vq, Fq
    except Exception:
        pass
    lo = V.min(0); diag = float(np.linalg.norm(V.max(0) - lo))
    a, b = diag * 1e-5, diag * 0.1
    best = None
    for _ in range(14):                                                                  # bisection on the cell size (face count falls monotonically)
        mid = math.sqrt(a * b)
        Vn, Fn = _cluster(V, F, mid, lo)
        if len(Fn) > target: a = mid
        else: b = mid; best = (Vn, Fn)
        if best is not None and len(best[1]) > 0.8 * target: break
    if best is None: best = _cluster(V, F, b, lo)
    return _compact(*best)


# ------------------------------------------------------------------------------------------------------------------ orientation
def _surface_samples(V, F, n=20000, seed=0):
    rng = np.random.default_rng(seed)
    ar = _face_areas(V, F); p = ar / ar.sum()
    idx = rng.choice(len(F), size=min(n, 4 * len(F)), p=p)
    u, v = rng.random(len(idx)), rng.random(len(idx))
    flip = u + v > 1; u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
    a, b, c = V[F[idx, 0]], V[F[idx, 1]], V[F[idx, 2]]
    return a + u[:, None] * (b - a) + v[:, None] * (c - a)


def mirror_symmetry(V, F):
    """Per axis: how well the part mirrors onto itself through the middle plane normal to that axis (0 = perfect, ~0.3 = unrelated).  None without scipy."""
    try:
        from scipy.spatial import cKDTree
    except ImportError:
        return None
    P = _surface_samples(V, F)
    lo, hi = V.min(0), V.max(0)
    diag = float(np.linalg.norm(hi - lo))
    tree = cKDTree(P)
    out = []
    for k in range(3):
        Q = P.copy(); Q[:, k] = (lo[k] + hi[k]) - Q[:, k]
        d = tree.query(Q, workers=-1)[0]
        out.append(float(np.percentile(d, 90)) / diag)
    return out


def detect_up(V, F):
    """'z' or 'y'.  STL / CAD convention is Z-up; game and DCC models (Blender export, Sketchfab, ...) are Y-up.
    Ships, cars, aircraft, animals ... are mirror-symmetric about a VERTICAL plane.  If a part is symmetric about the plane normal to the
    file's Z axis, but about neither the X nor the Y plane, that Z axis is a lateral axis, so the part must be standing along Y.
    Flat extruded profiles (thin in Z) are also Z-mirror-symmetric, hence the thickness test.  Returns (up, reason)."""
    ext = V.max(0) - V.min(0)
    s = mirror_symmetry(V, F)
    if s is None: return 'z', 'scipy missing: assuming Z-up'
    srt = np.sort(ext)
    if s[2] < 0.012 and s[2] < 0.5 * min(s[0], s[1]) and ext[2] >= 0.30 * srt[1]:
        return 'y', 'mirror-symmetric about Z plane only (lateral axis) -> Y-up model (symmetry x/y/z = %.3f/%.3f/%.3f)' % tuple(s)
    if s[0] < 0.02 and s[0] < 0.5 * min(s[1], s[2]) and ext[1] > 1.15 * ext[2]:
        return 'y', 'mirror-symmetric about X plane only and tallest along Y (chairs, people, animals) -> Y-up model (symmetry x/y/z = %.3f/%.3f/%.3f)' % tuple(s)
    return 'z', 'Z-up (STL convention; symmetry x/y/z = %.3f/%.3f/%.3f)' % tuple(s)


# ------------------------------------------------------------------------------------------------------------------ robust cross-sections
def slice_segments(V, F, axis, val):
    """Intersection of the triangle soup with the plane  x[axis] = val  -> (M, 2, 3) segments."""
    tri = V[F]
    d = tri[:, :, axis] - val
    d = np.where(np.abs(d) < 1e-12, 1e-12, d)
    sg = d > 0
    mix = sg.any(axis=1) & ~sg.all(axis=1)
    if not mix.any(): return np.zeros((0, 2, 3))
    tri, d, sg = tri[mix], d[mix], sg[mix]
    # the lone vertex on one side + the two on the other side -> the two edges that cross the plane
    lone = np.where(sg.sum(axis=1) == 1, np.argmax(sg, axis=1), np.argmax(~sg, axis=1))
    i0 = lone; i1 = (lone + 1) % 3; i2 = (lone + 2) % 3
    r = np.arange(len(tri))
    p0, p1, p2 = tri[r, i0], tri[r, i1], tri[r, i2]
    d0, d1, d2 = d[r, i0], d[r, i1], d[r, i2]
    a = p0 + (d0 / (d0 - d1))[:, None] * (p1 - p0)
    b = p0 + (d0 / (d0 - d2))[:, None] * (p2 - p0)
    return np.stack([a, b], axis=1)


def _raster_lines(a, b, x0, y0, res, nx, ny):
    """Burn segments a->b (M,2) into a boolean (ny,nx) grid (node (i,j) at x0+i*res, y0+j*res)."""
    L = np.linalg.norm(b - a, axis=1)
    m = np.maximum(1, np.ceil(L / (0.5 * res)).astype(np.int64) + 1)
    tot = int(m.sum())
    seg = np.repeat(np.arange(len(a)), m)
    t = (np.arange(tot) - np.repeat(np.cumsum(m) - m, m)) / np.maximum(m[seg] - 1, 1)
    p = a[seg] + t[:, None] * (b - a)[seg]
    ix = np.clip(np.rint((p[:, 0] - x0) / res).astype(np.int64), 0, nx - 1)
    iy = np.clip(np.rint((p[:, 1] - y0) / res).astype(np.int64), 0, ny - 1)
    g = np.zeros((ny, nx), dtype=bool)
    g[iy, ix] = True
    return g


def fill_section(seg, x0, y0, res, nx, ny, seal=2):
    """Solid cross-section mask from (possibly leaky, self-overlapping, internally-walled) slice segments.
    Burn the outline into a grid, thicken it by `seal` pixels so pin-holes / cracks close, flood-fill the outside from the border, call
    everything else solid, then undo the thickening.  No watertightness, orientation or loop-chaining required."""
    from scipy import ndimage as ndi
    if len(seg) == 0: return np.zeros((ny, nx), dtype=bool)
    bar = _raster_lines(seg[:, 0, :2], seg[:, 1, :2], x0, y0, res, nx, ny)
    st = ndi.generate_binary_structure(2, 2)
    wall = ndi.binary_dilation(bar, structure=st, iterations=seal)
    lab, n = ndi.label(~wall)
    border = np.unique(np.concatenate([lab[0], lab[-1], lab[:, 0], lab[:, -1]]))
    border = border[border > 0]
    outside = np.isin(lab, border)
    solid = ~outside
    solid = ndi.binary_erosion(solid, structure=st, iterations=seal) | bar          # give the thickening back, keep the exact outline pixels
    solid = ndi.binary_closing(solid, structure=st, iterations=1)
    lab, n = ndi.label(solid)                                                       # drop crumbs far smaller than the body
    if n > 1:
        sz = np.bincount(lab.ravel())[1:]
        solid = np.isin(lab, 1 + np.where(sz >= max(4, 0.003 * sz.max()))[0])
    return ndi.binary_fill_holes(solid)


def mask_polys(mask, x0, y0, res, smooth=0.6):
    """Outer boundary loops of a solid mask as closed polygons (list of (n,2) arrays in metres), longest first (sub-pixel via contourpy)."""
    from scipy import ndimage as ndi
    pad = np.pad(mask.astype(float), 2)
    f = ndi.gaussian_filter(pad, smooth) if smooth > 0 else pad
    try:
        import contourpy
        cg = contourpy.contour_generator(z=f, name='serial', line_type=contourpy.LineType.Separate)
        lines = cg.lines(0.5)
    except ImportError:
        import matplotlib.pyplot as plt
        fig = plt.figure(); ax = fig.add_subplot(111)
        cs = ax.contour(f, levels=[0.5]); plt.close(fig)
        lines = [np.asarray(sg) for sg in (cs.allsegs[0] if hasattr(cs, 'allsegs') else [])]
    polys = []
    for ln in lines:
        if len(ln) < 4: continue
        ln = np.asarray(ln, dtype=float)
        if np.linalg.norm(ln[0] - ln[-1]) > 1e-6: continue
        xy = np.stack([x0 + (ln[:, 0] - 2) * res, y0 + (ln[:, 1] - 2) * res], axis=1)
        polys.append(xy[:-1])
    polys.sort(key=lambda p: -abs(poly_area(p)))
    return polys


def poly_area(P):
    x, y = P[:, 0], P[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def douglas_peucker(P, tol):
    """Simplify a CLOSED polygon (n,2)."""
    n = len(P)
    if n <= 8: return P
    i0 = 0
    i1 = int(np.argmax(np.linalg.norm(P - P[0], axis=1)))
    def rec(lo, hi, pts, keep):
        if hi - lo < 2: return
        a, b = pts[lo], pts[hi]
        ab = b - a; L = np.linalg.norm(ab)
        seg = pts[lo + 1:hi]
        d = np.abs(ab[0] * (a[1] - seg[:, 1]) - (a[0] - seg[:, 0]) * ab[1]) / L if L > 1e-15 else np.linalg.norm(seg - a, axis=1)
        k = int(np.argmax(d))
        if d[k] > tol:
            m = lo + 1 + k; keep[m] = True
            rec(lo, m, pts, keep); rec(m, hi, pts, keep)
    PP = np.vstack([P, P[:1]])
    keep = np.zeros(n + 1, dtype=bool); keep[[0, i1, n]] = True
    rec(0, i1, PP, keep); rec(i1, n, PP, keep)
    return PP[:-1][keep[:-1]]


def build_sections(V, F, K, zlo=None, zhi=None, max_cells=12_000_000, margin=0.0):
    """Solid cross-sections of the mesh at K equally spaced heights between zlo and zhi (default: the mesh's z range, inset 0.2 mm).
    -> dict(zs, polys[K] (list of closed (n,2) loops, biggest first), area[K], cen[K,2], grid=(x0,y0,res,nx,ny))."""
    lo, hi = V.min(0), V.max(0)
    zlo = lo[2] + 2e-4 if zlo is None else zlo
    zhi = hi[2] - 2e-4 if zhi is None else zhi
    zs = np.linspace(zlo, zhi, K)
    ext = float(max(hi[0] - lo[0], hi[1] - lo[1]))
    res = max(2.5e-4, ext / 700.0)
    pad = 6 * res
    x0, y0 = lo[0] - pad, lo[1] - pad
    nx, ny = int(math.ceil((hi[0] - lo[0] + 2 * pad) / res)) + 1, int(math.ceil((hi[1] - lo[1] + 2 * pad) / res)) + 1
    polys, area, cen = [], np.zeros(K), np.zeros((K, 2))
    gx = x0 + res * np.arange(nx); gy = y0 + res * np.arange(ny)
    for k, z in enumerate(zs):
        seg = slice_segments(V, F, 2, float(z))
        if len(seg) == 0: polys.append([]); continue
        m = fill_section(seg, x0, y0, res, nx, ny)
        c = int(m.sum())
        if c == 0: polys.append([]); continue
        area[k] = c * res * res
        jj, ii = np.nonzero(m)
        cen[k] = (gx[ii].mean(), gy[jj].mean())
        polys.append(mask_polys(m, x0, y0, res))
    return dict(zs=zs, polys=polys, area=area, cen=cen, grid=(x0, y0, res, nx, ny))


def footprint_width(P2):
    """Narrowest extent of a 2-D point set over all directions -> (width, angle_deg of that direction, widest extent)."""
    ang = np.radians(np.arange(0, 180, 2.0))
    D = np.stack([np.cos(ang), np.sin(ang)], axis=1)
    pr = P2 @ D.T
    ptp = pr.max(0) - pr.min(0)
    k = int(np.argmin(ptp))
    return float(ptp[k]), float(np.degrees(ang[k])), float(ptp.max())
