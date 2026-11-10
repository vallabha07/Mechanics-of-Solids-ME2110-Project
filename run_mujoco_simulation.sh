#!/usr/bin/env bash
# mujoco_15.sh - geometry-true grasping hand in MuJoCo.
#   v3: fixes finger parts penetrating asymmetric parts.  Grasps are planned on the real part geometry
#       (signed-distance field), the hand has spread joints + wrist yaw + dome fingertips, unsafe grasps are
#       refused, and every plan is re-verified in MuJoCo rigid-contact physics (grip_gz/physics_check.py).

set -e

# usage:  bash mujoco_12.sh                                   -> built-in objects (pick in the GUI)
#         bash mujoco_12.sh part.stl                          -> your CAD part, aluminium, mm
#         bash mujoco_12.sh part.stl foam cm                  -> material and unit (positional)
#         bash mujoco_12.sh part.stl foam mm --mu 0.7 --E 5e6 --strength 2e5 --mass 0.12 --kind cellular
#         (flags use SI units: E, strength in Pa; mass in kg; density in kg/m3; F-allow in N; R-contact in m)
#         FINGERS=3 bash mujoco_12.sh ...                     -> 3/4/5 finger hand
FINGERS="${FINGERS:-5}"
CAD_IN=""; CAD_MAT="aluminium"; CAD_UNIT="mm"
if [ $# -gt 0 ] && [[ "$1" != -* ]]; then CAD_IN="$1"; shift; fi
if [ $# -gt 0 ] && [[ "$1" != -* ]]; then CAD_MAT="$1"; shift; fi
if [ $# -gt 0 ] && [[ "$1" != -* ]]; then CAD_UNIT="$1"; shift; fi
CAD_EXTRA=("$@")
if [ -n "$CAD_IN" ]; then
  CAD_IN="$(realpath -m "$CAD_IN")"
  [ -f "$CAD_IN" ] || { echo "ERROR: CAD file not found: $CAD_IN"; exit 1; }
fi
WS="${MOS_WS:-$HOME/mos_ws}"

source /opt/ros/humble/setup.bash
python3 -c "import numpy" 2>/dev/null || { echo "numpy missing: sudo apt install python3-numpy"; exit 1; }
python3 -c "import matplotlib" 2>/dev/null || { echo "matplotlib missing: sudo apt install python3-matplotlib"; exit 1; }
python3 -c "import tkinter" 2>/dev/null || { echo "tkinter missing: sudo apt install python3-tk"; exit 1; }
python3 -c "import mujoco" 2>/dev/null || {
  echo "Installing Python mujoco package..."
  pip3 install --user mujoco
}

python3 -c "import scipy" 2>/dev/null || {
  echo "Installing scipy (fast CAD distance fields)..."
  pip3 install --user scipy || sudo apt install -y python3-scipy || echo "scipy unavailable - CAD parts will plan more slowly"
}

python3 -c "import fast_simplification" 2>/dev/null || pip3 install --user fast-simplification >/dev/null 2>&1 || echo "(optional) fast-simplification unavailable - using built-in mesh decimation"
python3 -c "import trimesh" 2>/dev/null || pip3 install --user trimesh >/dev/null 2>&1 || echo "(optional) trimesh unavailable - PLY/OFF/3MF/GLB import disabled (STL/OBJ still work)"

PKG="$WS/src/grip_gz"
mkdir -p "$PKG"
mkdir -p "$PKG/$(dirname README.md)"
cat << 'GRIPEOF' > "$PKG/README.md"
# grip_gz v3 - geometry-true adaptive gripper in MuJoCo (ROS 2)

What changed from v2 (finger parts penetrating asymmetric parts):
  * The planner now works on the REAL part geometry (signed-distance field of the CAD mesh), not a cylinder of
    "half the width".  Contacts, clearance and the live monitor all use the same geometry.
  * New hand (after Barrett Hand / Robotiq 3F / Yale OpenHand): opposed finger groups, a SPREAD joint per finger,
    a wrist YAW joint, 3-phalanx tendon-synergy fingers and compliant DOME fingertips (single-point contact).
  * Grasp search over aim point x wrist yaw x finger subset; friction-cone check, exact force balance, minimum
    holding force with the true centre of mass, swept clearance (finger-part, finger-table, finger-finger, palm).
  * A part that cannot be held safely is reported as NO SAFE GRIP instead of being lifted anyway.
  * physics_check.py re-tests every plan in MuJoCo rigid-contact physics (real colliding bodies, friction) and keeps
    only a plan that really lifts the part with zero palm/phalanx contact:   python -m grip_gz.physics_check all 5

CAD input:  bash mujoco_12.sh part.stl [material] [unit] [--mu 0.6 --E 5e6 --strength 2e5 --mass 0.2 ...]
            or press BROWSE STL / OBJ in the dashboard's CAD INPUT bar.
Limits:     physics_check approximates a CAD part by its mid-height outline extruded to full height (exact for
            prismatic parts).  Torsional (soft-finger) contact friction is modelled; no other compliance.

Graphics: 8192 px shadow map, 16x offscreen MSAA, 96-segment round parts, textured bench and sky box
          (maximise the MuJoCo window for the sharpest image; a 3840x2160 offscreen buffer is configured).
Small parts: STL welding is now relative to part size (no false "not watertight" on tiny parts), inverted triangles
          are repaired, unit mistakes are flagged.  Parts need >= ~21 mm height; below ~35 mm width use FINGERS=3 or 4.

GRIPEOF

mkdir -p "$PKG/$(dirname package.xml)"
cat << 'GRIPEOF' > "$PKG/package.xml"
<?xml version="1.0"?>
<package format="3">
  <name>grip_gz</name>
  <version>0.2.0</version>
  <description>Mechanics of Solids adaptive grip force control simulation in MuJoCo</description>
  <maintainer email="co25btech11020@iith.ac.in">student</maintainer>
  <license>MIT</license>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>python3-matplotlib</exec_depend>
  <export><build_type>ament_python</build_type></export>
</package>
GRIPEOF

mkdir -p "$PKG/$(dirname setup.cfg)"
cat << 'GRIPEOF' > "$PKG/setup.cfg"
[develop]
script_dir=$base/lib/grip_gz
[install]
install_scripts=$base/lib/grip_gz
GRIPEOF

mkdir -p "$PKG/$(dirname setup.py)"
cat << 'GRIPEOF' > "$PKG/setup.py"
from glob import glob
from setuptools import setup

package_name = 'grip_gz'

setup(
    name=package_name,
    version='0.3.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='co25btech11020@iith.ac.in',
    description='Mechanics of Solids adaptive grip force control simulation in MuJoCo',
    license='MIT',
    entry_points={
        'console_scripts': [
            'grip_controller = grip_gz.controller_node:main',
            'grip_judge = grip_gz.judge_node:main',
            'grip_dashboard = grip_gz.dashboard:main',
            'grip_world = grip_gz.world:main',
            'grip_materials = grip_gz.materials:main',
            'grip_cad = grip_gz.cad:main',
            'grip_hand = grip_gz.hand_plant:main',
            'grip_physics = grip_gz.physics_check:main',
        ],
    },
)
GRIPEOF

mkdir -p "$PKG/$(dirname resource/grip_gz)"
cat << 'GRIPEOF' > "$PKG/resource/grip_gz"

GRIPEOF

mkdir -p "$PKG/$(dirname grip_gz/__init__.py)"
cat << 'GRIPEOF' > "$PKG/grip_gz/__init__.py"

GRIPEOF

mkdir -p "$PKG/$(dirname grip_gz/arm_ik.py)"
cat << 'GRIPEOF' > "$PKG/grip_gz/arm_ik.py"
"""arm_ik.py - Planar 2-link robotic arm forward and inverse kinematics.

World frame coordinates:
  Shoulder joint at (xs, 0, zs + z0).
  Link 1 (length L1) pitches by th1 about +Y.
  Link 2 (length L2) pitches by th2 about +Y relative to Link 1.
  Wrist joint pitches by th3 = -(th1 + th2) so the palm stays horizontal.
"""
import math

ARM = dict(
    xs=-0.55, zs=0.25,
    L1=0.40, L2=0.40,
    tcp_drop=0.06,
    wrist_home=(0.18, 0.28),
)


def ik_wrist(wx, wz, arm=ARM):
    """Inverse kinematics from wrist position (wx, wz) to joint angles (th1, th2, th3)."""
    X = wx - arm['xs']
    D = arm['zs'] - wz
    L1, L2 = arm['L1'], arm['L2']
    c2 = (X * X + D * D - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    c2 = max(-1.0, min(1.0, c2))
    th2 = math.acos(c2)
    th1 = math.atan2(D, X) - math.atan2(L2 * math.sin(th2), L1 + L2 * math.cos(th2))
    return th1, th2, -(th1 + th2)


def fk_wrist(th1, th2, arm=ARM):
    """Forward kinematics from (th1, th2) to wrist position (wx, wz)."""
    x = arm['xs'] + arm['L1'] * math.cos(th1) + arm['L2'] * math.cos(th1 + th2)
    z = arm['zs'] - arm['L1'] * math.sin(th1) - arm['L2'] * math.sin(th1 + th2)
    return x, z


def home_angles(arm=ARM):
    return ik_wrist(arm['wrist_home'][0], arm['wrist_home'][1], arm)


def tcp_home(arm=ARM):
    return arm['wrist_home'][0], arm['wrist_home'][1] - arm['tcp_drop']


def tcp_to_joint_offsets(tcp_x, tcp_z, arm=ARM):
    h1, h2, h3 = home_angles(arm)
    t1, t2, t3 = ik_wrist(tcp_x, tcp_z + arm['tcp_drop'], arm)
    return t1 - h1, t2 - h2, t3 - h3


def joints_to_tcp(d1, d2, arm=ARM):
    h1, h2, _ = home_angles(arm)
    wx, wz = fk_wrist(h1 + d1, h2 + d2, arm)
    return wx, wz - arm['tcp_drop']
GRIPEOF

mkdir -p "$PKG/$(dirname grip_gz/materials.py)"
cat << 'GRIPEOF' > "$PKG/grip_gz/materials.py"
"""materials.py - Specimen database, elastomeric pad properties, and Hertzian contact mechanics."""
import math

G = 9.81
INF = float('inf')

def _hand_open():
    from .hand import r_open
    return r_open()

GRIPPER = dict(
    F_max=40.0,
    finger_mass=0.05,
    finger_damping=8.0,
    open_half_gap=_hand_open(),
    travel=0.09,
    pad_x=0.03, pad_t=0.01, pad_h=0.04,
    lift_height=0.10,
)

PADS = {
    'small': dict(R=0.025, E=2.0e6, nu=0.49),
    'large': dict(R=0.100, E=2.0e6, nu=0.49),
}

OBJECTS = {
    'steel': dict(label='Steel block 1 kg', shape='box', dims=(0.05, 0.05, 0.05), vis='box',
                  m=1.00, mu=0.60, E=200e9, nu=0.30, R=INF, p_allow=250e6, F_allow=INF,
                  mode='Von Mises (Ductile Yield)', rgba=(0.62, 0.64, 0.68, 1)),
    'apple': dict(label='Apple', shape='cyl', dims=(0.037, 0.074), vis='sphere',
                  m=0.20, mu=0.70, E=5e6, nu=0.35, R=0.037, p_allow=0.30e6, F_allow=INF,
                  mode='Von Mises (Cellular Bruise)', rgba=(0.85, 0.15, 0.15, 1)),
    'egg': dict(label='Egg', shape='cyl', dims=(0.022, 0.044), vis='sphere',
                m=0.06, mu=0.40, E=30e9, nu=0.30, R=0.022, p_allow=INF, F_allow=20.0,
                mode='Rankine / Shell Fracture', rgba=(0.95, 0.92, 0.80, 1)),
    'glass': dict(label='Glass tumbler', shape='cyl', dims=(0.035, 0.10), vis='cyl',
                  m=0.25, mu=0.60, E=70e9, nu=0.22, R=0.035, p_allow=INF, F_allow=25.0,
                  mode='Rankine (Brittle Fracture)', rgba=(0.60, 0.85, 0.95, 1)),
    'bottle_empty': dict(label='Plastic bottle (empty)', shape='cyl', dims=(0.03, 0.20), vis='cyl',
                         m=0.03, mu=0.50, E=2e9, nu=0.40, R=0.03, p_allow=INF, F_allow=3.0,
                         mode='Thin-Shell Buckling', rgba=(0.30, 0.70, 0.90, 1)),
    'bottle_full': dict(label='Plastic bottle (full)', shape='cyl', dims=(0.03, 0.20), vis='cyl',
                        m=0.53, mu=0.50, E=2e9, nu=0.40, R=0.03, p_allow=INF, F_allow=12.0,
                        mode='Thin-Shell Buckling', rgba=(0.20, 0.45, 0.90, 1)),
    'foam': dict(label='Foam block', shape='box', dims=(0.05, 0.05, 0.04), vis='box',
                 m=0.02, mu=0.80, E=50e3, nu=0.30, R=INF, p_allow=20e3, F_allow=INF,
                 mode='Von Mises (Cellular Crush)', rgba=(0.95, 0.85, 0.20, 1)),
    'wet_glass': dict(label='Wet glass (mu 0.2)', shape='cyl', dims=(0.035, 0.10), vis='cyl',
                      m=0.25, mu=0.20, E=70e9, nu=0.22, R=0.035, p_allow=INF, F_allow=25.0,
                      mode='Rankine (Brittle Fracture)', rgba=(0.40, 0.70, 0.95, 1)),
    'steel_2kg': dict(label='Steel block 2 kg', shape='box', dims=(0.05, 0.05, 0.102), vis='box',
                      m=2.00, mu=0.60, E=200e9, nu=0.30, R=INF, p_allow=250e6, F_allow=INF,
                      mode='Von Mises (Ductile Yield)', rgba=(0.50, 0.52, 0.56, 1)),
}

GRASP_FG = {}      # planner result per specimen: friction capacity = mu * FG * (reference finger force)

def height(o): return o['dims'][2] if o['shape'] == 'box' else o['dims'][1]
def width(o): return o['dims'][1] if o['shape'] == 'box' else 2.0 * o['dims'][0]
def grasp_tcp_z(o):
    if o.get('grasp_z') is not None: return o['grasp_z']
    return max(height(o) - 0.028, 0.032)
def e_star(E1, nu1, E2, nu2): return 1.0 / ((1.0 - nu1 ** 2) / E1 + (1.0 - nu2 ** 2) / E2)
def r_eff(R_pad, R_obj): return R_pad if math.isinf(R_obj) else 1.0 / (1.0 / R_pad + 1.0 / R_obj)
def hertz_a(F, Re, Es): return (3.0 * F * Re / (4.0 * Es)) ** (1.0 / 3.0) if F > 0.0 else 0.0
def hertz_p0(F, Re, Es):
    if F <= 0.0: return 0.0
    a = hertz_a(F, Re, Es)
    return 3.0 * F / (2.0 * math.pi * a * a)
def hertz_delta(F, Re, Es):
    if F <= 0.0: return 0.0
    a = hertz_a(F, Re, Es)
    return a * a / Re
def force_for_p0(p, Re, Es): return INF if math.isinf(p) else (p ** 3) * (math.pi ** 3) * (Re ** 2) / (6.0 * (Es ** 2))

def truth(obj_key, pad_key='small', n=None, psi=0.0):
    if n is None:
        from .hand import HAND
        n = int(HAND['n_fingers'])
    o, p = OBJECTS[obj_key], PADS[pad_key]
    Es = e_star(p['E'], p['nu'], o['E'], o['nu'])
    Re = r_eff(p['R'], o['R'])
    N_slip = o['m'] * G / (GRASP_FG.get(obj_key, n) * o['mu'])
    N_hertz = force_for_p0(o['p_allow'], Re, Es)
    N_max = min(N_hertz, o['F_allow'], GRIPPER['F_max'])
    if N_max == GRIPPER['F_max'] and N_hertz > GRIPPER['F_max'] and o['F_allow'] > GRIPPER['F_max']:
        limiter = 'gripper'
    elif N_hertz <= o['F_allow']:
        limiter = 'Hertz pressure'
    else:
        limiter = 'shell force limit'
    return dict(Es=Es, Re=Re, N_slip=N_slip, N_hertz=N_hertz, N_max=N_max, limiter=limiter, n=n, psi=psi,
                K=4.0 / 3.0 * Es * math.sqrt(Re), p_allow=o['p_allow'], F_allow=o['F_allow'])

def main():
    import sys
    pad = sys.argv[1] if len(sys.argv) > 1 else 'small'
    print('Grip windows, %s pad.  N = force per fingertip.' % pad)
    print('%-14s %8s %9s %9s  %-18s %s' % ('object', 'N_slip', 'N_max', 'ratio', 'limited by', 'window'))
    for k in OBJECTS:
        t = truth(k, pad)
        ratio = t['N_max'] / t['N_slip']
        print('%-14s %8.3f %9.2f %9.1f  %-18s %s' % (
            k, t['N_slip'], t['N_max'], ratio, t['limiter'], 'OPEN' if ratio > 1.3 else 'TOO NARROW'))

if __name__ == '__main__':
    main()
GRIPEOF

mkdir -p "$PKG/$(dirname grip_gz/mos.py)"
cat << 'GRIPEOF' > "$PKG/grip_gz/mos.py"
"""mos.py - Mechanics of Solids subsurface Cauchy stress and failure evaluation."""
import math
from .materials import OBJECTS, PADS, e_star, hertz_a, hertz_delta, hertz_p0, r_eff

ZETA = 0.48

def axis_stresses(p0, nu, zeta=ZETA):
    term = (1.0 + nu) * (1.0 - zeta * math.atan(1.0 / zeta)) - 0.5 / (1.0 + zeta ** 2)
    sig_r = -p0 * term
    sig_z = -p0 / (1.0 + zeta ** 2)
    return sig_r, sig_r, sig_z

def mos_state(N, fy, obj_key, pad_key='small'):
    o, p = OBJECTS[obj_key], PADS[pad_key]
    Es = e_star(p['E'], p['nu'], o['E'], o['nu'])
    Re = r_eff(p['R'], o['R'])
    K = 4.0 / 3.0 * Es * math.sqrt(Re)
    nu, mu = o['nu'], o['mu']
    sig_y_allow = (0.62 * o['p_allow']) if not math.isinf(o['p_allow']) else None

    out = dict(Es=Es, Re=Re, K=K, N=N, fy=fy, mode=o.get('mode', 'Von Mises'))
    if N <= 1e-6:
        out.update(delta=0.0, a=0.0, eps_c=0.0, p0=0.0, p_mean=0.0, tau_mean=0.0, tau_edge=0.0, c_over_a=1.0,
                   sig1=0.0, sig2=0.0, sig3=0.0, tau_sub=0.0, sig_tresca=0.0, sig_vm=0.0, sig_rankine=0.0,
                   sig_y_allow=sig_y_allow, fos=float('inf'), fos_basis='-')
        return out

    a = hertz_a(N, Re, Es)
    area = math.pi * a * a
    p0 = hertz_p0(N, Re, Es)
    q = min(1.0, abs(fy) / max(mu * N, 1e-9))
    c_over_a = (1.0 - q) ** (1.0 / 3.0)
    mu_mob = min(mu, abs(fy) / N)

    sr, st, sz = axis_stresses(p0, nu)
    tau_axis = 0.5 * abs(sr - sz)
    tau_edge = mu_mob * p0
    sig_tresca = 2.0 * math.sqrt(tau_axis ** 2 + tau_edge ** 2)
    sig_vm = math.sqrt((sr - sz) ** 2 + 3.0 * tau_edge ** 2)
    sig_rankine = (1.0 - 2.0 * nu) / 3.0 * p0 + (4.0 + nu) * math.pi / 8.0 * tau_edge

    fos_p = (o['p_allow'] / p0) if not math.isinf(o['p_allow']) else math.inf
    fos_f = (o['F_allow'] / N) if not math.isinf(o['F_allow']) else math.inf
    fos = min(fos_p, fos_f)          # true value: no artificial cap (a stiff CAD part on a soft pad really has FoS in the hundreds)
    basis = 'Hertz / Von Mises yield' if fos_p <= fos_f else 'Thin-shell force limit'

    out.update(delta=hertz_delta(N, Re, Es), a=a, eps_c=0.2 * a / Re, p0=p0, p_mean=N / area,
               tau_mean=abs(fy) / area, tau_edge=tau_edge, c_over_a=c_over_a, sig1=sr, sig2=st, sig3=sz,
               tau_sub=tau_axis, sig_tresca=sig_tresca, sig_vm=sig_vm, sig_rankine=sig_rankine,
               sig_y_allow=sig_y_allow, fos=fos, fos_basis=basis)
    return out
GRIPEOF
mkdir -p "$PKG/$(dirname grip_gz/mech_extras.py)"
cat << 'GRIPEOF' > "$PKG/grip_gz/mech_extras.py"
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








GRIPEOF

mkdir -p "$PKG/$(dirname grip_gz/controller_core.py)"
cat << 'GRIPEOF' > "$PKG/grip_gz/controller_core.py"
"""controller_core.py - Blind adaptive tactile force controller."""
import math
from collections import deque

class Params:
    def __init__(self, **kw):
        self.strategy = 'adaptive'
        self.F_fixed = 25.0
        self.t_cal = 1.0
        self.f0_margin = 0.05
        self.f0_min = 0.05
        self.F_max = 80.0
        self.close_mult = 3.0
        self.contact_mult = 1.5
        self.t_squeeze = 1.0
        self.squeeze_mult = 3.0
        self.R_pad = 0.025
        self.R_obj = None
        self.E_pad = 2.0e6
        self.nu_pad = 0.49
        self.eps_allow = 0.05
        self.t_settle = 0.25
        self.W = 10
        self.r_thr = 0.15
        self.T_probe = 0.4
        self.T_bump = 0.15
        self.bump_mult = 3.0          # probe bump = 3 x noise floor (smaller pokes cannot detect slip on slippery parts)
        self.bump_frac = 0.10
        self.margin = 0.15            # safety margin on the minimum holding force (was 25 %)
        self.rate_abs = 0.6           # N/s   absolute force slew allowed while ramping
        self.rate_rel = 1.2           # 1/s   plus this fraction of the current force per second (geometric ramp)
        self.ramp_min_gain = 0.30     # a slip-ramp must raise the force by at least 30 % before it may end
        self.boost_step = 0.6         # every consecutive confirmed slip makes the next ramp 0.6 x faster (heavy parts need big forces)
        self.boost_max = 6.0
        self.t_hold = 1.2
        self.t_close_timeout = 10.0
        self.t_lift_timeout = 150.0
        self.dt = 0.01
        for k, v in kw.items():
            setattr(self, k, v)

def _mean(x): return sum(x) / len(x)
def _clip(x, lo, hi): return max(lo, min(hi, x))

class GripController:
    def __init__(self, P=None):
        self.P = P or Params()
        self.reset()

    def reset(self):
        P = self.P
        self.state = 'CALIB'
        self.t_state = None
        self.cal = []
        self.bzl = self.bzr = self.byl = self.byr = 0.0
        self.sl = self.sr = 1.0
        self.f0 = None
        self.gate_z = self.gate_y = self.gate_yp = 0.0
        self.fz = self.fy = 0.0
        self.bz = deque(maxlen=P.W + 4)
        self.by = deque(maxlen=P.W + 4)
        self.fd = 0.0
        self.fd_goal = 0.0
        self.x_contact = 0.0
        self.p_e_mm = 0.01
        self.contact_count = 0
        self.sq = []
        self.K = None
        self.Es_est = None
        self.E_obj_est = None
        self.E_obj_lb = False
        self.Re_est = self.P.R_pad
        self.fz_f = self.fy_f = 0.0
        self.eps_est = 0.0
        self.p0_est = 0.0
        self.vm_est = 0.0
        self.N_cap = P.F_max
        self.alpha_d = 0.1
        self.df = 0.0
        self.s_c = 0
        self.ramp = False
        self.ramp_ticks = 0
        self.ramp_quiet = 0
        self.fd_ramp0 = 0.0
        self.boost = 1.0
        self.probe_active = False
        self.probe_t0 = 0.0
        self.t_probe_last = 0.0
        self.pre_z = self.pre_y = self.bump_amt = 0.0
        self.last_ratio = float('nan')
        self.static_probes = 0
        self.margin_done = False
        self.cap_slip = 0
        self.mu_lb = 0.0
        self.m_est = 0.0
        self.t_lift0 = None
        self.result_hint = ''
        self.n_f = getattr(self, 'n_f', 2)
        self.psi = 0.0

    def _goto(self, s, t):
        self.state = s
        self.t_state = t

    def _finish_calib(self):
        P = self.P
        n = len(self.cal)
        self.bzl = sum(c[0] for c in self.cal) / n
        self.bzr = sum(c[1] for c in self.cal) / n
        self.byl = sum(c[2] for c in self.cal) / n
        self.byr = sum(c[3] for c in self.cal) / n
        zs = [((c[0] - self.bzl) + (c[1] - self.bzr)) / 2.0 for c in self.cal]
        ys = [((c[2] - self.byl) + (c[3] - self.byr)) / 2.0 for c in self.cal]
        self.f0 = max(max(abs(z) for z in zs) + P.f0_margin, P.f0_min)
        sz = math.sqrt(sum(z * z for z in zs) / n)
        sy = math.sqrt(sum(y * y for y in ys) / n)
        self.gate_z = max(5 * 0.82 * sz, 0.01)
        self.gate_y = max(5 * 0.82 * sy, 0.01)
        self.gate_yp = max(5 * 0.71 * sy, 0.01)
        self.cal = []

    def _update_mos_estimates(self, delta):
        """Blind estimate of the contact mechanics from the measured stiffness K = F / delta^1.5 = 4/3 * E* * sqrt(R*).
        One K gives one number, but E_obj and the contact radius R* are both unknown, so the branch is chosen from how stiff the
        contact is relative to the pad:
          - soft part  (E*_flat < 0.5 E*_pad): the pad indents the part too -> flat-part inversion gives E_obj (R* = R_pad);
          - stiff part (otherwise): E* is pad-limited (= E*_pad), so K reveals R* = (3K / 4E*_pad)^2 and E_obj can only be
            bounded from below.  (The old code always assumed R* = R_pad, which read a curved stiff part as a ~8 MPa 'soft' one
            and made every p0 / von Mises estimate ~40 % low.)"""
        P = self.P
        Es_pad = P.E_pad / (1.0 - P.nu_pad ** 2)
        if self.K is not None and self.K > 0:
            # contact curvature prior from the CAD / vision stage (None = unknown -> assume the pad radius, which can only under-read)
            Rg = P.R_obj
            Rstar = P.R_pad if Rg is None else (P.R_pad if math.isinf(Rg) else 1.0 / (1.0 / P.R_pad + 1.0 / Rg))
            Es_flat = (3.0 * self.K) / (4.0 * math.sqrt(Rstar))
            self.Re_est = Rstar
            r = Es_flat / Es_pad
            if r < 0.97:                                    # distinguishable from the pad alone -> real estimate
                self.Es_est, self.E_obj_lb = Es_flat, False
                inv_obj = (1.0 / Es_flat) - (1.0 / Es_pad)
                self.E_obj_est = min(250e9, (1.0 - 0.30 ** 2) / inv_obj)
            else:                                           # stiff part: pad-limited, K only bounds E_obj from below (3 % K resolution)
                self.Es_est, self.E_obj_lb = Es_pad, True
                if Rg is None:                              # unknown curvature: infer R* from K instead (stiff part => E* = E*_pad)
                    self.Re_est = min(max((3.0 * self.K / (4.0 * Es_pad)) ** 2, 1.5e-3), P.R_pad)
                self.E_obj_est = (1.0 - 0.30 ** 2) / ((1.0 / (0.97 * Es_pad)) - (1.0 / Es_pad))
        if delta > 1e-8 and self.fz > 1e-3:
            Re = self.Re_est
            a_est = math.sqrt(Re * delta)
            self.eps_est = 0.2 * (a_est / Re)
            self.p0_est = 1.5 * self.fz / max(math.pi * a_est * a_est, 1e-9)
            mu_mob = self.fy / max(self.fz, 0.05)
            self.vm_est = self.p0_est * math.sqrt(0.62 ** 2 + 3.0 * mu_mob ** 2)

    def _step_size(self):
        a = _clip(self.alpha_d, 0.1, 1.0)
        pe = _clip(self.p_e_mm, 0.01, 10.0)
        raw = math.exp(a) + math.exp(-pe)
        lo = math.exp(0.1) + math.exp(-10.0)
        hi = math.exp(1.0) + math.exp(-0.01)
        sc = (raw - lo) / (hi - lo)
        return 4.0 * (self.f0 / 5.0 + (self.f0 / 2.0 - self.f0 / 5.0) * sc)

    def _grip_logic(self, t):
        P = self.P
        if P.strategy == 'fixed': return P.F_fixed
        cap = min(self.N_cap, P.F_max)
        W = P.W
        if self.fd_goal > self.fd:      # slewed (not stepped) approach to a raised force target
            self.fd = min(self.fd + P.dt * (P.rate_abs + P.rate_rel * self.fd), self.fd_goal)
        bz, by = list(self.bz), list(self.by)
        self.s_c = 0
        dz = dy = 0.0
        if len(bz) >= W + 3:
            dz = _mean(bz[-3:]) - _mean(bz[-W - 3:-W])
            dy = _mean(by[-3:]) - _mean(by[-W - 3:-W])
            if dz > self.gate_z and dy > self.gate_y and dy > P.r_thr * dz:
                self.s_c = 1
                self.alpha_d = _clip(dy / dz, 0.1, 1.0)
        extra = 0.0
        if getattr(self, 'not_carried', False) and not self.ramp and not self.probe_active and t - getattr(self, 't_nc', -9.0) > 0.4:
            # wrist load cell: the arm is rising but the part's weight is not arriving -> the grip is too weak, raise it now
            self.ramp, self.ramp_ticks, self.ramp_quiet = True, 0, 0
            self.fd_ramp0 = self.fd
            self.t_nc = t
        if self.probe_active:
            extra = self.bump_amt
            if t - self.probe_t0 >= P.T_bump - 1e-9 and len(bz) >= 4:
                dzp, dyp = _mean(bz[-4:]) - self.pre_z, _mean(by[-4:]) - self.pre_y
                valid = dzp > 0.3 * self.bump_amt
                slipping = valid and dyp > P.r_thr * dzp and dyp > self.gate_yp
                self.last_ratio = dyp / dzp if valid else float('nan')
                self.probe_active = False
                self.t_probe_last = t
                if slipping:
                    self.ramp, self.ramp_ticks, self.ramp_quiet = True, 0, 0
                    self.fd_ramp0 = self.fd
                    self.boost = min(P.boost_max, self.boost + P.boost_step)
                    self.static_probes = 0
                    self.margin_done = False
                    self.alpha_d = _clip(self.last_ratio, 0.1, 1.0)
                    if self.fd >= cap - 1e-9: self.cap_slip += 1
                elif valid:
                    self.cap_slip = 0
                    self.static_probes += 1
                    if self.static_probes >= 2: self.boost = 1.0
                    if self.static_probes >= 2 and not self.margin_done:
                        self.fd_goal = min(self.fd * (1.0 + P.margin), cap)
                        self.margin_done = True
        elif self.ramp:
            self.ramp_ticks += 1
            # force slew limit: never ramp faster than (rate_abs + rate_rel*fd) N/s, so slip detection lag cannot overshoot
            self.df = min(self._step_size(), P.dt * self.boost * (P.rate_abs + P.rate_rel * self.fd))
            self.fd = min(self.fd + self.df, cap)
            self.ramp_quiet = 0 if self.s_c else self.ramp_quiet + 1
            gained = self.fd >= self.fd_ramp0 * (1.0 + P.ramp_min_gain) - 1e-9
            if (self.ramp_ticks > W + 5 and self.ramp_quiet >= 6 and gained) or (self.fd >= cap - 1e-9 and self.ramp_ticks > 30):
                self.ramp = False
                self.t_probe_last = t - P.T_probe + 0.30        # verify with a probe soon after the ramp
                if self.fd >= cap - 1e-9: self.cap_slip += 1
        elif t - self.t_probe_last >= P.T_probe and len(bz) >= 4:
            self.probe_active = True
            self.probe_t0 = t
            self.pre_z, self.pre_y = _mean(bz[-4:]), _mean(by[-4:])
            bump = P.bump_mult * self.f0
            if self.fz > 0.4 and self.mu_lb > 0.45:
                # friction already learned from a loaded contact: a smaller poke is enough to see slip (>= 1.5x the detection gate)
                bump = min(bump, max(1.5 * self.gate_yp / self.mu_lb, 1.5 * self.f0))
            self.bump_amt = min(max(bump, P.bump_frac * self.fd), P.F_max - self.fd)
        return min(self.fd + extra, P.F_max)

    def step(self, t, fzl, fzr, fyl, fyr, pos, delta, arm_at_grasp, arm_at_top):
        P = self.P
        if self.t_state is None: self.t_state = t
        ts = t - self.t_state
        s = self.state
        lift_enable, force, done = False, 0.0, False

        if s == 'CALIB':
            self.cal.append((fzl, fzr, fyl, fyr))
            if ts >= P.t_cal and len(self.cal) >= 20:
                self._finish_calib()
                self._goto('WAIT_ARM', t)
            return self._out(force, lift_enable, done)

        tl, tr = fzl - self.bzl, fzr - self.bzr
        yl, yr = fyl - self.byl, fyr - self.byr
        self.fz = (self.sl * tl + self.sr * tr) / 2.0
        self.fy = abs((yl + yr) / 2.0)
        self.bz.append(self.fz)
        self.by.append(self.fy)
        self.p_e_mm = _clip((pos - self.x_contact) * 1000.0 + delta * 1000.0, 0.01, 10.0)
        self._update_mos_estimates(delta)

        if s == 'WAIT_ARM':
            if arm_at_grasp:
                self.x_contact = pos
                self._goto('CLOSE', t)
        elif s == 'CLOSE':
            force = P.close_mult * self.f0
            c = (abs(tl) + abs(tr)) / 2.0
            self.contact_count = self.contact_count + 1 if c > P.contact_mult * self.f0 else 0
            if self.contact_count >= 3:
                self.sl = 1.0 if tl >= 0 else -1.0
                self.sr = 1.0 if tr >= 0 else -1.0
                self.x_contact = pos
                self.fd = self.f0
                self.sq = []
                self._goto('SQUEEZE', t)
            elif ts > P.t_close_timeout:
                self.result_hint = 'NO OBJECT FOUND'
                self._goto('ABORT', t)
        elif s == 'SQUEEZE':
            frac = _clip(ts / P.t_squeeze, 0.0, 1.0)
            force = P.F_fixed if P.strategy == 'fixed' else self.f0 * (1.0 + (P.squeeze_mult - 1.0) * frac)
            self.fd = force
            if self.fz > 1.8 * self.f0 and delta > 1e-6:
                self.sq.append(self.fz / (delta ** 1.5))
            if ts >= P.t_squeeze:
                if len(self.sq) >= 4:
                    ks = sorted(self.sq)
                    self.K = ks[len(ks) // 2]
                    delta_lim = ((P.eps_allow / 0.2) ** 2) * P.R_pad
                    self.N_cap = min(self.K * (delta_lim ** 1.5), P.F_max)
                    self._update_mos_estimates(delta)
                else:
                    self.K, self.N_cap = None, P.F_max
                self.fd = P.F_fixed if P.strategy == 'fixed' else self.f0
                force = self.fd
                self._goto('SETTLE', t)
        elif s == 'SETTLE':
            force = self.fd = (P.F_fixed if P.strategy == 'fixed' else self.f0)
            self.fd_goal = 0.0
            if ts >= P.t_settle:
                self.t_lift0 = t
                self.t_probe_last = t
                self._goto('LIFT', t)
        elif s in ('LIFT', 'HOLD'):
            lift_enable = (s == 'LIFT')
            force = self._grip_logic(t)
            if self.cap_slip >= 2:
                self.result_hint = 'NO SAFE GRIP'
                self._goto('ABORT', t)
            elif s == 'LIFT' and arm_at_top:
                self._goto('HOLD', t)
            elif s == 'LIFT' and t - self.t_lift0 > P.t_lift_timeout:
                self._goto('DONE', t)
            elif s == 'HOLD' and ts >= P.t_hold:
                self._goto('DONE', t)
            # friction lower bound: only from smoothed forces with a healthy signal-to-noise ratio (the old raw ratio at fz ~ noise level
            # latched values up to 1.0 for mu = 0.6 parts)
            self.fz_f += 0.08 * (self.fz - self.fz_f)
            self.fy_f += 0.08 * (self.fy - self.fy_f)
            if self.fz_f > max(6.0 * self.f0, 0.3) and not self.probe_active:
                self.mu_lb = max(self.mu_lb, self.fy_f / self.fz_f)
            self.m_est = self.n_f * (self.fy * math.cos(self.psi) + self.fz * math.sin(self.psi)) / 9.81
        elif s == 'DONE':
            force, done = self.fd, True
        elif s == 'ABORT':
            force, done = -P.close_mult * self.f0, True

        if self.state in ('DONE', 'ABORT'): done = True
        return self._out(force, lift_enable, done)

    def _out(self, force, lift_enable, done):
        return dict(state=self.state, force=force, lift_enable=lift_enable, done=done,
                    abort=(self.state == 'ABORT'), hint=self.result_hint,
                    fz=self.fz, fy=self.fy, fd=self.fd, f0=self.f0, s_c=self.s_c, df=self.df,
                    alpha=self.alpha_d, p_e=self.p_e_mm, K=self.K, Es_est=self.Es_est, E_obj_est=self.E_obj_est, E_obj_lb=int(self.E_obj_lb), Re_est=self.Re_est,
                    eps_est=self.eps_est, p0_est=self.p0_est, vm_est=self.vm_est, N_cap=self.N_cap,
                    mu_lb=self.mu_lb, m_est=self.m_est, probe=int(self.probe_active),
                    ramp=int(self.ramp), ratio=self.last_ratio)
GRIPEOF

mkdir -p "$PKG/$(dirname grip_gz/hand.py)"
cat << 'GRIPEOF' > "$PKG/grip_gz/hand.py"
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
GRIPEOF

# ---- grip_gz/shapes.py ----
cat << 'GRIPEOF' > "$PKG/grip_gz/shapes.py"
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
GRIPEOF

# ---- grip_gz/physics_check.py ----
cat << 'GRIPEOF' > "$PKG/grip_gz/physics_check.py"
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
GRIPEOF

mkdir -p "$PKG/$(dirname grip_gz/hand_plant.py)"
cat << 'GRIPEOF' > "$PKG/grip_gz/hand_plant.py"
"""hand_plant.py - Unconditionally stable compliant multi-finger contact & stick-slip physics."""
import math
import random
import numpy as np

from . import hand as H
from .arm_ik import tcp_home
from .controller_core import GripController, Params
from . import materials as MAT
from .materials import GRIPPER, OBJECTS, PADS, G, hertz_delta, truth
from .shapes import get_shape
from .mos import mos_state

def min_jerk(s):
    s = max(0.0, min(1.0, s))
    return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s)

# ---- realistic motion limits (SI) -------------------------------------------------------------
MOTION = dict(
    v_close=0.008,      # m/s  fingertip speed in the last gap_slow before expected contact (8 mm/s: gentle tactile search)
    v_approach=0.025,   # m/s  fingertip speed while still far from the object (guarded move)
    gap_slow=0.006,     # m  distance before expected contact where the slow search speed starts
    v_open=0.030,       # m/s  fingertip opening speed
    v_transit=0.12,     # m/s  peak TCP speed of the arm while travelling (min-jerk peak)
    v_final=0.05,       # m/s  peak TCP speed of the last descent onto the object
    t_min_raise=1.5, t_min_over=2.0, t_min_down=2.5,   # s  minimum durations
    lift_speed=0.010,   # m/s  steady lift speed
    w_yaw=0.9,          # rad/s peak wrist-yaw speed
)


class HandPlant:
    KT = 4000.0
    ZETA = 0.70
    KR = 0.85
    TAU_CONTACT = 0.0025

    def __init__(self, o, tr, plan):
        self.o, self.tr, self.plan = o, tr, plan
        self.n = plan['n']
        self.phis = plan['phis']
        self.active = list(plan['active'])
        self.n_act = max(1, sum(self.active))
        self.g = [g if a else 0.0 for g, a in zip(plan['g'], self.active)]
        self.cos_t = list(plan['cos_t'])
        self.fg = float(plan['fric_gain'])
        self.xc = list(plan['xc'])
        x, s, psi = H.curl_tables()
        k = int(np.argmax(x)) + 1
        self.tx, self.ts, self.tpsi = x[:k], s[:k], psi[:k]
        self.xmax = float(self.tx[-1])
        self.tFav = np.array([min(H.force_available(float(si)), 1e6) for si in self.ts])
        self.tC = np.array([H.compliance_along_normal(float(si)) for si in self.ts])
        self.xf = [0.0] * self.n
        self.vf = [0.0] * self.n
        self.N = [0.0] * self.n
        self.zo = self.vo = self.w = self.Ft = self.Fg = self.f_mean = 0.0
        self.psi_c = plan['psi']
        self.deflect = 0.0
        self.sat = self.torque_limited = False
        self._xc_cache = {}

    def s_of(self, x): return float(np.interp(x, self.tx, self.ts))
    def psi_of(self, x): return float(np.interp(x, self.tx, self.tpsi))
    def s_list(self):
        out = []
        for i in range(self.n):
            x = self.xf[i]
            if not self.active[i]: out.append(0.0); continue
            if self.N[i] > 0.0: x = max(self.xc[i], x - self._defl(self.N[i], self.xc[i]))
            out.append(self.s_of(x))
        return out

    def mean_xf(self): return sum(x for x, a in zip(self.xf, self.active) if a) / self.n_act
    def n_contact(self): return sum(1 for v in self.N if v > 0.0)
    def torques(self, i=0): return H.joint_torques(self.N[i], self.s_of(self.xf[i]))
    def _xcv(self, xc):
        key = round(xc, 5)                                              # 10 um buckets: the contact point now moves while the part slides
        v = self._xc_cache.get(key)
        if v is None:
            s = self.s_of(xc)
            v = (float(np.interp(s, self.ts, self.tFav)), float(np.interp(s, self.ts, self.tC)))
            self._xc_cache[key] = v
        return v
    def set_contact_travel(self, i, xc):
        """Move the contact point of finger i along its closing path (the pad rolled over the curved part)."""
        self.xc[i] = float(min(max(xc, 0.002), self.xmax))
    def _clip(self, F, xc): return min(F, self._xcv(xc)[0])
    def _defl(self, N, xc): return self._xcv(xc)[1] * N
    def close_speed(self): return max(self.vf) if self.vf else 0.0

    def step(self, F_cmd, vg, dt):
        mf, b = GRIPPER['finger_mass'], GRIPPER['finger_damping']
        o, tr = self.o, self.tr
        m, mu = o['m'], o['mu']
        n_pos, defl, lim, total_N = 0, 0.0, False, 0.0

        for i in range(self.n):
            if not self.active[i]:
                self.xf[i], self.vf[i], self.N[i] = 0.0, 0.0, 0.0            # parked finger: stays open
                continue
            xf, vf, xc = self.xf[i], self.vf[i], self.xc[i]
            Fi = F_cmd * self.g[i]                                           # weighted so the object's force/torque balance is zero
            N = 0.0
            if xf < xc or F_cmd <= 0.0:
                vf = (vf + (F_cmd / mf) * dt) / (1.0 + (b / mf) * dt)
                vlim = MOTION['v_close'] if (xc - xf) < MOTION['gap_slow'] else MOTION['v_approach']
                vf = max(-MOTION['v_open'], min(vlim, vf))                  # speed limits of the finger drive
                xf += vf * dt
                if xf >= xc and F_cmd > 0.0:
                    N = self._clip(Fi, xc)
                    lim = lim or (N < Fi - 1e-9)
                    xf, vf = xc + hertz_delta(N, tr['Re'], tr['Es']) + self._defl(N, xc), 0.0
                elif xf < 0.0: xf, vf = 0.0, 0.0
                elif xf > self.xmax: xf, vf = self.xmax, 0.0
            else:
                N = self._clip(Fi, xc)
                lim = lim or (N < Fi - 1e-9)
                target = xc + hertz_delta(N, tr['Re'], tr['Es']) + self._defl(N, xc)
                xf_new = xf + (dt / (self.TAU_CONTACT + dt)) * (target - xf)
                vf = (xf_new - xf) / dt
                xf = xf_new

            self.xf[i], self.vf[i], self.N[i] = xf, vf, N
            if N > 0.0:
                n_pos += 1
                total_N += N
                defl = max(defl, self._defl(N, xc))

        self.torque_limited, self.deflect = lim, defl
        if n_pos > 0: self.psi_c = sum(self.psi_of(self.xf[i]) for i in range(self.n) if self.N[i] > 0.0) / n_pos

        if n_pos == 0 or total_N <= 1e-9:
            self.w = self.Ft = self.f_mean = 0.0
            self.sat = False
            if self.zo > 0.0 or self.vo > 0.0:
                self.vo -= G * dt
                self.zo += self.vo * dt
                if self.zo <= 0.0: self.zo = self.vo = 0.0
            return

        kt = self.KT * n_pos
        ct = 2.0 * self.ZETA * math.sqrt(kt * m)
        was_sliding = abs(self.vo - vg) >= 2e-4
        if n_pos >= self.n_act:      # full grasp: capacity from the planned contact layout (COM, tilt, force weights)
            gsum = sum(self.g)
            cap_N = self.fg * total_N / max(gsum, 1e-9)
        else:                        # partial contact while closing: every contact only supports its own normal share
            cap_N = 0.5 * sum(self.N[i] * self.cos_t[i] for i in range(self.n))
        fric_cap = (mu * self.KR if was_sliding else mu) * cap_N

        denom = m + dt * (kt * dt + ct)
        rhs_force = -kt * (self.w - vg * dt) + ct * vg - m * G
        vo_stick = (m * self.vo + dt * rhs_force) / denom
        w_stick = self.w + (vo_stick - vg) * dt
        Ft_trial = -kt * w_stick - ct * (vo_stick - vg)

        if abs(Ft_trial) <= fric_cap:
            self.sat, Ft, vo_next, w_next = False, Ft_trial, vo_stick, w_stick
        else:
            self.sat = True
            Ft = (1.0 if Ft_trial >= 0.0 else -1.0) * (mu * self.KR * cap_N)
            w_next = -Ft / kt
            vo_next = self.vo + ((Ft - m * G) / m) * dt

        zo_next = self.zo + vo_next * dt
        if zo_next <= 0.0 and vo_next <= 0.0:
            self.zo = self.vo = 0.0
            w_table = self.w - vg * dt
            Ft_table = -kt * w_table + ct * vg
            if abs(Ft_table) <= fric_cap:
                self.w, self.Ft, self.sat = w_table, Ft_table, False
            else:
                self.Ft = (1.0 if Ft_table >= 0.0 else -1.0) * fric_cap
                self.w, self.sat = -self.Ft / kt, True
        else:
            self.zo, self.vo, self.w, self.Ft = max(0.0, zo_next), vo_next, w_next, Ft

        self.Fg, self.f_mean = 0.0, abs(self.Ft) / n_pos

    def N_mean(self): return sum(self.N) / self.n_act
    def N_peak(self): return max(self.N)
    def sense(self, rng, bias):
        Nm = self.N_mean()
        sig = 0.015 * math.sqrt(2.0 / self.n_act)
        g = lambda: rng.gauss(0.0, sig)
        fzl, fzr = Nm + bias['zl'] + g(), -Nm + bias['zr'] + g()
        fyl, fyr = -self.f_mean + bias['yl'] + g(), -self.f_mean + bias['yr'] + g()
        d_sens = max(0.0, hertz_delta(Nm, self.tr['Re'], self.tr['Es']) + rng.gauss(0.0, 3e-6))
        return fzl, fzr, fyl, fyr, d_sens

class Episode:
    BIAS = dict(zl=0.05, zr=-0.03, yl=0.30, yr=0.31)
    LIFT_SPEED = MOTION['lift_speed']
    LIFT_ACCEL = 0.02          # m/s^2 smooth start / stop of the lift

    def __init__(self, obj_key, pad_key='small', strat_key='adaptive', seed=0, n=None):
        if n is not None: H.HAND['n_fingers'] = int(n)
        self.obj_key, self.pad_key, self.strat_key = obj_key, pad_key, strat_key
        self.o, pad = OBJECTS[obj_key], PADS[pad_key]
        self.plan = H.plan_grasp(self.o)
        self.n = self.plan['n']
        n_act = max(1, sum(self.plan['active']))
        if self.plan['ok']: MAT.GRASP_FG[obj_key] = self.plan['fric_gain']
        else: MAT.GRASP_FG.pop(obj_key, None)
        self.tr = truth(obj_key, pad_key, n=n_act)
        self.shape = get_shape(self.o)
        self.yaw = self.plan['yaw']
        self.yaw_cmd = 0.0
        self.k_tick = 0
        self.clear = dict(rigid=1.0, dome_pen=0.0, table=1.0, ff=1.0)
        self.min_rigid, self.max_dome_pen, self.min_ff, self.min_table = 1.0, 0.0, 1.0, 1.0
        self.collided = False
        self._trk_key = None                      # (hand height rel. to part, yaw) of the last contact-point update
        self.dome_gap = [0.0] * self.n            # signed gap dome surface -> part (m), refreshed by the clearance monitor
        self.rng = random.Random(seed)
        self.ctrl = GripController(Params(strategy=strat_key, F_max=GRIPPER['F_max'], R_obj=self.o['R'], R_pad=pad['R'], E_pad=pad['E'], nu_pad=pad['nu']))
        self.ctrl.n_f = n_act
        self.plant = HandPlant(self.o, self.tr, self.plan)
        self.tcp0 = tcp_home()
        self.min_clear = 1.0
        self.tcp_g = (0.0, self.plan['z_palm'])
        self.z_hover = max(self.plan['z_hover'], self.tcp0[1])
        d_raise = self.z_hover - self.tcp0[1]
        d_over = abs(self.tcp_g[0] - self.tcp0[0])
        d_down = max(self.z_hover - self.tcp_g[1], 0.0)
        pk = 1.875                                  # peak / mean speed of a min-jerk profile
        self.T_raise = max(MOTION['t_min_raise'], pk * d_raise / MOTION['v_transit']) if d_raise > 0.005 else 0.0
        self.T_over = max(MOTION['t_min_over'], pk * d_over / MOTION['v_transit'], pk * abs(self.yaw) / MOTION['w_yaw'])
        self.T_down = max(MOTION['t_min_down'], pk * d_down / MOTION['v_final'])
        self.tcp_speed = 0.0
        self._tcp_prev = self.tcp0
        self.lift_v = 0.0
        self.lift_held = False
        self._vrel = 0.0
        self.t, self.t_app0, self.t_lift0 = 0.0, None, None
        self.arm_stage = 'HOME'
        self.tcp_cmd = self.tcp0
        self.rise, self.at_grasp, self.at_top, self.crushed = 0.0, False, False, False
        self.peak_N, self.peak_vm, self.lifted_max, self.min_fos = 0.0, 0.0, 0.0, float('inf')
        self.ms = mos_state(0.0, 0.0, obj_key, pad_key)
        self.out, self.done = None, False
        self.t_hist, self.fz_hist, self.fy_hist, self.fd_hist = [], [], [], []
        self.p0_hist, self.vm_hist, self.tr_hist, self.rk_hist, self.eps_hist = [], [], [], [], []

    def _arm(self, out):
        vg = 0.0
        if self.arm_stage == 'HOME':
            self.tcp_cmd = self.tcp0
            if self.ctrl.state != 'CALIB':
                self.arm_stage, self.t_app0 = 'RAISE' if self.T_raise > 0 else 'OVER', self.t
        if self.arm_stage in ('RAISE', 'OVER', 'DESCEND'):
            tt = self.t - self.t_app0
            x0, z0 = self.tcp0
            xg, zp, zh = self.tcp_g[0], self.tcp_g[1], self.z_hover
            if tt < self.T_raise:
                self.arm_stage, self.tcp_cmd = 'RAISE', (x0, z0 + (zh - z0) * min_jerk(tt / self.T_raise))
            elif tt < self.T_raise + self.T_over:
                self.arm_stage, self.tcp_cmd = 'OVER', (x0 + (xg - x0) * min_jerk((tt - self.T_raise) / self.T_over), zh)
            elif tt < self.T_raise + self.T_over + self.T_down:
                self.arm_stage, self.tcp_cmd = 'DESCEND', (xg, zh + (zp - zh) * min_jerk((tt - self.T_raise - self.T_over) / self.T_down))
            else:
                self.tcp_cmd = self.tcp_g
                if tt >= self.T_raise + self.T_over + self.T_down + 0.2:
                    self.arm_stage, self.at_grasp = 'GRASP', True
        elif self.arm_stage == 'GRASP':
            self.tcp_cmd = self.tcp_g
            if out['lift_enable']: self.arm_stage, self.t_lift0 = 'LIFT', self.t
        elif self.arm_stage == 'LIFT':
            if out['lift_enable'] and not self.at_top:
                self._vrel += 0.08 * ((self.lift_v - self.plant.vo) - self._vrel)       # wrist load cell: hand rising, part not following
                nc = self._vrel > 0.004 and self.rise > 0.003
                self.ctrl.not_carried = nc
                self.lift_held = bool(out['ramp'] or out['s_c'] or nc)   # slip / not carried -> stop lifting until the grip is firm
                v_tgt = 0.0 if self.lift_held else self.LIFT_SPEED
                self.lift_v += max(-self.LIFT_ACCEL * 0.01, min(self.LIFT_ACCEL * 0.01, v_tgt - self.lift_v))
                vg = self.lift_v
                self.rise += vg * 0.01
                if self.rise >= GRIPPER['lift_height']: self.rise, self.at_top = GRIPPER['lift_height'], True
            self.tcp_cmd = (self.tcp_g[0], self.tcp_g[1] + self.rise)
        if self.t_app0 is not None:                                    # wrist yaw: turns while hovering, before the descent
            self.yaw_cmd = self.yaw * min_jerk((self.t - self.t_app0 - self.T_raise) / max(self.T_over, 1e-6))
        return vg

    def tick(self, dt=0.01, sub=10):
        pl = self.plant
        fzl, fzr, fyl, fyr, d_sens = pl.sense(self.rng, self.BIAS)
        self.ctrl.psi = 0.0
        out = self.ctrl.step(self.t, fzl, fzr, fyl, fyr, pl.mean_xf(), d_sens, self.at_grasp, self.at_top)
        vg = self._arm(out)
        self._track_contacts()
        if self.ctrl.state in ('CALIB', 'WAIT_ARM'):
            for i in range(self.n):
                pl.xf[i] = float(np.interp(H.preshape_curl(self.tcp_cmd[1], self.plan['s_clear'][i]), pl.ts, pl.tx)) if pl.active[i] else 0.0
                pl.vf[i] = 0.0
        self.min_clear = min(self.min_clear, min(self.tcp_cmd[1] + H.lowest_z(s) for s in pl.s_list()))
        sub_dt = dt / sub
        for _ in range(sub):
            pl.step(out['force'], vg, sub_dt)
            self.peak_N, self.lifted_max = max(self.peak_N, max(pl.N)), max(self.lifted_max, pl.zo)
        self._check_clearance(pl)
        ms = mos_state(pl.N_peak(), pl.f_mean, self.obj_key, self.pad_key)      # most loaded pad governs the stress
        if ms['fos'] < 1.0 and self.ctrl.state not in ('CALIB', 'WAIT_ARM'): self.crushed = True
        self.min_fos, self.peak_vm = min(self.min_fos, ms['fos']), max(self.peak_vm, ms['sig_vm'])
        self.tcp_speed = math.hypot(self.tcp_cmd[0] - self._tcp_prev[0], self.tcp_cmd[1] - self._tcp_prev[1]) / dt
        self._tcp_prev = self.tcp_cmd
        self.ms, self.out = ms, out
        self.t_hist.append(self.t); self.fz_hist.append(out['fz']); self.fy_hist.append(out['fy']); self.fd_hist.append(out['fd'])
        self.p0_hist.append(ms['p0'] / 1e3); self.vm_hist.append(ms['sig_vm'] / 1e3); self.tr_hist.append(ms['sig_tresca'] / 1e3)
        self.rk_hist.append(ms['sig_rankine'] / 1e3); self.eps_hist.append(ms['eps_c'] * 100.0)
        self.t += dt
        self.done = bool(out['done']) or self.t >= 90.0
        return out

    def _track_contacts(self):
        """Keep the contact point of every touching finger ON the part surface.  The plant used to fix the contact travel at the value
        planned for the initial hand height; when the pads slid up a curved part (egg, apple, any CAD curve) during squeeze / lift the
        surface moved away from the fingertips and the display showed 1-5 mm gaps.  The hand height relative to the part is known
        (arm command minus part rise), so the contact travel is re-solved on the real geometry whenever it changed by > 0.15 mm."""
        if not self.plan['ok'] or self.arm_stage not in ('GRASP', 'LIFT'): return
        pl = self.plant
        z_rel = self.tcp_cmd[1] - pl.zo
        key = (round(z_rel / 1.5e-4), round(self.yaw_cmd, 3))
        if key == self._trk_key: return
        self._trk_key = key
        lay = H.layout(self.n)
        cand = dict(phi=self.plan['phis'], yaw=self.yaw_cmd)
        s_c, ok = H._find_contacts(self.shape, lay, [cand], z_rel, H.HAND['tip_r'])
        idx = [i for i in range(self.n) if pl.active[i] and ok[0, i]]
        if not idx: return
        s = np.array([s_c[0, i] for i in idx])
        sl = H.s_limit()
        for _ in range(3):                                           # secant polish of  sdf(dome(s)) = tip radius  (the table is coarse)
            pts = np.array([H._dome_world(lay, cand, z_rel, i, float(si)) for i, si in zip(idx, s)])
            f = self.shape.sdf(pts) - H.HAND['tip_r']
            pts2 = np.array([H._dome_world(lay, cand, z_rel, i, float(min(si + 2e-3, sl))) for i, si in zip(idx, s)])
            df = (self.shape.sdf(pts2) - H.HAND['tip_r'] - f) / 2e-3
            step = np.where(np.abs(df) > 1e-4, -f / np.where(np.abs(df) > 1e-4, df, 1.0), 0.0)
            s = np.clip(s + np.clip(step, -0.05, 0.05), 0.0, sl)
        pts = np.array([H._dome_world(lay, cand, z_rel, i, float(si)) for i, si in zip(idx, s)])
        res = np.abs(self.shape.sdf(pts) - H.HAND['tip_r'])
        for i, si, r in zip(idx, s, res):
            if r < 3e-4:                                             # only follow a surface that was really found; otherwise keep the planned travel
                pl.set_contact_travel(i, float(np.interp(si, pl.ts, pl.tx)))

    def _check_clearance(self, pl):
        """3-D clearance of the whole hand against the real part geometry (signed distance), a few times per second of sim."""
        self.k_tick += 1
        if self.k_tick % 3 or not self.plan['ok']: return
        geoms = H.hand_geometry(self.plan['bases'], self.plan['phis'], pl.s_list())
        full = (self.k_tick % 9 == 0)
        c = H.pose_clearance(self.shape, geoms, self.yaw_cmd, self.tcp_cmd[1] - pl.zo, with_palm=True) if full else None
        if c is None:
            c = H.pose_clearance_fast(self.shape, geoms, self.yaw_cmd, self.tcp_cmd[1] - pl.zo)
        pen = 0.0
        for i in range(self.n):                                          # soft dome may sink by the pad's own deformation
            if not pl.active[i]: continue
            allow = (hertz_delta(pl.N[i], self.tr['Re'], self.tr['Es']) + pl._defl(pl.N[i], pl.xc[i]) if pl.N[i] > 0 else 0.0) + 0.0012
            pen = max(pen, -float(c['dome'][i]) - allow)
        self.dome_gap = [float(x) for x in c['dome']]
        self.clear = dict(rigid=c['rigid'], dome_pen=pen, table=c['table'], ff=c['ff'] if c['ff'] is not None else self.clear['ff'])
        if self.ctrl.state in ('CALIB',): return
        self.min_rigid = min(self.min_rigid, c['rigid'])
        self.max_dome_pen = max(self.max_dome_pen, pen)
        self.min_table = min(self.min_table, c['table'])
        if c['ff'] is not None: self.min_ff = min(self.min_ff, c['ff'])
        if c['rigid'] < -0.0006 or pen > 0.0015 or c['table'] < -0.0006 or (c['ff'] is not None and c['ff'] < -0.001): self.collided = True

    def verdict(self):
        H_ = GRIPPER['lift_height']
        frac = self.plant.zo / H_
        if self.crushed: return 'CRUSHED'
        if self.collided: return 'COLLISION'
        if self.out and self.out['abort']: return self.out['hint'] or 'ABORT'
        if frac >= 0.5: return 'OK'
        if self.lifted_max / H_ >= 0.2: return 'SLIPPED OFF'
        return 'NOT LIFTED'

def run_headless(obj_key, pad_key='small', strat_key='adaptive', seed=0, n=None):
    ep = Episode(obj_key, pad_key, strat_key, seed, n)
    while not ep.done: ep.tick()
    return ep

def main(args=None):
    import sys
    from .materials import OBJECTS as OBJ
    ns = [int(sys.argv[1])] if len(sys.argv) > 1 else [4, 5]
    for n in ns:
        print('--- %d-finger hand, small pad ---' % n)
        print('%-13s %-12s %7s %8s %8s %8s %8s %9s' % ('object', 'adaptive', 'peak N', 'N_slip', 'N_max', 'm_est g', 'fixed25', 'table clr'))
        for k in OBJ:
            try:
                from .physics_check import verified_plan
                H.HAND['n_fingers'] = n
                verified_plan(k, n)
            except Exception as exc:
                print('physics check skipped:', exc)
            if not H.plan_grasp(OBJ[k])['ok']:
                print('%-13s no feasible grasp: %s' % (k, H.plan_grasp(OBJ[k])['msgs'][0][:80])); continue
            a, f = run_headless(k, 'small', 'adaptive', 0, n), run_headless(k, 'small', 'fixed', 0, n)
            print('%-13s %-12s %7.2f %8.3f %8.1f %8.0f %8s %6.1f mm' % (
                k, a.verdict(), a.peak_N, a.tr['N_slip'], a.tr['N_max'], a.out['m_est'] * 1000, f.verdict(), a.min_clear * 1000))

if __name__ == '__main__':
    main()
GRIPEOF

mkdir -p "$PKG/$(dirname grip_gz/cad.py)"
cat << 'GRIPEOF' > "$PKG/grip_gz/cad.py"
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

    # ---- 2. orientation: up axis, then turn the narrowest width onto the finger axis (y) -------------------------------------------------
    up = str(s.get('up') or 'auto').lower().replace('+', '')
    if up == 'auto':
        up, why = MP.detect_up(V, F)
        notes.append('Orientation (auto): ' + why + ('' if up == 'z' else '.  Override with --up z|y|-y|x|-x|-z.'))
    elif up not in ('z', 'y', '-y', 'x', '-x', '-z'): raise CadError('Unknown up axis %r (use auto, z, y, -y, x, -x, -z).' % up)
    else: notes.append('Orientation: %s axis up (as requested).' % up)
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
    MAXDIM = 0.30                                                       # largest dimension the arm / hand workspace is sized for
    if fit and big < 0.012: kfit = 0.040 / big
    elif fit and big > MAXDIM: kfit = MAXDIM / big
    if kfit > 1.0:
        warn.append('Part was only %.2f mm in size - auto-scaled x%.3g to %.0f mm. If that is wrong, pick the right UNIT (m / cm / inch) or pass --scale.' % (big * 1000, kfit, big * kfit * 1000))
    elif kfit < 1.0: warn.append('Largest dimension was %.0f mm (limit %.0f mm): scaled down.' % (big * 1000, MAXDIM * 1000))
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
    if wmin < 0.004: raise CadError('Part too thin at grasp height (%.1f mm): the fingertips cannot pinch it. Try another UP axis, a larger scale, or a different part.' % (wmin * 1000))
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
        warn.insert(0, 'SCALE: %s - the part is simulated at %.4g x its original size (%.1f x %.1f x %.1f mm -> %.1f x %.1f x %.1f mm).' % (('dimensions of the STL were REDUCED %.3g times' % (1.0 / ktot)) if ktot < 1 else ('dimensions of the STL were ENLARGED %.3g times' % ktot), ktot, *(1000 * ext0), *(1000 * ext)))
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
    rep = dict(key=key, name=key[4:], file=path, unit=unit, material=str(s.get('material') or 'aluminium').lower(), kind=kind, volume=vol, mass=mass, density=mass / vol, extents=tuple(float(x) for x in ext), height=h, width=gg['w'], grasp_z=zg, com=tuple(float(x) for x in com), R_contact=R_obj, mu=ov('mu', mat['mu']), E=ov('E', mat['E']), nu=ov('nu', mat['nu']), p_allow=p_allow, F_allow=F_allow, strength=ov('strength', mat['strength']), watertight=wt, faces=int(len(F)), mesh_out=os.path.join(CAD_DIR, key + '.stl'), warnings=warn, up=up, scale=ktot, orig_extents=tuple(float(x) for x in ext0))
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
GRIPEOF

mkdir -p "$PKG/$(dirname grip_gz/meshprep.py)"
cat << 'GRIPEOF' > "$PKG/grip_gz/meshprep.py"
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
GRIPEOF

mkdir -p "$PKG/$(dirname grip_gz/gripper_models.py)"
cat << 'GRIPEOF' > "$PKG/grip_gz/gripper_models.py"
"""gripper_models.py - Procedural multi-material STL mesh generator for the arm and n-finger hand."""
import math
import os
import struct
import numpy as np

from . import hand as H
from .arm_ik import ARM

MESH_DIR = os.path.expanduser('~/.grip_gz_cad/gripper')
COLORS = {'steel': (0.62, 0.65, 0.70, 1.0), 'dark': (0.20, 0.21, 0.25, 1.0), 'hub': (0.36, 0.38, 0.43, 1.0), 'light': (0.80, 0.82, 0.85, 1.0), 'accent': (0.95, 0.50, 0.10, 1.0), 'rubber': (0.12, 0.12, 0.13, 1.0), 'bolt': (0.85, 0.86, 0.88, 1.0), 'black': (0.17, 0.18, 0.21, 1.0), 'white': (0.96, 0.96, 0.94, 1.0), 'charcoal': (0.30, 0.32, 0.36, 1.0)}

def _cat(parts):
    Vs, Fs, off = [], [], 0
    for V, F in parts: Vs.append(V); Fs.append(F + off); off += len(V)
    return np.vstack(Vs), np.vstack(Fs)

def box(cx, cy, cz, sx, sy, sz):
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    V = np.array([[cx + a * hx, cy + b * hy, cz + c * hz] for c in (-1, 1) for b in (-1, 1) for a in (-1, 1)], dtype=np.float64)
    F = np.array([(0, 2, 1), (1, 2, 3), (4, 5, 6), (5, 7, 6), (0, 1, 4), (1, 5, 4), (2, 6, 3), (3, 6, 7), (0, 4, 2), (2, 4, 6), (1, 3, 5), (3, 7, 5)], dtype=np.int64)
    return V, F

def cyl(cx, cy, cz, r, length, axis='y', n=96):
    ang = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    V = np.vstack([np.c_[r * np.cos(ang), r * np.sin(ang), np.full(n, -length / 2.0)], np.c_[r * np.cos(ang), r * np.sin(ang), np.full(n, length / 2.0)], [[0, 0, -length / 2.0]], [[0, 0, length / 2.0]]])
    F = np.array([(i, (i + 1) % n, n + (i + 1) % n) for i in range(n)] + [(i, n + (i + 1) % n, n + i) for i in range(n)] + [(2 * n + 1, n + i, n + (i + 1) % n) for i in range(n)] + [(2 * n, (i + 1) % n, i) for i in range(n)], dtype=np.int64)
    if axis == 'y': V = np.c_[V[:, 0], V[:, 2], -V[:, 1]]
    elif axis == 'x': V = np.c_[V[:, 2], V[:, 1], -V[:, 0]]
    return V + np.array([cx, cy, cz]), F

def ellipsoid(cx, cy, cz, rx, ry, rz, nu=96, nv=48):
    V = [[0, 0, 1]] + [[math.sin(math.pi * i / nv) * math.cos(2.0 * math.pi * j / nu), math.sin(math.pi * i / nv) * math.sin(2.0 * math.pi * j / nu), math.cos(math.pi * i / nv)] for i in range(1, nv) for j in range(nu)] + [[0, 0, -1]]
    F = [(0, 1 + j, 1 + (j + 1) % nu) for j in range(nu)] + [(1 + i * nu + j, 1 + (i + 1) * nu + j, 1 + i * nu + (j + 1) % nu) for i in range(nv - 2) for j in range(nu)] + [(1 + i * nu + (j + 1) % nu, 1 + (i + 1) * nu + j, 1 + (i + 1) * nu + (j + 1) % nu) for i in range(nv - 2) for j in range(nu)] + [(len(V) - 1, 1 + (nv - 2) * nu + (j + 1) % nu, 1 + (nv - 2) * nu + j) for j in range(nu)]
    return np.array(V) * np.array([rx, ry, rz]) + np.array([cx, cy, cz]), np.array(F, dtype=np.int64)

def rotz(V, F, phi): return np.c_[math.cos(phi) * V[:, 0] - math.sin(phi) * V[:, 1], math.sin(phi) * V[:, 0] + math.cos(phi) * V[:, 1], V[:, 2]], F

def screws_y(cx, cz, ring_r, y_face, count=5, r=0.0016, h=0.0015):
    return _cat([cyl(cx + ring_r * math.cos(2.0 * math.pi * k / count), y_face, cz + ring_r * math.sin(2.0 * math.pi * k / count), r, h, 'y', 24) for k in range(count)])

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
    return {'charcoal': _cat([_cat([box(0, 0, 0.010, 0.22, 0.22, 0.020)] + [cyl(sx * 0.09, sy * 0.09, 0.023, 0.008, 0.006, 'z', 32) for sx in (-1, 1) for sy in (-1, 1)]), _cat([cyl(0, 0, base_h, 0.052, 0.090, 'y')])]), 'black': _cat([box(0, 0, 0.020 + (base_h - 0.020) / 2.0, 0.12, 0.12, base_h - 0.020), box(0, 0, 0.060, 0.15, 0.15, 0.030)]), 'bolt': _cat([screws_y(0, base_h, 0.036, 0.0465, 6), screws_y(0, base_h, 0.036, -0.0465, 6)])}

def part_link1(L):
    return {'black': _cat([box(L / 2.0, 0, 0, L - 0.04, 0.040, 0.056), box(L / 2.0, 0, 0.0305, L - 0.10, 0.030, 0.005)]), 'charcoal': _cat([cyl(0, 0, 0, 0.040, 0.066, 'y'), cyl(L, 0, 0, 0.040, 0.066, 'y')]), 'bolt': _cat([screws_y(0, 0, 0.026, 0.0345, 5), screws_y(L, 0, 0.026, 0.0345, 5), screws_y(L, 0, 0.026, -0.0345, 5)]), 'white': cyl(0, 0, 0, 0.020, 0.080, 'y')}

def part_link2(L):
    return {'black': _cat([box(L / 2.0, 0, 0, L - 0.04, 0.040, 0.050), box(L / 2.0, 0, 0.0275, L - 0.10, 0.030, 0.005)]), 'charcoal': _cat([cyl(0, 0, 0, 0.037, 0.092, 'y'), cyl(L, 0, 0, 0.032, 0.070, 'y')]), 'bolt': _cat([screws_y(0, 0, 0.024, 0.0475, 5), screws_y(0, 0, 0.024, -0.0475, 5), screws_y(L, 0, 0.020, 0.0365, 5)]), 'white': cyl(L, 0, 0, 0.016, 0.090, 'y')}

def part_palm(n, lay):
    """Hand body in the HAND frame (z = 0 is the finger-base plane, i.e. the palm underside); the finger sockets sit at the layout bases."""
    hx, hy, th = H.HAND['palm_x'], H.HAND['palm_y'], H.HAND['palm_thick']
    sockets, bolts = [], []
    for f in lay:
        bx, by = f['base']
        sockets.append(cyl(bx, by, 0.0, 0.0155, 0.036, 'y', 28) if False else box(bx, by, 0.004, 0.034, 0.040, 0.016))
        bolts.append(cyl(bx, by, 0.0125, 0.0045, 0.003, 'z', 32))
    return {'black': _cat([box(0, 0, th / 2.0, 2 * hx, 2 * hy, th)]), 'charcoal': _cat([cyl(0, 0, th + 0.020, 0.032, 0.040, 'z', 96), box(0, 0, th + 0.002, 2 * hx - 0.012, 2 * hy - 0.012, 0.004)] + sockets), 'bolt': _cat(bolts), 'white': box(0, 0, th + 0.0045, 0.020, 0.006, 0.003)}

def part_phalanx(k, L):
    hub_len = (0.034, 0.032, 0.030)[k - 1]
    bolts = _cat([screws_y(0, 0, 0.0075, hub_len / 2.0 + 0.0006, 5), screws_y(0, 0, 0.0075, -hub_len / 2.0 - 0.0006, 5)])
    hub = cyl(0, 0, 0, (0.0145, 0.0135, 0.0125)[k - 1], hub_len, 'y', 96)
    if k < 3: return {'black': _cat([box(0.5 * (0.010 + L - 0.010), 0, 0, L - 0.020, 0.020, 0.022), box(L - 0.012, 0, 0, 0.012, 0.024, 0.026)]), 'charcoal': hub, 'bolt': bolts}
    R, cx = H.HAND['tip_r'], L - H.HAND['pad_back']
    return {'black': box(0.5 * (0.010 + cx - 0.012), 0, 0, cx - 0.022, 0.019, 0.022), 'white': ellipsoid(cx, 0, 0, R, R, R, 96, 48), 'charcoal': hub, 'bolt': bolts}

def ensure_gripper_meshes(z0=0.080, n=None):
    n = int(n or H.HAND['n_fingers'])
    H.HAND['n_fingers'] = n
    os.makedirs(MESH_DIR, exist_ok=True)
    raw_parts = {'base': ('base', part_base(ARM['zs'] + z0)), 'link1': ('link1', part_link1(ARM['L1'])), 'link2': ('link2', part_link2(ARM['L2'])), 'palm': ('palm', part_palm(n, H.layout(n))), 'phalanx1': ('phalanx1', part_phalanx(1, H.HAND['L'][0])), 'phalanx2': ('phalanx2', part_phalanx(2, H.HAND['L'][1])), 'phalanx3': ('phalanx3', part_phalanx(3, H.HAND['L'][2]))}
    mesh_map = {}
    for key, (uname, groups) in raw_parts.items():
        entries = []
        for cname, (V, F) in groups.items():
            stl_name = 'grip3_%s_%s_n%d.stl' % (key, cname, n) if key == 'palm' else 'grip3_%s_%s.stl' % (key, cname)
            fpath = os.path.join(MESH_DIR, stl_name)
            write_stl(fpath, V, F)
            entries.append((cname, fpath))
        mesh_map[key] = entries
    return mesh_map
GRIPEOF

mkdir -p "$PKG/$(dirname grip_gz/world.py)"
cat << 'GRIPEOF' > "$PKG/grip_gz/world.py"
"""world.py - MuJoCo MJCF Scene & Articulated Hand-Arm Hierarchy Builder."""
import math
import os

from . import gripper_models
from . import hand as H
from .arm_ik import ARM
from .materials import OBJECTS, height, width

WORLD = 'grip_world'
TABLE_TOP_Z = 0.080

def _f(x): return '%.6g' % x
def _rgba_str(rgba): return ' '.join('%.4f' % v for v in rgba)
def _quat_zy(phi, beta):
    cz, sz, cy, sy = math.cos(phi / 2.0), math.sin(phi / 2.0), math.cos(beta / 2.0), math.sin(beta / 2.0)
    return '%s %s %s %s' % (_f(cz * cy), _f(-sz * sy), _f(cz * sy), _f(cy * sz))

def _obj_geom_xml(gname, o, crushed=False, mesh_asset_name=None):
    h, w, c_str = height(o), width(o), _rgba_str((1.0, 0.08, 0.08, 1.0) if crushed else o['rgba'])
    if o.get('shape') == 'mesh' and mesh_asset_name and o.get('mesh_ok', True): return '<geom name="%s" type="mesh" mesh="%s" rgba="%s" contype="0" conaffinity="0"/>' % (gname, mesh_asset_name, c_str)
    if o.get('shape') == 'mesh': return '<geom name="%s" type="cylinder" size="%s %s" rgba="%s" contype="0" conaffinity="0"/>' % (gname, _f(w * (0.68 if crushed else 0.5)), _f(h * (0.46 if crushed else 0.5)), c_str)
    if crushed: return '<geom name="%s" type="box" size="%s %s %s" rgba="%s" contype="0" conaffinity="0"/>' % (gname, _f(w * 0.59), _f(w * 0.34), _f(h * 0.46), c_str)
    if o['vis'] == 'sphere': return '<geom name="%s" type="sphere" size="%s" rgba="%s" contype="0" conaffinity="0"/>' % (gname, _f(o['dims'][0]), c_str)
    if o['shape'] == 'box': return '<geom name="%s" type="box" size="%s %s %s" rgba="%s" contype="0" conaffinity="0"/>' % (gname, _f(o['dims'][0] / 2.0), _f(o['dims'][1] / 2.0), _f(o['dims'][2] / 2.0), c_str)
    return '<geom name="%s" type="cylinder" size="%s %s" rgba="%s" contype="0" conaffinity="0"/>' % (gname, _f(o['dims'][0]), _f(o['dims'][1] / 2.0), c_str)

def build_mjcf(n_fingers=None, z0=TABLE_TOP_Z):
    n = int(n_fingers or H.HAND['n_fingers'])
    H.HAND['n_fingers'] = n
    lay = H.layout(n)
    mesh_map = gripper_models.ensure_gripper_meshes(z0, n)
    xs, zs, L1, L2, L_p = ARM['xs'], ARM['zs'], ARM['L1'], ARM['L2'], H.HAND['L']

    xml = []
    a = xml.append
    a('<mujoco model="grip_mos_lab"><compiler angle="radian" coordinate="local"/>')
    a('<statistic extent="0.8" center="0 0 0.12"/>')
    a('<option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast"/>')
    a('<visual><headlight ambient="0.42 0.44 0.48" diffuse="0.60 0.62 0.66" specular="0.25 0.25 0.28"/><quality shadowsize="4096" offsamples="4" numslices="96" numstacks="48" numquads="24"/><map znear="0.005" zfar="20.0" shadowclip="1.6" shadowscale="0.55" haze="0.25" force="0.0"/><rgba haze="0.10 0.13 0.19 1"/><global offwidth="1920" offheight="1080" fovy="38" elevation="-18" azimuth="132"/></visual>')
    
    a('<asset>')
    a('<texture name="sky_tex" type="skybox" builtin="gradient" rgb1="0.36 0.50 0.70" rgb2="0.02 0.03 0.06" width="512" height="3072"/>')
    a('<texture name="wood_tex" type="2d" builtin="flat" width="1024" height="1024" rgb1="0.60 0.43 0.27" rgb2="0.52 0.37 0.23" mark="random" markrgb="0.38 0.26 0.15" random="0.03"/>')
    a('<texture name="grid_tex" type="2d" builtin="checker" width="1024" height="1024" rgb1="0.11 0.13 0.17" rgb2="0.14 0.17 0.22" mark="edge" markrgb="0.20 0.24 0.30"/>')
    a('<material name="floor_mat" texture="grid_tex" texrepeat="8 8" reflectance="0.12" shininess="0.4" specular="0.3"/>')
    a('<material name="bench_mat" texture="wood_tex" texuniform="true" texrepeat="3 3" rgba="1 1 1 1" reflectance="0.08" shininess="0.6" specular="0.4"/>')
    a('<material name="table_leg_mat" rgba="0.16 0.18 0.22 1" reflectance="0.20" shininess="0.7" specular="0.6"/>')
    a('<material name="fixture_mat" rgba="0.12 0.14 0.18 1" reflectance="0.25" shininess="0.8" specular="0.7"/>')
    for cname, rgba in gripper_models.COLORS.items(): a('<material name="mat_%s" rgba="%s" shininess="0.65" specular="0.45"/>' % (cname, _rgba_str(rgba)))
    for key, entries in mesh_map.items():
        for cname, fpath in entries: a('<mesh name="mesh_%s_%s" file="%s"/>' % (key, cname, fpath))
    for obj_key, o in OBJECTS.items():
        if o.get('shape') == 'mesh' and o.get('mesh') and os.path.isfile(o['mesh']) and o.get('mesh_ok', True):
            a('<mesh name="cad_%s_intact" file="%s" scale="1 1 1"/>' % (obj_key, o['mesh']))
            a('<mesh name="cad_%s_crushed" file="%s" scale="1.18 0.68 0.92"/>' % (obj_key, o['mesh']))
    a('</asset>')

    a('<worldbody>')
    a('<light name="key_light" directional="true" castshadow="true" pos="-0.4 -0.8 2.6" dir="0.30 0.45 -0.84" diffuse="0.88 0.90 0.95" specular="0.35 0.35 0.40"/>')
    a('<light name="fill_light" directional="false" castshadow="false" pos="1.0 1.2 2.0" dir="-0.4 -0.5 -0.75" diffuse="0.36 0.42 0.52" specular="0.10 0.10 0.12"/>')
    a('<light name="gantry_spot" directional="false" castshadow="false" pos="0 0 0.78" dir="0 0 -1" diffuse="0.35 0.40 0.48" specular="0.2 0.2 0.25"/>')
    a('<geom name="lab_floor" type="plane" pos="0 0 -0.42" size="4 4 0.05" material="floor_mat" contype="0" conaffinity="0"/>')
    
    a('<body name="table_pedestal" pos="0 0 0">')
    a('<geom name="bench_top" type="box" pos="-0.18 0 0.048" size="0.59 0.41 0.028" material="bench_mat" contype="0" conaffinity="0"/>')
    # table legs (floor at z=-0.42 up to underside of the top at z=0.020), inset from the corners
    for lx in (-0.72, 0.36):
        for ly in (-0.36, 0.36):
            a('<geom name="table_leg_%s_%s" type="box" pos="%s %s -0.20" size="0.030 0.030 0.220" material="table_leg_mat" contype="0" conaffinity="0"/>' % ('p' if lx > 0 else 'n', 'p' if ly > 0 else 'n', _f(lx), _f(ly)))
    # apron rails just under the top, plus low stretchers for a sturdy table look
    for ly in (-0.36, 0.36):
        a('<geom name="apron_x_%s" type="box" pos="-0.18 %s -0.012" size="0.540 0.012 0.032" material="table_leg_mat" contype="0" conaffinity="0"/>' % ('p' if ly > 0 else 'n', _f(ly)))
        a('<geom name="stretcher_x_%s" type="box" pos="-0.18 %s -0.30" size="0.540 0.010 0.014" material="table_leg_mat" contype="0" conaffinity="0"/>' % ('p' if ly > 0 else 'n', _f(ly)))
    for lx in (-0.72, 0.36):
        a('<geom name="apron_y_%s" type="box" pos="%s 0 -0.012" size="0.012 0.330 0.032" material="table_leg_mat" contype="0" conaffinity="0"/>' % ('p' if lx > 0 else 'n', _f(lx)))
        a('<geom name="stretcher_y_%s" type="box" pos="%s 0 -0.30" size="0.010 0.330 0.014" material="table_leg_mat" contype="0" conaffinity="0"/>' % ('p' if lx > 0 else 'n', _f(lx)))
    a('<geom name="fixture_plate" type="cylinder" pos="0 0 0.0775" size="0.16 0.002" material="fixture_mat" contype="0" conaffinity="0"/>')
    a('</body>')

    a('<body name="grip_arm_base" pos="%s 0 0">' % _f(xs))
    for cname, _ in mesh_map['base']: a('<geom type="mesh" mesh="mesh_base_%s" material="mat_%s" contype="0" conaffinity="0"/>' % (cname, cname))
    a('  <body name="grip_arm_link1" pos="0 0 %s">' % _f(zs + z0))
    a('    <joint name="shoulder" type="hinge" axis="0 1 0" limited="false"/>')
    for cname, _ in mesh_map['link1']: a('    <geom type="mesh" mesh="mesh_link1_%s" material="mat_%s" contype="0" conaffinity="0"/>' % (cname, cname))
    a('    <body name="grip_arm_link2" pos="%s 0 0">' % _f(L1))
    a('      <joint name="elbow" type="hinge" axis="0 1 0" limited="false"/>')
    for cname, _ in mesh_map['link2']: a('      <geom type="mesh" mesh="mesh_link2_%s" material="mat_%s" contype="0" conaffinity="0"/>' % (cname, cname))
    a('      <body name="grip_arm_palm" pos="%s 0 0">' % _f(L2))
    a('        <joint name="wrist" type="hinge" axis="0 1 0" limited="false"/>')
    a('        <inertial pos="0 0 0" mass="0.05" diaginertia="1e-5 1e-5 1e-5"/>')
    a('        <body name="grip_hand" pos="0 0 %s">' % _f(-ARM['tcp_drop']))
    a('          <joint name="yaw" type="hinge" axis="0 0 1" limited="false"/>')
    for cname, _ in mesh_map['palm']: a('          <geom type="mesh" mesh="mesh_palm_%s" material="mat_%s" contype="0" conaffinity="0"/>' % (cname, cname))
    for i, f in enumerate(lay):
        bx, by = f['base']
        cz, sz = math.cos(f['phi0'] / 2.0), math.sin(f['phi0'] / 2.0)
        a('          <body name="grip_f%d_sp" pos="%s %s 0" quat="%s 0 0 %s">' % (i, _f(bx), _f(by), _f(cz), _f(sz)))
        a('            <joint name="f%d_spread" type="hinge" axis="0 0 1" limited="false"/>' % i)
        a('            <inertial pos="0 0 0" mass="0.02" diaginertia="1e-6 1e-6 1e-6"/>')
        a('            <body name="grip_f%d_p1" pos="0 0 0" quat="%s 0 %s 0">' % (i, _f(math.cos(math.pi / 4.0)), _f(math.sin(math.pi / 4.0))))
        a('          <joint name="f%d_mcp" type="hinge" axis="0 1 0" limited="false"/>' % i)
        for cname, _ in mesh_map['phalanx1']: a('          <geom type="mesh" mesh="mesh_phalanx1_%s" material="mat_%s" contype="0" conaffinity="0"/>' % (cname, cname))
        a('          <body name="grip_f%d_p2" pos="%s 0 0">' % (i, _f(L_p[0])))
        a('            <joint name="f%d_pip" type="hinge" axis="0 1 0" limited="false"/>' % i)
        for cname, _ in mesh_map['phalanx2']: a('            <geom type="mesh" mesh="mesh_phalanx2_%s" material="mat_%s" contype="0" conaffinity="0"/>' % (cname, cname))
        a('            <body name="grip_f%d_p3" pos="%s 0 0">' % (i, _f(L_p[1])))
        a('              <joint name="f%d_dip" type="hinge" axis="0 1 0" limited="false"/>' % i)
        for cname, _ in mesh_map['phalanx3']: a('              <geom type="mesh" mesh="mesh_phalanx3_%s" material="mat_%s" contype="0" conaffinity="0"/>' % (cname, cname))
        a('            </body>')
        a('          </body>')
        a('        </body>')
        a('        </body>')
    a('        </body>')
    a('      </body>')
    a('    </body>')
    a('  </body>')
    a('</body>')

    for obj_key, o in OBJECTS.items():
        m_i = ('cad_%s_intact' % obj_key) if o.get('shape') == 'mesh' and o.get('mesh_ok', True) else None
        m_c = ('cad_%s_crushed' % obj_key) if o.get('shape') == 'mesh' and o.get('mesh_ok', True) else None
        a('<body name="obj_%s" mocap="true" pos="0 0 -5.0">' % obj_key + _obj_geom_xml('geom_%s' % obj_key, o, False, m_i) + '</body>')
        a('<body name="crush_%s" mocap="true" pos="0 0 -5.0">' % obj_key + _obj_geom_xml('geom_crush_%s' % obj_key, o, True, m_c) + '</body>')

    a('</worldbody></mujoco>')
    return '\n'.join(xml)
GRIPEOF

mkdir -p "$PKG/$(dirname grip_gz/dashboard.py)"
cat << 'GRIPEOF' > "$PKG/grip_gz/nptel_priority.py"
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


def _txt(ax, s, C, fs=6.9, col=None):
    ax.text(0.0, 1.0, s, va='top', ha='left', family='monospace', fontsize=fs, color=col or C['fg'], transform=ax.transAxes)


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
    ax.text(tn / 2, -0.12, f'sigma_n {sn / sc:.3f}', color=COL['a'], fontsize=7, ha='center')
    ax.text(tn, tt / 2, f' tau {tau / sc:.3f}', color=COL['c'], fontsize=7)
    ax.text(tn / 2, tt / 2 + 0.1, f't = S n  ({np.linalg.norm(R["t"]) / sc:.3f})', color=COL['d'], fontsize=7)
    ax.set_xlim(-1.35, 1.35); ax.set_ylim(-1.2, 1.25); ax.set_aspect('equal')
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
    ax.text(0, -0.05, f"point (x, y, z) = ({E_['pt'][0]:.2f}, {E_['pt'][1]:.2f}, {E_['pt'][2]:.2f}) a\nsigma_xx {S0[0, 0] / sc:.3f}   sigma_zz {S0[2, 2] / sc:.3f}\ntau_xz {S0[0, 2] / sc:.3f}   sigma_yy {S0[1, 1] / sc:.3f}  [{un}]",
            color=C['fg'], fontsize=7, ha='center', va='center')
    ax.set_xlim(-1.6, 1.6); ax.set_ylim(-1.3, 1.3)
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
    _txt(ax, '\n'.join(lines), C, 6.8)
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
    ax.text(L * 0.5, 0.012, f"F axial {S['F']:.1f} N", color=COL['e'], fontsize=7, ha='center')
    ax.text(0, 0.045, f"M_root {S['M']:.2f} N.m   V {S['V']:.1f} N   T {S['T'] * 1e3:.0f} N.mm", color=C['fg'], fontsize=7)
    ax.text(0, 0.065, f"OD {2e3 * S['ro']:.0f}/ID {2e3 * S['ri']:.0f} mm, L {1e3 * L:.0f} mm", color=C['fg'], fontsize=7)
    ax.set_xlim(-0.03, L + ov_o + 0.06); ax.set_ylim(-0.12, 0.08)
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
    ax.text(0.02, 0.04, f"sigma_max = M c / I = {bc['sig_root'] / 1e6:.3f} MPa\ntau_max = {s0['tauf']:.2f} V/A = {bc['tau_root'] / 1e6:.3f} MPa",
            transform=ax.transAxes, color=C['fg'], fontsize=7)
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
    _txt(ax, '\n'.join(lines), C, 6.5)
    ax = _ax(fig, gs[0, 1], C, 'permissible grip force by theory vs no-slip force', False)
    cap = fs['F_other']
    vals = [min(fs['N_perm'][t], 3 * max(cap, 1)) for t in THEORIES]
    cols = [COL['d'] if t == th else COL['a'] for t in THEORIES]
    left_min = 1e-2
    ax.barh(range(5), [max(v - left_min, 1e-9) for v in vals], left=left_min, color=cols)
    ax.set_yticks(range(5)); ax.set_yticklabels(THEORIES, fontsize=7, color=C['fg'])
    ax.axvline(fs['N_slip'] * SLIP_SF, color=COL['c'], lw=1.5); ax.text(fs['N_slip'] * SLIP_SF, 4.6, f' slip x{SLIP_SF}', color=COL['c'], fontsize=7)
    if ctx['N'] > left_min:
        ax.axvline(ctx['N'], color='w', lw=0.8, ls=':')
        ax.text(ctx['N'], 4.55, ' now', color=C['fg'], fontsize=7)
    ax.set_xscale('log')
    ax.set_xlim(left_min, max(max(vals) * 2.2, 10.0, cap * 1.3))
    ax.axvline(cap, color=COL['b'], lw=1.2, ls='--'); ax.text(cap, -0.9, ' gripper / shell limit', color=COL['b'], fontsize=7, ha='right')
    for i, t in enumerate(THEORIES):
        Np = fs['N_perm'][t]
        ax.text(max(vals[i], left_min) * 1.2, i, ('inf' if math.isinf(Np) else f'{Np:.2f} N'), va='center', fontsize=7, color=C['fg'])
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
    ax.text(0.0, 1.0, lines[0], va='top', ha='left', family='monospace', fontsize=11, color=col, fontweight='bold', transform=ax.transAxes)
    ax.text(0.0, 0.86, '\n'.join(lines[1:]), va='top', ha='left', family='monospace', fontsize=6.7, color=C['fg'], transform=ax.transAxes)
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

GRIPEOF

cat << 'GRIPEOF' > "$PKG/grip_gz/dashboard.py"
"""dashboard.py - Low-Latency MuJoCo 3D Simulator Bridge & MoS Telemetry Console."""
import math
import os
import shlex
import time
import threading
import numpy as np
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node

from . import cad as cadmod
from . import hand as HD
from . import nptel_priority as NP
from .hand_plant import Episode, run_headless
from .arm_ik import ARM, ik_wrist, tcp_home
from .materials import GRIPPER, OBJECTS, PADS, G, height
from .world import TABLE_TOP_Z, build_mjcf

def fmt_modulus(val): return "probing..." if val is None or math.isnan(val) or val <= 0 else (f"{val/1e9:.2f} GPa" if val >= 1e9 else f"{val/1e6:.2f} MPa")
def fmt_fos(f):
    """Safety factor as it really is (no cap).  inf -> no load / no limit."""
    if f is None or (isinstance(f, float) and math.isnan(f)): return "-"
    if math.isinf(f): return "no limit"
    return f"{f:.2f}" if f < 100 else (f"{f:.0f}" if f < 1000 else f"{f:.2e}")
def fmt_stress(val): return "inf" if val is None or math.isinf(val) else (f"{val/1e6:.3f} MPa" if abs(val) >= 1e6 else f"{val/1e3:.2f} kPa")

class MuJoCoSimBridgeNode(Node):
    def __init__(self):
        super().__init__('grip_sim_dashboard_node')
        self.declare_parameter('object', 'apple')
        self.declare_parameter('pad', 'small')
        self.declare_parameter('seed', 0)
        self.declare_parameter('cad', '')
        self.declare_parameter('cad_material', 'aluminium')
        self.declare_parameter('cad_unit', 'mm')
        self.declare_parameter('cad_args', '')
        self.declare_parameter('fingers', 5)

        self.init_obj = self.get_parameter('object').value
        self.init_cad_spec = self.init_cad_rep = self.init_cad_error = None
        self.cad_dirty = False
        cad_file = str(self.get_parameter('cad').value or '').strip()
        if cad_file:
            try:
                argv = [cad_file, '--material', str(self.get_parameter('cad_material').value or 'aluminium'),
                        '--unit', str(self.get_parameter('cad_unit').value or 'mm')] + shlex.split(str(self.get_parameter('cad_args').value or ''))
                spec = cadmod.spec_from_args(argv)
                ckey, crep = self.load_cad(spec)
                self.init_obj, self.init_cad_spec, self.init_cad_rep = ckey, spec, crep
                self.get_logger().info('CAD part loaded:\n' + cadmod.format_report(crep))
            except BaseException as e:
                self.init_cad_error = str(e) or repr(e)
                self.get_logger().error('CAD load failed: %s' % self.init_cad_error)

        self.init_pad = self.get_parameter('pad').value
        self.seed = int(self.get_parameter('seed').value)
        HD.HAND['n_fingers'] = max(3, min(5, int(self.get_parameter('fingers').value)))

        self.running = self.paused = self.stop_req = False
        self.z0 = TABLE_TOP_Z
        self._mj_lock = threading.Lock()
        self.mj_model = self.mj_data = self.viewer = self.n_spawned = self.active_obj_key = None
        self.known_objects = set()
        self.qadr_arm = ()
        self.qadr_fingers = []
        self.mocap_intact, self.mocap_crush = {}, {}
        self.ensure_scene(HD.HAND['n_fingers'], self.init_obj)

    def load_cad(self, spec):
        """Analyse a CAD file with the given spec, register it as a specimen. Returns (key, report). Raises CadError."""
        key, obj, rep = cadmod.make_object(spec)
        obj['mesh_ok'] = self._mesh_compiles(obj.get('mesh'))
        if not obj['mesh_ok']:                                    # should not happen (face limit is enforced in cad.py) - degrade gracefully: coarser mesh, then its convex hull
            try:
                from . import meshprep as MP
                V, F = cadmod.load_mesh(obj['mesh'])
                V, F = MP._compact(*MP.weld(V, F)) if False else MP.weld(V, F)
                for tgt in (30000, 8000, 2000):
                    V2, F2 = MP.decimate(V, F, tgt); F2, _ = MP.orient_consistent(F2)
                    cadmod._write_stl(obj['mesh'], V2, F2)
                    if self._mesh_compiles(obj['mesh']): obj['mesh_ok'] = True; rep['warnings'].append('Display mesh reduced to %d triangles so MuJoCo could load it.' % len(F2)); break
            except Exception: pass
        if not obj['mesh_ok']:
            rep['warnings'].append('MuJoCo could not load this mesh; a plain cylinder is shown instead (physics is unaffected).')
        cadmod.register(key, obj)
        self.cad_dirty = True              # forces the MuJoCo scene to be rebuilt with the new / changed mesh
        return key, rep

    @staticmethod
    def _mesh_compiles(path):
        try:
            mujoco.MjModel.from_xml_string('<mujoco><asset><mesh name="m" file="%s"/></asset><worldbody><geom type="mesh" mesh="m"/></worldbody></mujoco>' % path)
            return True
        except Exception:
            return False

    def ensure_scene(self, n=None, obj_key=None):
        n = int(n or HD.HAND['n_fingers'])
        with self._mj_lock:
            if self.mj_model is None or self.n_spawned != n or self.cad_dirty or (obj_key is not None and obj_key not in self.known_objects):
                HD.HAND['n_fingers'] = n
                self.cad_dirty = False
                self.mj_model = mujoco.MjModel.from_xml_string(build_mjcf(n_fingers=n, z0=self.z0))
                self.mj_data = mujoco.MjData(self.mj_model)
                self.n_spawned, self.known_objects = n, set(OBJECTS.keys())
                self.qadr_arm = tuple(self.mj_model.jnt_qposadr[mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, jn)] for jn in ('shoulder', 'elbow', 'wrist'))
                self.qadr_yaw = self.mj_model.jnt_qposadr[mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, 'yaw')]
                self.qadr_spread = [self.mj_model.jnt_qposadr[mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, f'f{i}_spread')] for i in range(n)]
                self.qadr_fingers = [tuple(self.mj_model.jnt_qposadr[mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, f'f{i}_{jt}')] for jt in ('mcp', 'pip', 'dip')) for i in range(n)]
                for k in OBJECTS:
                    bid_i, bid_c = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, f'obj_{k}'), mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, f'crush_{k}')
                    if bid_i >= 0: self.mocap_intact[k] = self.mj_model.body_mocapid[bid_i]
                    if bid_c >= 0: self.mocap_crush[k] = self.mj_model.body_mocapid[bid_c]
                if self.viewer is not None:
                    try: self.viewer.close()
                    except Exception: pass
                try:
                    self.viewer = mujoco.viewer.launch_passive(self.mj_model, self.mj_data, show_left_ui=False, show_right_ui=False)
                    self.viewer.cam.lookat[:], self.viewer.cam.distance, self.viewer.cam.elevation, self.viewer.cam.azimuth = [-0.08, 0.0, 0.18], 1.15, -18.0, 132.0
                except Exception: self.viewer = None
            if obj_key is not None:
                for k, mid in self.mocap_intact.items(): self.mj_data.mocap_pos[mid] = [0.0, 0.0, -5.0]
                for k, mid in self.mocap_crush.items(): self.mj_data.mocap_pos[mid] = [0.0, 0.0, -5.0]
                self.active_obj_key = obj_key
                if obj_key in OBJECTS and obj_key in self.mocap_intact:
                    self.mj_data.mocap_pos[self.mocap_intact[obj_key]] = [0.0, 0.0, height(OBJECTS[obj_key]) / 2.0 + self.z0]
            self._set_arm_and_fingers_locked(*tcp_home(), [0.0] * n, 0.0, [0.0] * n)
            mujoco.mj_forward(self.mj_model, self.mj_data)
            if self.viewer and self.viewer.is_running(): self.viewer.sync()

    def _set_arm_and_fingers_locked(self, tcp_x, tcp_z, s_list, yaw=0.0, sigmas=None):
        th1, th2, th3 = ik_wrist(tcp_x, tcp_z + ARM['tcp_drop'])
        self.mj_data.qpos[self.qadr_arm[0]], self.mj_data.qpos[self.qadr_arm[1]], self.mj_data.qpos[self.qadr_arm[2]] = th1, th2, th3
        self.mj_data.qpos[self.qadr_yaw] = yaw
        for i, sg in enumerate(sigmas or []): self.mj_data.qpos[self.qadr_spread[i]] = sg
        for i, s in enumerate(s_list):
            q1, q2, q3 = HD.joint_angles(s)
            self.mj_data.qpos[self.qadr_fingers[i][0]], self.mj_data.qpos[self.qadr_fingers[i][1]], self.mj_data.qpos[self.qadr_fingers[i][2]] = q1, q2, q3

    def sync_frame(self, tcp_x, tcp_z, s_list, o, obj_x, obj_rise_z, crushed, yaw=0.0, sigmas=None, ep=None):
        with self._mj_lock:
            if self.mj_model is None: return
            if not all(math.isfinite(v) for v in [tcp_x, tcp_z, yaw, *s_list]): return
            self._set_arm_and_fingers_locked(tcp_x, tcp_z, s_list, yaw, sigmas)
            k = self.active_obj_key
            if k in self.mocap_intact and k in self.mocap_crush:
                z_world, mid_i, mid_c = height(o) / 2.0 + obj_rise_z + self.z0, self.mocap_intact[k], self.mocap_crush[k]
                self.mj_data.mocap_pos[mid_i] = [0.0, 0.0, -5.0] if crushed else [obj_x, 0.0, z_world]
                self.mj_data.mocap_pos[mid_c] = [obj_x, 0.0, z_world] if crushed else [0.0, 0.0, -5.0]
            mujoco.mj_forward(self.mj_model, self.mj_data)
            self._draw_contacts(ep, obj_x, obj_rise_z, crushed)
            if self.viewer and self.viewer.is_running(): self.viewer.sync()

    def _draw_contacts(self, ep, obj_x, rise_z, crushed):
        """Red dot on the part where each loaded fingertip touches it + a force arrow along the contact normal (viewer overlay only)."""
        v = self.viewer
        if v is None or not hasattr(v, 'user_scn'): return
        scn = v.user_scn
        scn.ngeom = 0
        if ep is None or crushed or not ep.plan.get('ok'): return
        try:
            m, d, pl, R = self.mj_model, self.mj_data, ep.plant, HD.HAND['tip_r']
            L3, pb = HD.HAND['L'][2], HD.HAND['pad_back']
            org = np.array([obj_x, 0.0, self.z0 + rise_z])
            for i in range(ep.n):
                if not pl.active[i] or pl.N[i] <= 0.01: continue
                bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f'grip_f{i}_p3')
                c = d.xpos[bid] + d.xmat[bid].reshape(3, 3) @ np.array([L3 - pb, 0.0, 0.0])
                P = (c - org)[None]
                sd = float(ep.shape.sdf(P)[0])
                nrm = ep.shape.normal(P)[0]
                p_surf = c - nrm * sd                                   # nearest point on the part surface
                if scn.ngeom + 2 > scn.maxgeom: break
                mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.0035, 0, 0]), p_surf, np.eye(3).ravel(), np.array([1.0, 0.1, 0.1, 1.0], dtype=np.float32))
                scn.geoms[scn.ngeom].emission = 0.9
                scn.ngeom += 1
                # arrow sits OUTSIDE the fingertip dome (dome reaches ~2*tip_r from the surface) and points at the contact point
                base = 2.0 * R + 0.004
                ln = min(0.004 + 0.0015 * float(pl.N[i]), 0.025)
                mujoco.mjv_connector(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_ARROW, 0.0022, p_surf + nrm * (base + ln), p_surf + nrm * base)
                scn.geoms[scn.ngeom].rgba[:] = (1.0, 0.85, 0.1, 1.0)
                scn.geoms[scn.ngeom].emission = 0.9
                scn.ngeom += 1
        except Exception:
            scn.ngeom = 0

    def close_viewer(self):
        with self._mj_lock:
            if self.viewer is not None:
                try: self.viewer.close()
                except Exception: pass
                self.viewer = None

class SimDashboardGUI:
    BG_MAIN, BG_CARD, BG_HEADER, BORDER = "#0b0f17", "#111827", "#161f30", "#1f2937"
    TEXT_MAIN, TEXT_MUTED = "#f3f4f6", "#9ca3af"
    ACCENT_BLUE, ACCENT_GREEN, ACCENT_AMBER, ACCENT_RED, ACCENT_PURPLE = "#38bdf8", "#22c55e", "#f59e0b", "#ef4444", "#a855f7"
    FSM_STAGES = ["CALIB", "WAIT_ARM", "CLOSE", "SQUEEZE", "SETTLE", "LIFT", "HOLD"]

    def __init__(self, ros_node):
        self.node = ros_node
        self.root = tk.Tk()
        self.root.title("grip_gz // MuJoCo Dexterous Adaptive Force & MoS Telemetry Console")
        self.root.geometry("1440x960")
        self.root.configure(bg=self.BG_MAIN)
        self._latest_ep = self._latest_strat = None
        self._dirty_ui = False
        self._episode_setup_needed = True
        self._last_plot_t = 0.0
        self._last_err = None

        style = ttk.Style(self.root)
        style.theme_use("clam")
        for k in ("TCombobox", "Stick.Horizontal.TProgressbar", "Torque.Horizontal.TProgressbar"):
            style.configure(k, fieldbackground="#1e293b", background="#334155" if k == "TCombobox" else (self.ACCENT_BLUE if "Stick" in k else self.ACCENT_PURPLE), foreground="#f8fafc", arrowcolor="#38bdf8", bordercolor="#334155" if k == "TCombobox" else "#1e293b", lightcolor="#1e293b" if k == "TCombobox" else (self.ACCENT_BLUE if "Stick" in k else self.ACCENT_PURPLE), darkcolor="#1e293b" if k == "TCombobox" else (self.ACCENT_BLUE if "Stick" in k else self.ACCENT_PURPLE))
        style.configure("TNotebook", background=self.BG_CARD, borderwidth=0)
        style.configure("TNotebook.Tab", background="#1e293b", foreground=self.TEXT_MUTED, padding=(12, 4), font=("Consolas", 9, "bold"))
        style.map("TNotebook.Tab", background=[("selected", "#0f172a")], foreground=[("selected", self.ACCENT_BLUE)])
        style.configure("Dark.Treeview", background="#0f172a", fieldbackground="#0f172a", foreground="#e2e8f0", rowheight=22, borderwidth=0, font=("Consolas", 9))
        style.configure("Dark.Treeview.Heading", background="#1e293b", foreground=self.ACCENT_BLUE, font=("Consolas", 9, "bold"), relief="flat")
        style.map("Dark.Treeview", background=[("selected", "#334155")])
        style.map("TCombobox", fieldbackground=[("readonly", "#1e293b")], selectbackground=[("readonly", "#1e293b")], selectforeground=[("readonly", "#38bdf8")])

        top = tk.Frame(self.root, bg=self.BG_HEADER, pady=10, padx=14, highlightthickness=1, highlightbackground=self.BORDER)
        top.pack(fill=tk.X)
        brand = tk.Frame(top, bg=self.BG_HEADER)
        brand.pack(side=tk.LEFT, padx=(0, 14))
        tk.Label(brand, text="GRIP_MUJOCO", bg=self.BG_HEADER, fg=self.ACCENT_BLUE, font=("Consolas", 13, "bold")).pack(side=tk.LEFT)
        tk.Label(brand, text=" // MoS TACTILE LAB", bg=self.BG_HEADER, fg=self.TEXT_MUTED, font=("Consolas", 10, "bold")).pack(side=tk.LEFT)

        def add_combo(label, var, values, width):
            tk.Label(top, text=label, bg=self.BG_HEADER, fg=self.TEXT_MUTED, font=("Consolas", 9, "bold")).pack(side=tk.LEFT, padx=(6, 3))
            cb = ttk.Combobox(top, textvariable=var, values=values, width=width, state="readonly", font=("Consolas", 9))
            cb.pack(side=tk.LEFT, padx=(0, 6))
            return cb

        self.obj_var, self.strat_var, self.pad_var, self.fing_var, self.speed_var = tk.StringVar(value=self.node.init_obj), tk.StringVar(value="Adaptive (blind)"), tk.StringVar(value="small (R=25mm)" if self.node.init_pad == "small" else "large (R=100mm)"), tk.StringVar(value=str(HD.HAND['n_fingers'])), tk.StringVar(value="1x (Real-time)")
        self.obj_cb = add_combo("OBJECT", self.obj_var, list(OBJECTS.keys()), 16)
        add_combo("MODE", self.strat_var, ["Adaptive (blind)", "Fixed 25 N"], 15)
        add_combo("PAD", self.pad_var, ["small (R=25mm)", "large (R=100mm)"], 14)
        add_combo("FINGERS", self.fing_var, ["3", "4", "5"], 3)
        add_combo("SPEED", self.speed_var, ["0.5x", "1x (Real-time)", "2x", "4x"], 13)
        self.speed_mult = 1.0
        self.speed_var.trace_add("write", self._speed_changed)

        btn_frame = tk.Frame(top, bg=self.BG_HEADER)
        btn_frame.pack(side=tk.RIGHT)
        self.btns = {}
        for t, c, cmd in [("▶ RUN EPISODE", "#16a34a", self.start_sim), ("⏸ PAUSE", "#d97706", self.toggle_pause), ("⏹ STOP", "#dc2626", self.stop_sim), ("▦ BENCHMARK ALL", "#2563eb", self.start_benchmark)]:
            b = tk.Button(btn_frame, text=t, bg=c, fg="white", command=cmd, font=("Consolas", 9, "bold"), bd=0, padx=9, pady=5, cursor="hand2", activebackground="#334155")
            b.pack(side=tk.LEFT, padx=3)
            self.btns[t] = b

        bar = tk.Frame(self.root, bg="#0f172a", pady=6, padx=14, highlightthickness=1, highlightbackground=self.BORDER)
        bar.pack(fill=tk.X)
        tk.Label(bar, text="CONTROLLER PIPELINE:", bg="#0f172a", fg=self.TEXT_MUTED, font=("Consolas", 9, "bold")).pack(side=tk.LEFT, padx=(0, 10))
        self.stage_pills = {st: tk.Label(bar, text=f" {st} ", bg="#1e293b", fg="#64748b", font=("Consolas", 9, "bold"), padx=8, pady=2) for st in self.FSM_STAGES}
        for pill in self.stage_pills.values(): pill.pack(side=tk.LEFT, padx=3)
        self.arm_badge = tk.Label(bar, text="ARM: HOME | t = 0.00s", bg="#0f172a", fg=self.ACCENT_BLUE, font=("Consolas", 9, "bold"))
        self.arm_badge.pack(side=tk.RIGHT)

        self._build_cad_bar()

        main = tk.Frame(self.root, bg=self.BG_MAIN)
        main.pack(fill=tk.BOTH, expand=True, padx=12, pady=10)
        left = tk.Frame(main, bg=self.BG_MAIN, width=560)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 12))
        left.pack_propagate(False)

        kpi_row = tk.Frame(left, bg=self.BG_MAIN)
        kpi_row.pack(fill=tk.X, pady=(0, 8))
        self.kpi_vars, self.kpi_boxes = {}, {}
        for i, (key, title, init_v, color) in enumerate([("fz", "NORMAL FORCE f_z", "0.00 N", self.ACCENT_BLUE), ("fos", "SAFETY FACTOR", "-", self.ACCENT_GREEN), ("vm", "VON MISES σ_vM", "0.0 kPa", "#f472b6"), ("lift", "OBJECT LIFT z", "0.0 mm", self.ACCENT_AMBER)]):
            card = tk.Frame(kpi_row, bg=self.BG_CARD, padx=8, pady=6, highlightthickness=1, highlightbackground=self.BORDER)
            card.grid(row=0, column=i, sticky="nsew", padx=(0 if i == 0 else 4, 0))
            kpi_row.columnconfigure(i, weight=1)
            tk.Label(card, text=title, bg=self.BG_CARD, fg=self.TEXT_MUTED, font=("Consolas", 8, "bold")).pack(anchor="w")
            v = tk.StringVar(value=init_v)
            lbl = tk.Label(card, textvariable=v, bg=self.BG_CARD, fg=color, font=("Consolas", 13, "bold"))
            lbl.pack(anchor="w", pady=(2, 0))
            self.kpi_vars[key], self.kpi_boxes[key] = v, lbl

        gauge_card = tk.Frame(left, bg=self.BG_CARD, padx=10, pady=7, highlightthickness=1, highlightbackground=self.BORDER)
        gauge_card.pack(fill=tk.X, pady=(0, 8))
        g1 = tk.Frame(gauge_card, bg=self.BG_CARD)
        g1.pack(fill=tk.X, pady=2)
        self.stick_lbl = tk.Label(g1, text="Mindlin Stick Zone (c/a): 100%", bg=self.BG_CARD, fg=self.TEXT_MAIN, font=("Consolas", 8, "bold"), width=32, anchor="w")
        self.stick_lbl.pack(side=tk.LEFT)
        self.stick_bar = ttk.Progressbar(g1, orient="horizontal", mode="determinate", maximum=100, value=100, style="Stick.Horizontal.TProgressbar")
        self.stick_bar.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=(8, 0))
        g2 = tk.Frame(gauge_card, bg=self.BG_CARD)
        g2.pack(fill=tk.X, pady=2)
        self.torque_lbl = tk.Label(g2, text="MCP Peak Torque Load:   0% (0.00 Nm)", bg=self.BG_CARD, fg=self.TEXT_MAIN, font=("Consolas", 8, "bold"), width=32, anchor="w")
        self.torque_lbl.pack(side=tk.LEFT)
        self.torque_bar = ttk.Progressbar(g2, orient="horizontal", mode="determinate", maximum=100, value=0, style="Torque.Horizontal.TProgressbar")
        self.torque_bar.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=(8, 0))

        self.panel_frame = tk.Frame(left, bg=self.BG_CARD, highlightthickness=2, highlightbackground=self.BORDER)
        self.panel_frame.pack(fill=tk.BOTH, expand=True)
        self.result_lbl = tk.Label(self.panel_frame, text="STATUS: STANDBY", bg="#1e293b", fg=self.ACCENT_BLUE, font=("Consolas", 11, "bold"), pady=8)
        self.result_lbl.pack(fill=tk.X, side=tk.BOTTOM)
        self.nb = ttk.Notebook(self.panel_frame)
        self.nb.pack(fill=tk.BOTH, expand=True)
        tab_tel = tk.Frame(self.nb, bg=self.BG_CARD)
        tab_fing = tk.Frame(self.nb, bg=self.BG_CARD)
        tab_res = tk.Frame(self.nb, bg=self.BG_CARD)
        self.nb.add(tab_tel, text=" TELEMETRY ")
        self.nb.add(tab_fing, text=" FINGERS ")
        self.nb.add(tab_res, text=" RESULTS ")
        tab_cad = tk.Frame(self.nb, bg=self.BG_CARD)
        self.nb.add(tab_cad, text=" CAD REPORT ")
        self.cad_text = tk.Text(tab_cad, bg="#0f172a", fg="#e2e8f0", font=("Consolas", 9), wrap="word", bd=0, padx=10, pady=8, height=10)
        self.cad_text.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)
        self.cad_text.insert("1.0", "No CAD part loaded.\n\nUse  BROWSE STL/OBJ...  in the CAD INPUT bar, set the unit and material\n(edit any property if you know better), then press  ANALYSE & GRIP.")
        self.cad_text.config(state="disabled")
        tab_mech = tk.Frame(self.nb, bg=self.BG_CARD)
        self.nb.add(tab_mech, text=" ANALYSIS ")
        mbar = tk.Frame(tab_mech, bg=self.BG_CARD); mbar.pack(fill=tk.X, padx=6, pady=(6, 0))
        tk.Button(mbar, text="OPEN ANALYSIS GRAPHS", command=self.open_np_window, bg="#1e293b", fg=self.ACCENT_GREEN,
                  font=("Consolas", 8, "bold"), relief="flat", padx=8).pack(side=tk.LEFT)
        tk.Label(mbar, text="stress state - element equilibrium - wrist shaft - beams - finger deflection - failure limits",
                 bg=self.BG_CARD, fg=self.TEXT_MUTED, font=("Consolas", 7)).pack(side=tk.LEFT, padx=8)
        self.mech_text = tk.Text(tab_mech, bg="#0f172a", fg="#e2e8f0", font=("Consolas", 8), wrap="none", bd=0, padx=8, pady=6, height=10)
        msb = ttk.Scrollbar(tab_mech, orient="vertical", command=self.mech_text.yview)
        self.mech_text.configure(yscrollcommand=msb.set)
        msb.pack(side=tk.RIGHT, fill=tk.Y, pady=6)
        self.mech_text.pack(fill=tk.BOTH, expand=True, padx=(6, 0), pady=6)
        self.mech_text.insert("1.0", "The analysis report appears here once the pads touch the object.")
        self.mech_text.config(state="disabled")
        self._mech_t = 0.0
        self.telemetry_lbl = tk.Label(tab_tel, text="MuJoCo 3D Engine Ready & Tactile Controller Standby...", bg=self.BG_CARD, fg=self.TEXT_MAIN, font=("Consolas", 9), justify=tk.LEFT, anchor="nw", padx=12, pady=10)
        self.telemetry_lbl.pack(fill=tk.BOTH, expand=True)

        fcols = (("finger", "Finger", 58), ("contact", "Contact", 62), ("gap", "Gap mm", 58), ("N", "N (N)", 62), ("mcp", "MCP Nm", 66), ("pip", "PIP Nm", 66), ("dip", "DIP Nm", 66), ("s", "Curl s", 56))
        self.fing_tree = ttk.Treeview(tab_fing, columns=[c[0] for c in fcols], show="headings", style="Dark.Treeview", height=6)
        for cid, ctext, w in fcols:
            self.fing_tree.heading(cid, text=ctext)
            self.fing_tree.column(cid, width=w, anchor="center", stretch=True)
        self.fing_tree.tag_configure("touch", foreground=self.ACCENT_GREEN)
        self.fing_tree.tag_configure("free", foreground="#64748b")
        self.fing_tree.tag_configure("sum", foreground=self.ACCENT_AMBER)
        self.fing_tree.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

        rcols = (("n", "#", 28), ("obj", "Object", 92), ("mode", "Mode", 62), ("fg", "Fn", 26), ("res", "Result", 92), ("peak", "Peak N", 56), ("fos", "FoS", 46), ("lift", "Lift mm", 56), ("m", "m_est g", 56))
        self.res_tree = ttk.Treeview(tab_res, columns=[c[0] for c in rcols], show="headings", style="Dark.Treeview")
        for cid, ctext, w in rcols:
            self.res_tree.heading(cid, text=ctext)
            self.res_tree.column(cid, width=w, anchor="center" if cid != "obj" else "w", stretch=True)
        self.res_tree.tag_configure("ok", foreground=self.ACCENT_GREEN)
        self.res_tree.tag_configure("bad", foreground=self.ACCENT_RED)
        self.res_tree.tag_configure("warn", foreground=self.ACCENT_AMBER)
        rsb = ttk.Scrollbar(tab_res, orient="vertical", command=self.res_tree.yview)
        self.res_tree.configure(yscrollcommand=rsb.set)
        rsb.pack(side=tk.RIGHT, fill=tk.Y, pady=6)
        self.res_tree.pack(fill=tk.BOTH, expand=True, padx=(6, 0), pady=6)
        self._res_count = 0
        self._bench_running = False

        plot_frame = tk.Frame(main, bg=self.BG_CARD, highlightthickness=1, highlightbackground=self.BORDER)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        self.fig = Figure(figsize=(7.6, 6.8), dpi=100, facecolor=self.BG_CARD)
        self.ax_force = self.fig.add_subplot(211)
        self.ax_stress = self.fig.add_subplot(212)
        self.fig.subplots_adjust(hspace=0.34, left=0.10, right=0.97, top=0.93, bottom=0.08)
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=4, pady=4)
        
        self.root.after(66, self._ui_poll_loop)
        if self.node.init_cad_spec:
            self._cad_fill_from_spec(self.node.init_cad_spec)
            self._cad_show_report(self.node.init_cad_rep)
        if self.node.init_cad_error:
            err = self.node.init_cad_error
            self.cad_status.config(text="CAD load failed: " + err, fg=self.ACCENT_RED)
            self.root.after(900, lambda: messagebox.showerror("CAD load failed", err))
        self.root.after(600, self.start_sim)

    # ------------------------------------------------------------------ CAD input
    CAD_FIELDS = (  # key, label, width, scale to SI, hint
        ("mass", "MASS g", 7, 1e-3, "blank = density x volume"),
        ("density", "RHO kg/m3", 7, 1.0, ""),
        ("mu", "MU", 5, 1.0, "friction coefficient pad/part"),
        ("E", "E GPa", 7, 1e9, "Young's modulus"),
        ("nu", "NU", 5, 1.0, "Poisson ratio"),
        ("strength", "SIGMA_FAIL MPa", 8, 1e6, "yield / fracture / crush strength"),
        ("F_allow", "F_MAX N", 6, 1.0, "blank = none (shell default 10 N)"),
        ("R_contact", "R mm", 6, 1e-3, "blank = measured from the mesh"),
    )

    def _build_cad_bar(self):
        BG = "#0d1424"
        cad = tk.Frame(self.root, bg=BG, pady=6, padx=14, highlightthickness=1, highlightbackground=self.BORDER)
        cad.pack(fill=tk.X)
        r1 = tk.Frame(cad, bg=BG)
        r1.pack(fill=tk.X)
        r2 = tk.Frame(cad, bg=BG)
        r2.pack(fill=tk.X, pady=(5, 0))

        def lab(parent, text): tk.Label(parent, text=text, bg=BG, fg=self.TEXT_MUTED, font=("Consolas", 8, "bold")).pack(side=tk.LEFT, padx=(8, 3))
        def entry(parent, var, width): 
            e = tk.Entry(parent, textvariable=var, width=width, bg="#1e293b", fg="#f8fafc", insertbackground="#f8fafc", relief="flat", font=("Consolas", 9), highlightthickness=1, highlightbackground="#334155", highlightcolor=self.ACCENT_BLUE)
            e.pack(side=tk.LEFT)
            return e
        def combo(parent, var, values, width):
            cb = ttk.Combobox(parent, textvariable=var, values=values, width=width, state="readonly", font=("Consolas", 9))
            cb.pack(side=tk.LEFT)
            return cb

        tk.Label(r1, text="CAD INPUT:", bg=BG, fg=self.ACCENT_PURPLE, font=("Consolas", 9, "bold")).pack(side=tk.LEFT)
        tk.Button(r1, text="📂 BROWSE STL / OBJ...", bg="#7c3aed", fg="white", command=self.cad_browse, font=("Consolas", 9, "bold"), bd=0, padx=9, pady=4, cursor="hand2", activebackground="#334155").pack(side=tk.LEFT, padx=(8, 4))
        self.cad_path_var = tk.StringVar(value="")
        entry(r1, self.cad_path_var, 52)
        self.cad_unit_var, self.cad_up_var, self.cad_rot_var = tk.StringVar(value="mm"), tk.StringVar(value="auto"), tk.StringVar(value="")
        lab(r1, "UNIT"); combo(r1, self.cad_unit_var, list(cadmod.UNITS.keys()), 4)
        lab(r1, "UP AXIS"); combo(r1, self.cad_up_var, ["auto", "z", "y", "-y", "x", "-x", "-z"], 5)
        lab(r1, "ROT Z deg (blank=auto)"); entry(r1, self.cad_rot_var, 5)
        self.cad_status = tk.Label(r1, text="", bg=BG, fg=self.TEXT_MUTED, font=("Consolas", 8, "bold"), anchor="w")
        self.cad_status.pack(side=tk.LEFT, padx=12)

        self.cad_mat_var = tk.StringVar(value="aluminium")
        self.cad_kind_var = tk.StringVar(value="ductile")
        self.cad_vars = {k: tk.StringVar(value="") for k, *_ in self.CAD_FIELDS}
        lab(r2, "MATERIAL")
        mcb = combo(r2, self.cad_mat_var, list(cadmod.MATERIALS.keys()), 13)
        mcb.bind("<<ComboboxSelected>>", self._cad_material_changed)
        for key, label, width, _scale, _hint in self.CAD_FIELDS:
            lab(r2, label); entry(r2, self.cad_vars[key], width)
        lab(r2, "FAILURE MODE"); combo(r2, self.cad_kind_var, list(cadmod.KIND_INFO.keys()), 9)
        tk.Button(r2, text="▶ ANALYSE & GRIP", bg="#16a34a", fg="white", command=self.cad_apply, font=("Consolas", 9, "bold"), bd=0, padx=9, pady=4, cursor="hand2", activebackground="#334155").pack(side=tk.LEFT, padx=(14, 0))
        self._cad_busy = False
        self._cad_material_changed()

    def cad_browse(self):
        path = filedialog.askopenfilename(parent=self.root, title="Select a CAD mesh (STL / OBJ)",
                                          filetypes=[("CAD mesh", "*.stl *.STL *.obj *.OBJ *.ply *.off *.3mf *.glb"), ("All files", "*.*")])
        if path:
            self.cad_path_var.set(path)
            self.cad_status.config(text="file selected - pick unit & material, then ANALYSE & GRIP", fg=self.TEXT_MUTED)

    def _cad_material_changed(self, *_):
        m = cadmod.MATERIALS.get(self.cad_mat_var.get())
        if not m: return
        v = self.cad_vars
        v["mass"].set("")
        v["density"].set("%g" % m["rho"]); v["mu"].set("%g" % m["mu"]); v["E"].set("%g" % (m["E"] / 1e9))
        v["nu"].set("%g" % m["nu"]); v["strength"].set("%g" % (m["strength"] / 1e6))
        v["F_allow"].set("10" if m["kind"] == "shell" else ""); v["R_contact"].set("")
        self.cad_kind_var.set(m["kind"])

    def _cad_spec(self):
        path = self.cad_path_var.get().strip().strip('"').strip("'")
        if not path: raise ValueError("Choose a CAD file first (BROWSE STL / OBJ...).")
        spec = dict(file=os.path.expanduser(path), unit=self.cad_unit_var.get(), up=self.cad_up_var.get(), material=self.cad_mat_var.get(), kind=self.cad_kind_var.get())
        try:
            if self.cad_rot_var.get().strip(): spec["rotz"] = float(self.cad_rot_var.get())
        except ValueError: raise ValueError("ROT Z must be a number (degrees).")
        for key, label, _w, scale, _h in self.CAD_FIELDS:
            txt = self.cad_vars[key].get().strip()
            if txt:
                try: spec[key] = float(txt) * scale
                except ValueError: raise ValueError("%s must be a number (got %r)." % (label, txt))
        return spec

    def _cad_fill_from_spec(self, spec):
        self.cad_path_var.set(spec.get("file", ""))
        self.cad_unit_var.set(spec.get("unit", "mm")); self.cad_up_var.set(spec.get("up", "auto")); self.cad_rot_var.set("" if spec.get("rotz") in (None, "") else "%g" % float(spec["rotz"]))
        self.cad_mat_var.set(spec.get("material", "aluminium")); self._cad_material_changed()
        if spec.get("kind"): self.cad_kind_var.set(spec["kind"])
        for key, _l, _w, scale, _h in self.CAD_FIELDS:
            if spec.get(key) not in (None, ""): self.cad_vars[key].set("%g" % (float(spec[key]) / scale))

    def _cad_show_report(self, rep):
        self.cad_text.config(state="normal")
        self.cad_text.delete("1.0", tk.END)
        self.cad_text.insert("1.0", cadmod.format_report(rep) + "\n\nMaterial / failure mode used: %s / %s\nE = %.3g GPa, nu = %.2f, mu = %.2f, strength = %.3g MPa" % (
            rep["material"], rep["kind"], rep["E"] / 1e9, rep["nu"], rep["mu"], rep["strength"] / 1e6))
        self.cad_text.config(state="disabled")

    def cad_apply(self):
        if self._cad_busy: return
        try: spec = self._cad_spec()
        except ValueError as e:
            messagebox.showerror("CAD input", str(e)); return
        self._cad_busy = True
        self._cad_n = max(3, min(5, int(self.fing_var.get())))
        self.cad_status.config(text="analysing mesh and planning the grasp (can take up to a minute)...", fg=self.ACCENT_AMBER)
        threading.Thread(target=self._cad_worker, args=(spec,), daemon=True).start()

    def _cad_worker(self, spec):
        try:
            key, rep = self.node.load_cad(spec)
            try:
                from .physics_check import plan_summary
                rep['plan_text'] = plan_summary(key, getattr(self, '_cad_n', 5))      # plan + MuJoCo proof here, so the report says why a part is refused
            except Exception as exc:
                rep['plan_text'] = 'Grasp planning failed: %r' % (exc,)
            self.root.after(0, self._cad_done, key, rep, None)
        except BaseException as e:
            self.root.after(0, self._cad_done, None, None, str(e) or repr(e))

    def _cad_done(self, key, rep, err):
        self._cad_busy = False
        if err:
            self.cad_status.config(text="CAD error: " + err, fg=self.ACCENT_RED)
            messagebox.showerror("CAD input", err)
            return
        self.obj_cb["values"] = list(OBJECTS.keys())
        self.obj_var.set(key)
        self._cad_show_report(rep)
        self.nb.select(3)
        ok_pad = [("GRIPPABLE" if self._grippable(key, p) and "NO SAFE GRIP" not in rep.get("plan_text", "") else "NO SAFE GRIP") for p in ("small", "large")]
        self.cad_status.config(text="loaded %s | small pad: %s | large pad: %s%s" % (key, ok_pad[0], ok_pad[1], " | " + rep["warnings"][0] if rep["warnings"] else ""), fg=self.ACCENT_GREEN if "GRIPPABLE" in ok_pad else self.ACCENT_AMBER)
        self.start_sim()

    @staticmethod
    def _grippable(key, pad):
        from .materials import truth
        t = truth(key, pad)
        return t["N_slip"] < t["N_max"]

    def _speed_changed(self, *_):
        try: self.speed_mult = max(0.1, float(self.speed_var.get().split("x")[0]))
        except Exception: self.speed_mult = 1.0

    def _init_persistent_plot_artists(self):
        self.ax_force.clear()
        self.ax_force.set_facecolor("#090d16")
        self.line_fz, = self.ax_force.plot([], [], color="#38bdf8", lw=1.8, label="f_z Normal (N)")
        self.line_fy, = self.ax_force.plot([], [], color="#22c55e", lw=1.6, label="f_y Shear (N)")
        self.line_fd, = self.ax_force.plot([], [], color="#f43f5e", ls="--", lw=1.5, label="f_d Target (N)")
        self.line_nslip = self.ax_force.axhline(0.0, color="#e2e8f0", ls=":", lw=1.2, label="N_slip")
        self.line_nmax = self.ax_force.axhline(40.0, color="#c084fc", ls=":", lw=1.5, label="N_max")
        self.ax_force.set_ylabel("Force / Finger (N)", color="#94a3b8", fontsize=8.5)
        self.ax_force.tick_params(colors="#94a3b8", labelsize=8)
        for sp in self.ax_force.spines.values(): sp.set_color("#1e293b")
        self.ax_force.legend(loc="upper left", facecolor="#111827", edgecolor="#1f2937", labelcolor="#f8fafc", fontsize=7.5, ncol=3)
        self.ax_force.grid(True, color="#1e293b", linestyle="--", alpha=0.6)

        self.ax_stress.clear()
        self.ax_stress.set_facecolor("#090d16")
        self.line_p0, = self.ax_stress.plot([], [], color="#38bdf8", lw=1.3, ls="--", label="Peak Hertz Traction p0 (kPa)")
        self.line_vm, = self.ax_stress.plot([], [], color="#f43f5e", lw=1.9, label="Von Mises Stress σ_vM (kPa)")
        self.line_tr, = self.ax_stress.plot([], [], color="#f59e0b", lw=1.3, label="Tresca Shear σ_Tr (kPa)")
        self.line_rk, = self.ax_stress.plot([], [], color="#c084fc", lw=1.3, label="Rankine Tensile σ_Rk (kPa)")
        self.line_sy = self.ax_stress.axhline(0.0, color="#ef4444", ls=":", lw=1.5, visible=False, label="Yield Limit σ_Y")
        self.ax_stress.set_xlabel("Simulation Time (s)", color="#94a3b8", fontsize=8.5)
        self.ax_stress.set_ylabel("Stress (kPa)", color="#94a3b8", fontsize=8.5)
        self.ax_stress.tick_params(colors="#94a3b8", labelsize=8)
        for sp in self.ax_stress.spines.values(): sp.set_color("#1e293b")
        self.ax_stress.legend(loc="upper left", facecolor="#111827", edgecolor="#1f2937", labelcolor="#f8fafc", fontsize=7.5, ncol=2)
        self.ax_stress.grid(True, color="#1e293b", linestyle="--", alpha=0.6)
        self._episode_setup_needed = True

    def _ui_poll_loop(self):
        if self._dirty_ui and self._latest_ep is not None:
            self._dirty_ui = False
            try: self.update_panel_and_plot(self._latest_ep, self._latest_strat)
            except Exception as e:
                if repr(e) != self._last_err:
                    import traceback; traceback.print_exc()
                    self._last_err = repr(e)
        self.root.after(66, self._ui_poll_loop)

    def _mech_compute(self, ep):
        ms, o = ep.ms, ep.o
        if ms.get('N', 0.0) <= 1e-6: return
        self._np_ctx = dict(key=ep.obj_key, o=o, ms=ms, N=ms['N'], pad=ep.pad_key, n_f=int(ep.plant.n_act),
                            s_curl=float(ep.plant.s_of(ep.plant.xf[0])))

    def update_mech(self, ep):
        now = time.perf_counter()
        if not ep.done and now - self._mech_t < 0.6: return
        self._mech_t = now
        self._mech_compute(ep)
        ctx = getattr(self, '_np_ctx', None)
        if ctx is None: return
        txt = f"{ctx['o']['label']}  |  pad force N = {ctx['N']:.2f} N  (peak-loaded finger)\n\n"
        try: txt += NP.info_text(ctx, self._np_par())
        except Exception as e: txt += f"[analysis report unavailable: {e!r}]"
        top = self.mech_text.yview()[0]
        self.mech_text.config(state="normal"); self.mech_text.delete("1.0", tk.END); self.mech_text.insert("1.0", txt)
        self.mech_text.config(state="disabled"); self.mech_text.yview_moveto(top)
        if getattr(self, '_np_win', None) is not None and ep.done: self._np_draw()

    # ---------------- analysis graphs window ----------------
    def _np_par(self):
        v = getattr(self, '_np_vars', None)
        if not v: return dict(NP.DEFAULTS)
        return dict(theta=float(v['theta'].get()), phi=float(v['phi'].get()), comp=v['comp'].get(), fsec=v['fsec'].get(),
                    theory=v['theory'].get(), fos=float(v['fos'].get()))

    def open_np_window(self):
        if getattr(self, '_np_win', None) is not None:
            try: self._np_win.lift(); self._np_draw(); return
            except tk.TclError: self._np_win = None
        w = tk.Toplevel(self.root); w.title("Stress, structure and failure analysis at the current grasp"); w.configure(bg=self.BG_CARD)
        w.geometry("1380x860")
        fig = Figure(figsize=(14.0, 7.6), dpi=100, facecolor=self.BG_CARD)
        D = NP.DEFAULTS
        self._np_vars = dict(theta=tk.DoubleVar(value=D['theta']), phi=tk.DoubleVar(value=D['phi']), comp=tk.StringVar(value=D['comp']),
                             fsec=tk.StringVar(value=D['fsec']), theory=tk.StringVar(value=D['theory']), fos=tk.DoubleVar(value=D['fos']))
        self._np_page = 0
        bar = tk.Frame(w, bg=self.BG_CARD); bar.pack(fill=tk.X, side=tk.BOTTOM)
        for i, (nm, _) in enumerate(NP.PAGES):
            tk.Button(bar, text=nm, command=lambda i=i: self._np_set_page(i), bg="#1e293b", fg=self.ACCENT_BLUE, relief="flat").pack(side=tk.LEFT, fill=tk.X, expand=True)
        tk.Button(bar, text="REFRESH", command=self._np_draw, bg="#1e293b", fg=self.ACCENT_AMBER, relief="flat").pack(side=tk.LEFT)
        ctl = tk.Frame(w, bg=self.BG_CARD); ctl.pack(fill=tk.X, side=tk.TOP, padx=6, pady=4)
        def lab(t): tk.Label(ctl, text=t, bg=self.BG_CARD, fg=self.TEXT_MUTED, font=("Consolas", 8, "bold")).pack(side=tk.LEFT, padx=(8, 2))
        def scale(var, lo, hi, step):
            s = tk.Scale(ctl, variable=var, from_=lo, to=hi, resolution=step, orient=tk.HORIZONTAL, length=110, bg=self.BG_CARD, fg=self.TEXT_MAIN,
                         highlightthickness=0, troughcolor="#1e293b", font=("Consolas", 7), command=lambda _=None: self._np_draw()); s.pack(side=tk.LEFT)
        def combo(var, vals, width):
            c = ttk.Combobox(ctl, textvariable=var, values=list(vals), width=width, state="readonly"); c.pack(side=tk.LEFT)
            c.bind("<<ComboboxSelected>>", lambda _e: self._np_draw())
        lab("PLANE theta"); scale(self._np_vars['theta'], 0, 90, 1)
        lab("phi"); scale(self._np_vars['phi'], 0, 360, 5)
        lab("BEAM"); combo(self._np_vars['comp'], NP.COMPONENTS, 9)
        lab("FINGER SECTION"); combo(self._np_vars['fsec'], NP.FINGER_SECTIONS, 24)
        lab("FAILURE THEORY"); combo(self._np_vars['theory'], NP.THEORIES, 12)
        lab("FoS"); scale(self._np_vars['fos'], 1.0, 4.0, 0.1)
        cv = FigureCanvasTkAgg(fig, master=w); cv.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self._np_win, self._np_fig, self._np_cv = w, fig, cv
        w.protocol("WM_DELETE_WINDOW", lambda: (w.destroy(), setattr(self, '_np_win', None)))
        self._np_draw()

    def _np_set_page(self, i):
        self._np_page = i
        self._np_draw()

    def _np_draw(self):
        ctx = getattr(self, '_np_ctx', None)
        if getattr(self, '_np_win', None) is None or ctx is None: return
        try:
            NP.draw_page(self._np_fig, self._np_page, ctx, self._np_par(), dict(bg=self.BG_CARD, fg="#e2e8f0", grid="#334155"))
            self._np_fig.suptitle(f"{ctx['o']['label']}   N = {ctx['N']:.2f} N   [{NP.PAGES[self._np_page][0]}]", color="#f8fafc", fontsize=10)
            self._np_cv.draw_idle()
        except Exception:
            import traceback; traceback.print_exc()

    def toggle_pause(self): self.node.paused = not self.node.paused
    def stop_sim(self): self.node.stop_req = True

    def start_sim(self):
        params = dict(obj_key=self.obj_var.get(), strat_label=self.strat_var.get(),
                      pad='small' if 'small' in self.pad_var.get() else 'large',
                      n=max(3, min(5, int(self.fing_var.get()))))
        self.node.stop_req = True                    # asks a running episode to finish; the new thread waits for it
        self._init_persistent_plot_artists()
        threading.Thread(target=self.run_episode, args=(params,), daemon=True).start()

    def _verdict_tag(self, v):
        return "ok" if v == "OK" else ("warn" if v in ("NOT LIFTED", "ABORT") else "bad")

    def add_result_row(self, obj, mode, n, verdict, peak, fos, lift_mm, m_est_g):
        self._res_count += 1
        self.res_tree.insert("", tk.END, values=(self._res_count, obj, mode, n, verdict, f"{peak:.2f}", fmt_fos(fos), f"{lift_mm:.1f}", f"{m_est_g:.0f}"), tags=(self._verdict_tag(verdict),))
        kids = self.res_tree.get_children()
        if kids: self.res_tree.see(kids[-1])

    def log_episode(self, ep):
        if getattr(ep, "_logged", False) or not ep.plan["ok"]: return
        ep._logged = True
        m_est = (ep.out["m_est"] * 1000.0) if ep.out else 0.0
        self.add_result_row(ep.obj_key if hasattr(ep, "obj_key") else self.obj_var.get(), ep.strat_key if hasattr(ep, "strat_key") else self.strat_var.get().split()[0].lower(), ep.plant.n, ep.verdict(), ep.peak_N, ep.min_fos, ep.plant.zo * 1000.0, m_est)

    def update_finger_table(self, pl, ep=None):
        n = pl.n
        kids = self.fing_tree.get_children()
        if len(kids) != n + 1:
            for k in kids: self.fing_tree.delete(k)
            for i in range(n): self.fing_tree.insert("", tk.END, iid=f"f{i}", values=())
            self.fing_tree.insert("", tk.END, iid="fsum", values=(), tags=("sum",))
        for i in range(n):
            tq = pl.torques(i)
            touching = pl.N[i] > 0.0
            self.fing_tree.item(f"f{i}", values=(f"F{i+1}", "YES" if touching else "-", (f"{ep.dome_gap[i]*1000:+.2f}" if ep is not None else ""), f"{pl.N[i]:.2f}", f"{tq[0]:.2f}", f"{tq[1]:.2f}", f"{tq[2]:.2f}", f"{pl.s_of(pl.xf[i]):.2f}"), tags=("touch" if touching else "free",))
        self.fing_tree.item("fsum", values=("TOTAL", f"{pl.n_contact()}/{n}", "", f"{sum(pl.N):.2f}", "", "", "", ""), tags=("sum",))

    def start_benchmark(self):
        if self._bench_running: return
        self._bench_running = True
        self.btns["▦ BENCHMARK ALL"].config(state=tk.DISABLED, text="▦ RUNNING...")
        self.nb.select(2)
        threading.Thread(target=self._benchmark_worker, daemon=True).start()

    def _benchmark_worker(self):
        try:
            if self.node.running:
                self.node.stop_req = True
                t0 = time.time()
                while self.node.running and time.time() - t0 < 4.0: time.sleep(0.05)
            n = max(3, min(5, int(self.fing_var.get())))
            pad = "small" if "small" in self.pad_var.get() else "large"
            for k in list(OBJECTS.keys()):
                for strat in ("adaptive", "fixed"):
                    try:
                        ep = run_headless(k, pad, strat, self.node.seed, n)
                        if not ep.plan["ok"]:
                            row = (k, strat, n, "CANNOT GRASP", 0.0, float("nan"), 0.0, 0.0)
                        else:
                            row = (k, strat, n, ep.verdict(), ep.peak_N, ep.min_fos, ep.plant.zo * 1000.0, (ep.out["m_est"] * 1000.0) if ep.out else 0.0)
                    except Exception as e:
                        row = (k, strat, n, "ERROR", 0.0, float("nan"), 0.0, 0.0)
                    self.root.after(0, lambda r=row: self.add_result_row(*r))
        finally:
            self._bench_running = False
            self.root.after(0, lambda: self.btns["▦ BENCHMARK ALL"].config(state=tk.NORMAL, text="▦ BENCHMARK ALL"))

    def run_episode(self, params):
        t0 = time.time()
        while self.node.running and time.time() - t0 < 4.0: time.sleep(0.02)
        self.node.running, self.node.stop_req, self.node.paused = True, False, False
        try: self._run_episode(params)
        except Exception:
            import traceback; traceback.print_exc()
        finally: self.node.running = False

    def _run_episode(self, params):
        obj_key, strat_label, n = params['obj_key'], params['strat_label'], params['n']
        o = OBJECTS[obj_key]
        try:
            from .physics_check import verified_plan
            verified_plan(obj_key, n)            # MuJoCo rigid-contact proof; replaces the plan by the first alternative that really lifts
        except Exception as exc:
            print('physics check skipped:', exc)
        ep = Episode(obj_key, params['pad'], 'adaptive' if 'Adaptive' in strat_label else 'fixed', self.node.seed, n)
        if not ep.plan['ok']:
            self.node.ensure_scene(n, obj_key)                      # show the part and the hand at home, never a stale / previous scene
            self.node.sync_frame(ep.tcp0[0], ep.tcp0[1], [0.0] * n, o, 0.0, 0.0, False, 0.0, [0.0] * n)
            self._latest_ep, self._latest_strat, self._dirty_ui = ep, strat_label, True
            return

        self.node.ensure_scene(n, obj_key)
        self.node.sync_frame(ep.tcp_cmd[0], ep.tcp_cmd[1], ep.plant.s_list(), o, 0.0, 0.0, False, ep.yaw_cmd, ep.plan['sigmas'])

        SIM_DT, FRAME_DT = 0.01, 1.0 / 60.0
        acc, last = 0.0, time.perf_counter()
        while ep.t < 90.0 and not self.node.stop_req:
            if self.node.paused:
                while self.node.paused and not self.node.stop_req: time.sleep(0.03)
                last = time.perf_counter()
                continue
            t_frame = time.perf_counter()
            mult = self.speed_mult                                  # 1.0 = simulated time runs exactly at wall-clock speed
            acc = min(acc + (t_frame - last) * mult, 0.25 * max(mult, 1.0))
            last = t_frame
            nt = int(acc / SIM_DT)
            acc -= nt * SIM_DT
            for _ in range(nt):
                ep.tick()
                if ep.done: break
            if nt:
                self.node.sync_frame(ep.tcp_cmd[0], ep.tcp_cmd[1], ep.plant.s_list(), o, 0.0, ep.plant.zo, ep.crushed, ep.yaw_cmd, ep.plan['sigmas'], ep)
                self._latest_ep, self._latest_strat, self._dirty_ui = ep, strat_label, True
            if ep.done: break
            sleep_s = FRAME_DT - (time.perf_counter() - t_frame)
            if sleep_s > 0.001: time.sleep(sleep_s)

    def update_panel_and_plot(self, ep, strat_label):
        if not ep.plan['ok']:
            self.panel_frame.config(highlightbackground=self.ACCENT_AMBER)
            self.result_lbl.config(text="CANNOT GRASP: " + '; '.join(sorted(set(ep.plan['msgs']))[:2]), fg=self.ACCENT_AMBER)
            return
        if ep.out is None: return

        o, out, ms, tr, pl = ep.o, ep.out, ep.ms, ep.tr, ep.plant
        active_key = "HOLD" if out['state'] in ("HOLD", "DONE") else out['state']
        for st, pill in self.stage_pills.items():
            pill.config(bg=self.ACCENT_BLUE if st == active_key else ("#3f1d24" if out['state'] == "ABORT" else "#1e293b"), fg="#090d16" if st == active_key else ("#f87171" if out['state'] == "ABORT" else "#64748b"))
        self.arm_badge.config(text=f"ARM: {ep.arm_stage:<7} | t = {ep.t:5.2f}s")
        
        self.kpi_vars["fz"].set(f"{out['fz']:.2f} N")
        self.kpi_vars["fos"].set(fmt_fos(ms['fos']))
        self.kpi_vars["vm"].set(fmt_stress(ms['sig_vm']))
        self.kpi_vars["lift"].set(f"{pl.zo*1000:.1f} mm")
        self.kpi_boxes["fos"].config(fg=self.ACCENT_GREEN if ms['fos'] >= 1.5 else (self.ACCENT_AMBER if ms['fos'] >= 1.0 else self.ACCENT_RED))
        
        self.stick_bar["value"] = max(0.0, min(100.0, ms['c_over_a'] * 100.0))
        self.stick_lbl.config(text=f"Mindlin Stick Zone (c/a): {self.stick_bar['value']:4.0f}%")
        self.torque_bar["value"] = min(100.0, (pl.torques(0)[0] / max(HD.HAND['tau_max'][0], 1e-6)) * 100.0)
        self.torque_lbl.config(text=f"MCP Peak Torque Load: {self.torque_bar['value']:4.0f}% ({pl.torques(0)[0]:.2f} Nm)")

        self.telemetry_lbl.config(text=(
            f"SPECIMEN : {o['label']}  |  {ep.pad_key} pad  |  {o['mode']}\n"
            f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
            f"▸ 1. BLIND TACTILE CONTROLLER\n"
            f"  Force f_z / f_y / f_d : {out['fz']:5.2f} / {out['fy']:5.2f} / {out['fd']:5.2f} N  (f0={out['f0'] or 0.0:.2f}N)\n"
            f"  Slip Gate s_c / alpha : {out['s_c']} / {out['alpha']:4.2f}   [ramp={out['ramp']}  probe={out['probe']}]\n"
            f"  Probed E* / Force Cap : {fmt_modulus(out['Es_est'])}  ->  Cap {out['N_cap']:.1f} N\n"
            f"  Inferred E_obj / R*    : {'> ' if out['E_obj_lb'] else ''}{fmt_modulus(out['E_obj_est'])}{' (pad-limited: stiff part)' if out['E_obj_lb'] else ''} / {out['Re_est']*1000:.1f} mm\n"
            f"  Inferred Mass / mu_lb : {out['m_est']*1000:4.0f} g (true {o['m']*1000:.0f} g)  |  mu >= {out['mu_lb']:.2f}\n\n"
            f"▸ 2. HERTZIAN CONTACT & MINDLIN TRACTIONS\n"
            f"  Composite E*, R*      : {fmt_modulus(ms['Es'])},  R* = {ms['Re']*1000:.1f} mm\n"
            f"  Indentation d, a, e_c : d = {ms['delta']*1000:5.3f} mm, a = {ms['a']*1000:5.2f} mm, e_c = {ms['eps_c']*100:4.2f}%\n"
            f"  Normal p0 / p_mean    : {fmt_stress(ms['p0'])} / {fmt_stress(ms['p_mean'])}\n"
            f"  Shear tau_m / edge    : {fmt_stress(ms['tau_mean'])} / {fmt_stress(ms['tau_edge'])}\n\n"
            f"▸ 3. SUB-SURFACE CAUCHY STRESS (z = 0.48a)\n"
            f"  Principal (sr, sz)    : {fmt_stress(ms['sig1'])},  {fmt_stress(ms['sig3'])}\n"
            f"  Von Mises sig_vM      : {fmt_stress(ms['sig_vm'])}  (Allow: {fmt_stress(ms['sig_y_allow'])})\n"
            f"  Tresca / Rankine      : {fmt_stress(ms['sig_tresca'])} / {fmt_stress(ms['sig_rankine'])}\n"
            f"  Arm vs Object Lift    : {ep.rise*1000:5.1f} mm / {pl.zo*1000:5.1f} mm  (slip: {-(ep.rise - pl.zo)*1000.0:4.1f} mm)\n\n"
            f"▸ 4. DEXTEROUS HAND MECHANICS ({pl.n}-Finger Synergy)\n"
            f"  Synergy Curl s / Tilt : {pl.s_of(pl.mean_xf()):4.2f} / {math.degrees(pl.psi_of(pl.mean_xf())):4.0f} deg   (Contacts: {pl.n_contact()}/{pl.n})\n"
            f"  Torques MCP/PIP/DIP   : {pl.torques(0)[0]:4.2f} / {pl.torques(0)[1]:4.2f} / {pl.torques(0)[2]:4.2f} Nm  (defl {pl.deflect*1000:4.2f} mm)\n"
            f"  Finger / Arm TCP speed: {pl.close_speed()*1000:4.1f} mm/s / {ep.tcp_speed*1000:5.1f} mm/s  (lift paused for slip: {'YES' if ep.lift_held else 'no'})\n"
            f"  Gravity W vs n*mu*N   : {o['m']*G:5.2f} N  vs  {sum(pl.N)*o['mu']:6.2f} N  (Margin x{(sum(pl.N)*o['mu']/(o['m']*G) if o['m']*G>0 else 0):.1f})"
        ))

        self.update_finger_table(pl, ep)
        self.update_mech(ep)
        if ep.done: self.log_episode(ep)

        if ep.crushed:
            self.panel_frame.config(highlightbackground=self.ACCENT_RED)
            self.result_lbl.config(text=f"✖ CRUSHED / YIELDED  (min FoS={fmt_fos(ep.min_fos)}, Peak {ep.peak_N:.1f}N)", bg="#450a0a", fg="#fca5a5")
        elif out['done']:
            if out['abort']:
                self.panel_frame.config(highlightbackground=self.ACCENT_AMBER)
                self.result_lbl.config(text=f"⚠ ABORTED: {out['hint']} (Safety Cap Reached)", bg="#451a03", fg="#fcd34d")
            elif pl.zo / GRIPPER['lift_height'] >= 0.5:
                self.panel_frame.config(highlightbackground=self.ACCENT_GREEN)
                self.result_lbl.config(text=f"✔ GRASP & LIFT SUCCESS  (Peak {ep.peak_N:.2f}N, min FoS={fmt_fos(ep.min_fos)})", bg="#052e16", fg="#86efac")
            elif ep.lifted_max / GRIPPER['lift_height'] >= 0.2:
                self.panel_frame.config(highlightbackground=self.ACCENT_RED)
                self.result_lbl.config(text="✖ OBJECT SLIPPED OFF DURING LIFT", bg="#450a0a", fg="#fca5a5")
            else:
                self.panel_frame.config(highlightbackground=self.ACCENT_RED)
                self.result_lbl.config(text="✖ FAILED TO LIFT OBJECT", bg="#450a0a", fg="#fca5a5")
        elif pl.zo > 0.005:
            self.panel_frame.config(highlightbackground=self.ACCENT_GREEN)
            self.result_lbl.config(text=f"▲ LIFTING SPECIMEN ({pl.zo*1000:.1f} mm, FoS={self.kpi_vars['fos'].get()})", bg="#052e16", fg="#86efac")
        else:
            self.panel_frame.config(highlightbackground=self.BORDER)
            self.result_lbl.config(text=f"● EXECUTING: {out['state']} ({ep.arm_stage})...", bg="#1e293b", fg=self.ACCENT_BLUE)

        now = time.perf_counter()
        if not ep.done and now - self._last_plot_t < 0.4: return
        self._last_plot_t = now
        m = min(len(ep.t_hist), len(ep.fz_hist), len(ep.fy_hist), len(ep.fd_hist), len(ep.p0_hist), len(ep.vm_hist), len(ep.tr_hist), len(ep.rk_hist))
        if m == 0: return
        step = max(1, m // 500)
        T, FZ, FY, FD = ep.t_hist[:m:step], ep.fz_hist[:m:step], ep.fy_hist[:m:step], ep.fd_hist[:m:step]
        P0, VM, TR, RK = ep.p0_hist[:m:step], ep.vm_hist[:m:step], ep.tr_hist[:m:step], ep.rk_hist[:m:step]
        if self._episode_setup_needed:
            self._episode_setup_needed = False
            self.line_nslip.set_ydata([tr['N_slip'], tr['N_slip']])
            if tr['N_max'] < 65.0:
                self.line_nmax.set_ydata([tr['N_max'], tr['N_max']])
                self.line_nmax.set_visible(True)
            else: self.line_nmax.set_visible(False)
            if ms['sig_y_allow'] is not None and ms['sig_y_allow'] < 1e6:
                self.line_sy.set_ydata([ms['sig_y_allow'] / 1e3, ms['sig_y_allow'] / 1e3])
                self.line_sy.set_visible(True)
            else: self.line_sy.set_visible(False)

        self.line_fz.set_data(T, FZ); self.line_fy.set_data(T, FY); self.line_fd.set_data(T, FD)
        self.ax_force.set_xlim(0.0, max(5.0, T[-1] + 0.5))
        self.ax_force.set_ylim(0.0, max(max(FZ, default=1.0), max(FD, default=1.0), tr['N_slip'] * 1.3, 2.0) * 1.15)
        self.ax_force.set_title(f"TACTILE FORCE & SLIP CONTROL  //  {pl.n}-Finger Hand on {o['label']}", color="#f8fafc", fontsize=9.5, fontweight="bold")
        
        self.line_p0.set_data(T, P0); self.line_vm.set_data(T, VM); self.line_tr.set_data(T, TR); self.line_rk.set_data(T, RK)
        self.ax_stress.set_xlim(0.0, max(5.0, T[-1] + 0.5))
        self.ax_stress.set_ylim(0.0, max(max(P0, default=1.0), max(VM, default=1.0), max(TR, default=1.0), 1.0) * 1.15)
        self.ax_stress.set_title(f"SUB-SURFACE CAUCHY STRESSES  //  Strain e_c = {ms['eps_c']*100:.2f}%  |  Stick Zone c/a = {ms['c_over_a']*100:.0f}%", color="#f8fafc", fontsize=9.5, fontweight="bold")
        self.canvas.draw_idle()

def main(args=None):
    rclpy.init(args=args)
    node = MuJoCoSimBridgeNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    gui = SimDashboardGUI(node)
    try: gui.root.mainloop()
    finally:
        node.close_viewer()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()
GRIPEOF

mkdir -p "$PKG/$(dirname grip_gz/controller_node.py)"
cat << 'GRIPEOF' > "$PKG/grip_gz/controller_node.py"
from .dashboard import main

if __name__ == '__main__':
    main()
GRIPEOF

mkdir -p "$PKG/$(dirname launch/grip.launch.py)"
cat << 'GRIPEOF' > "$PKG/launch/grip.launch.py"
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _setup(context):
    return [Node(package='grip_gz', executable='grip_dashboard', output='screen', parameters=[{
        'object': LaunchConfiguration('object').perform(context),
        'pad': LaunchConfiguration('pad').perform(context),
        'seed': int(LaunchConfiguration('seed').perform(context)),
        'cad': LaunchConfiguration('cad').perform(context),
        'cad_material': LaunchConfiguration('cad_material').perform(context),
        'cad_unit': LaunchConfiguration('cad_unit').perform(context),
        'cad_args': LaunchConfiguration('cad_args').perform(context),
        'fingers': int(LaunchConfiguration('fingers').perform(context)),
    }])]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('object', default_value='apple'),
        DeclareLaunchArgument('pad', default_value='small'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('cad', default_value=''),
        DeclareLaunchArgument('cad_material', default_value='aluminium'),
        DeclareLaunchArgument('cad_unit', default_value='mm'),
        DeclareLaunchArgument('cad_args', default_value=''),
        DeclareLaunchArgument('fingers', default_value='5'),
        OpaqueFunction(function=_setup),
    ])
GRIPEOF

echo "grip_gz (MuJoCo + MoS + CAD, v3) written to: $PKG"

cd "$WS"
rm -rf "$WS/build/grip_gz" "$WS/install/grip_gz"
colcon build --packages-select grip_gz || exit 1
source install/setup.bash


if [ -n "$CAD_IN" ]; then
  echo "--- CAD analysis ---"
  ros2 run grip_gz grip_cad "$CAD_IN" --material "$CAD_MAT" --unit "$CAD_UNIT" "${CAD_EXTRA[@]}" || exit 1
  CAD_ARGS_STR=""
  if [ ${#CAD_EXTRA[@]} -gt 0 ]; then CAD_ARGS_STR="$(printf '%q ' "${CAD_EXTRA[@]}")"; fi
  ros2 launch grip_gz grip.launch.py cad:="$CAD_IN" cad_material:="$CAD_MAT" cad_unit:="$CAD_UNIT" cad_args:="$CAD_ARGS_STR" fingers:="$FINGERS"
else
  ros2 launch grip_gz grip.launch.py object:="${OBJECT:-apple}" pad:="${PAD:-small}" fingers:="$FINGERS"
fi
