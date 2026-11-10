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
