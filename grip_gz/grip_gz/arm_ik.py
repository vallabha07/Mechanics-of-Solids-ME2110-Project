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
