"""hand_plant.py - Unconditionally stable compliant multi-finger contact & stick-slip physics."""
import math
import random
import numpy as np

from . import hand as H
from .arm_ik import tcp_home
from .controller_core import GripController, Params
from .materials import GRIPPER, OBJECTS, PADS, G, hertz_delta, truth
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
        ro = plan['r_open']
        self.xc = [max(0.002, ro - rho) for rho in plan['rhos']]
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
            if self.N[i] > 0.0: x = max(self.xc[i], x - self._defl(self.N[i], self.xc[i]))
            out.append(self.s_of(x))
        return out

    def mean_xf(self): return sum(self.xf) / self.n
    def n_contact(self): return sum(1 for v in self.N if v > 0.0)
    def torques(self, i=0): return H.joint_torques(self.N[i], self.s_of(self.xf[i]))
    def _xcv(self, xc):
        v = self._xc_cache.get(xc)
        if v is None:
            s = self.s_of(xc)
            v = (float(np.interp(s, self.ts, self.tFav)), float(np.interp(s, self.ts, self.tC)))
            self._xc_cache[xc] = v
        return v
    def _clip(self, F, xc): return min(F, self._xcv(xc)[0])
    def _defl(self, N, xc): return self._xcv(xc)[1] * N
    def close_speed(self): return max(self.vf) if self.vf else 0.0

    def step(self, F_cmd, vg, dt):
        mf, b = GRIPPER['finger_mass'], GRIPPER['finger_damping']
        o, tr = self.o, self.tr
        m, mu = o['m'], o['mu']
        n_pos, defl, lim, total_N = 0, 0.0, False, 0.0

        for i in range(self.n):
            xf, vf, xc = self.xf[i], self.vf[i], self.xc[i]
            N = 0.0
            if xf < xc or F_cmd <= 0.0:
                vf = (vf + (F_cmd / mf) * dt) / (1.0 + (b / mf) * dt)
                vlim = MOTION['v_close'] if (xc - xf) < MOTION['gap_slow'] else MOTION['v_approach']
                vf = max(-MOTION['v_open'], min(vlim, vf))                  # speed limits of the finger drive
                xf += vf * dt
                if xf >= xc and F_cmd > 0.0:
                    N = self._clip(F_cmd, xc)
                    lim = lim or (N < F_cmd - 1e-9)
                    xf, vf = xc + hertz_delta(N, tr['Re'], tr['Es']) + self._defl(N, xc), 0.0
                elif xf < 0.0: xf, vf = 0.0, 0.0
                elif xf > self.xmax: xf, vf = self.xmax, 0.0
            else:
                N = self._clip(F_cmd, xc)
                lim = lim or (N < F_cmd - 1e-9)
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
        fric_cap = (mu * self.KR if was_sliding else mu) * total_N

        denom = m + dt * (kt * dt + ct)
        rhs_force = -kt * (self.w - vg * dt) + ct * vg - m * G
        vo_stick = (m * self.vo + dt * rhs_force) / denom
        w_stick = self.w + (vo_stick - vg) * dt
        Ft_trial = -kt * w_stick - ct * (vo_stick - vg)

        if abs(Ft_trial) <= fric_cap:
            self.sat, Ft, vo_next, w_next = False, Ft_trial, vo_stick, w_stick
        else:
            self.sat = True
            Ft = (1.0 if Ft_trial >= 0.0 else -1.0) * (mu * self.KR * total_N)
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

    def N_mean(self): return sum(self.N) / self.n
    def sense(self, rng, bias):
        Nm = self.N_mean()
        sig = 0.015 * math.sqrt(2.0 / self.n)
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
        self.tr = truth(obj_key, pad_key, n=self.n)
        self.rng = random.Random(seed)
        self.ctrl = GripController(Params(strategy=strat_key, F_max=GRIPPER['F_max'], R_pad=pad['R'], E_pad=pad['E'], nu_pad=pad['nu']))
        self.ctrl.n_f = self.n
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
        self.T_over = max(MOTION['t_min_over'], pk * d_over / MOTION['v_transit'])
        self.T_down = max(MOTION['t_min_down'], pk * d_down / MOTION['v_final'])
        self.tcp_speed = 0.0
        self._tcp_prev = self.tcp0
        self.lift_v = 0.0
        self.lift_held = False
        self.t, self.t_app0, self.t_lift0 = 0.0, None, None
        self.arm_stage = 'HOME'
        self.tcp_cmd = self.tcp0
        self.rise, self.at_grasp, self.at_top, self.crushed = 0.0, False, False, False
        self.peak_N, self.peak_vm, self.lifted_max, self.min_fos = 0.0, 0.0, 0.0, 99.9
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
                self.lift_held = bool(out['ramp'] or out['s_c'])        # slip detected -> stop lifting until the grip is firm
                v_tgt = 0.0 if self.lift_held else self.LIFT_SPEED
                self.lift_v += max(-self.LIFT_ACCEL * 0.01, min(self.LIFT_ACCEL * 0.01, v_tgt - self.lift_v))
                vg = self.lift_v
                self.rise += vg * 0.01
                if self.rise >= GRIPPER['lift_height']: self.rise, self.at_top = GRIPPER['lift_height'], True
            self.tcp_cmd = (self.tcp_g[0], self.tcp_g[1] + self.rise)
        return vg

    def tick(self, dt=0.01, sub=10):
        pl = self.plant
        fzl, fzr, fyl, fyr, d_sens = pl.sense(self.rng, self.BIAS)
        self.ctrl.psi = 0.0
        out = self.ctrl.step(self.t, fzl, fzr, fyl, fyr, pl.mean_xf(), d_sens, self.at_grasp, self.at_top)
        vg = self._arm(out)
        if self.ctrl.state in ('CALIB', 'WAIT_ARM'):
            for i in range(self.n):
                pl.xf[i] = float(np.interp(H.preshape_curl(self.tcp_cmd[1], self.plan['s_clear'][i]), pl.ts, pl.tx))
                pl.vf[i] = 0.0
        self.min_clear = min(self.min_clear, min(self.tcp_cmd[1] + H.lowest_z(s) for s in pl.s_list()))
        sub_dt = dt / sub
        for _ in range(sub):
            pl.step(out['force'], vg, sub_dt)
            self.peak_N, self.lifted_max = max(self.peak_N, max(pl.N)), max(self.lifted_max, pl.zo)
        ms = mos_state(pl.N_mean(), pl.f_mean, self.obj_key, self.pad_key)      # once per tick
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

    def verdict(self):
        H_ = GRIPPER['lift_height']
        frac = self.plant.zo / H_
        if self.crushed: return 'CRUSHED'
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
            a, f = run_headless(k, 'small', 'adaptive', 0, n), run_headless(k, 'small', 'fixed', 0, n)
            print('%-13s %-12s %7.2f %8.3f %8.1f %8.0f %8s %6.1f mm' % (
                k, a.verdict(), a.peak_N, a.tr['N_slip'], a.tr['N_max'], a.out['m_est'] * 1000, f.verdict(), a.min_clear * 1000))

if __name__ == '__main__':
    main()
