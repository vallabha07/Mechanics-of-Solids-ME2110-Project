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
        P = self.P
        if self.K is not None and self.K > 0:
            self.Es_est = (3.0 * self.K) / (4.0 * math.sqrt(P.R_pad))
            Es_pad_only = P.E_pad / (1.0 - P.nu_pad ** 2)
            inv_obj = (1.0 / max(self.Es_est, 1.0)) - (1.0 / Es_pad_only)
            self.E_obj_est = min(250e9, (1.0 - 0.30 ** 2) / inv_obj) if inv_obj > 1e-9 else 50e9
        if delta > 1e-8 and self.fz > 1e-3:
            a_est = math.sqrt(P.R_pad * delta)
            self.eps_est = 0.2 * (a_est / P.R_pad)
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
            if self.fz > 1e-3:
                self.mu_lb = max(self.mu_lb, self.fy / self.fz)
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
                    alpha=self.alpha_d, p_e=self.p_e_mm, K=self.K, Es_est=self.Es_est, E_obj_est=self.E_obj_est,
                    eps_est=self.eps_est, p0_est=self.p0_est, vm_est=self.vm_est, N_cap=self.N_cap,
                    mu_lb=self.mu_lb, m_est=self.m_est, probe=int(self.probe_active),
                    ramp=int(self.ramp), ratio=self.last_ratio)
