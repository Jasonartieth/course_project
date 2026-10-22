"""Runtime safety monitor, evaluated every control step by the runner (real arm and simulation alike).

Abort conditions (any one stops the run; the runner then holds the last measured pose):
  * any joint current above I_INST * inst_frac                              (instantaneous overload)
  * any joint current above I_CONT * cont_frac for longer than cont_time_s  (sustained overload)
  * any motor temperature above temp_max_c                                  (manual: never exceed 50 degC)
  * measured joint outside [Q_MIN - pos_tol, Q_MAX + pos_tol]
  * |q_meas - q_ref| above track_err_rad for longer than track_time_s       (q_ref = profile-filtered goal)
  * stale or invalid data: non-finite values, or identical readings for stale_s seconds while moving
"""

from dataclasses import dataclass

import numpy as np

from . import spec


@dataclass
class SafetyConfig:
    inst_frac: float = 0.9
    cont_frac: float = 1.3
    cont_time_s: float = 1.5
    temp_max_c: float = 45.0
    pos_tol: float = np.deg2rad(2.0)
    track_err_rad: float = 0.15
    track_time_s: float = 0.5
    stale_s: float = 0.5


class SafetyMonitor:
    def __init__(self, cfg: SafetyConfig = SafetyConfig()):
        self.cfg = cfg
        self.over_cont_since = np.full(4, np.nan)
        self.track_since = np.full(4, np.nan)
        self.last_q = None
        self.same_since = np.nan

    def update(self, t, state, q_ref, dq_ref):
        """Return None if OK, otherwise a human-readable abort reason."""
        c = self.cfg
        q, cur, temp = state.q[:4], state.current[:4], state.temp
        if not (np.all(np.isfinite(q)) and np.all(np.isfinite(cur))):
            return "non-finite measurement"
        over = np.abs(cur) > spec.I_INST * c.inst_frac
        if over.any():
            j = int(np.argmax(over))
            return f"instantaneous current on {spec.JOINT_NAMES[j]}: {cur[j]:.2f} A > {spec.I_INST[j] * c.inst_frac:.2f} A"
        reason = self._timed(t, np.abs(cur) > spec.I_CONT * c.cont_frac, self.over_cont_since, c.cont_time_s)
        if reason is not None:
            return f"sustained current on {spec.JOINT_NAMES[reason]}: {cur[reason]:.2f} A for > {c.cont_time_s} s"
        if np.nanmax(temp) > c.temp_max_c:
            return f"motor temperature {np.nanmax(temp):.1f} C > {c.temp_max_c} C"
        if np.any(q < spec.Q_MIN - c.pos_tol) or np.any(q > spec.Q_MAX + c.pos_tol):
            return f"joint position out of range: {np.rad2deg(q).round(1)} deg"
        err = np.abs(q - q_ref)
        reason = self._timed(t, err > c.track_err_rad, self.track_since, c.track_time_s)
        if reason is not None:
            return f"tracking error on {spec.JOINT_NAMES[reason]}: {err[reason]:.3f} rad for > {c.track_time_s} s"
        moving = np.abs(dq_ref).max() > 0.05
        if self.last_q is not None and moving and np.array_equal(q, self.last_q):
            if np.isnan(self.same_since):
                self.same_since = t
            elif t - self.same_since > c.stale_s:
                return "measurements frozen while the reference is moving (I/O failure?)"
        else:
            self.same_since = np.nan
        self.last_q = q.copy()
        return None

    @staticmethod
    def _timed(t, cond, since, limit):
        since[~cond] = np.nan
        start = np.isnan(since) & cond
        since[start] = t
        late = cond & (t - since > limit)
        return int(np.argmax(late)) if late.any() else None
