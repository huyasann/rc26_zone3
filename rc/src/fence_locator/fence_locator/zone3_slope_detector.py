from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np


@dataclass
class SlopeSample:
    t: float
    x: float
    y: float
    z: float
    yaw: float
    pitch_abs: float


@dataclass
class SlopeFit:
    root_x: float
    root_y: float
    root_yaw: float
    ramp_yaw: float
    low: SlopeSample
    top: SlopeSample
    z_gain: float
    xy_dist: float
    residual: float
    score: float
    source: str


@dataclass
class SlopeDetectorConfig:
    ramp_x: float = 2.275
    ramp_low_y: float = 1.30
    ramp_top_y: float = -0.20
    min_z_gain: float = 0.34
    max_z_gain: float = 0.70
    min_xy_dist: float = 0.70
    min_pitch_abs_deg: float = 12.0
    event_min_z_gain: float = 0.035
    event_min_pitch_abs_deg: float = 6.0
    event_backtrack_sec: float = 1.0
    window_sec: float = 35.0
    min_fit_points: int = 8
    strong_residual: float = 0.18
    weak_residual: float = 0.42


def _rot(yaw: float, x: float, y: float) -> tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * x - s * y, s * x + c * y


class SlopeTrajectoryDetector:
    """Odom ramp-span fitter used as an independent Zone3 validation branch."""

    def __init__(self, cfg: SlopeDetectorConfig | None = None) -> None:
        self.cfg = cfg or SlopeDetectorConfig()
        self.samples: deque[SlopeSample] = deque()
        self.ramp_event_since: float | None = None
        self.best_fit: SlopeFit | None = None
        self.reason = "waiting"

    def reset(self) -> None:
        self.samples.clear()
        self.ramp_event_since = None
        self.best_fit = None
        self.reason = "reset"

    def add_sample(self, sample: SlopeSample) -> SlopeFit | None:
        self.samples.append(sample)
        cutoff = sample.t - self.cfg.window_sec
        while self.samples and self.samples[0].t < cutoff:
            self.samples.popleft()
        fit = self.update()
        if fit is not None and (self.best_fit is None or fit.score > self.best_fit.score):
            self.best_fit = fit
        return self.best_fit

    def update(self) -> SlopeFit | None:
        pts = list(self.samples)
        if len(pts) < 18:
            self.reason = "too_few_odom"
            return None
        span = self._online_ramp_span(pts)
        if span is None:
            return None
        start, end = span
        fit = self._make_ramp_fit(pts, start, end)
        return fit

    def _online_ramp_span(self, pts: list[SlopeSample]) -> tuple[int, int] | None:
        if self.ramp_event_since is None:
            self._detect_ramp_event(pts)
        if self.ramp_event_since is None:
            return None

        event_i = next((i for i, p in enumerate(pts) if p.t >= self.ramp_event_since), None)
        if event_i is None:
            self.reason = "event_outside_window"
            return None
        back_t = self.ramp_event_since - self.cfg.event_backtrack_sec
        back_i = next((i for i, p in enumerate(pts) if p.t >= back_t), 0)
        start_hi = min(len(pts), event_i + 12)
        if start_hi <= back_i:
            self.reason = "bad_start_window"
            return None
        start = min(range(back_i, start_hi), key=lambda i: pts[i].z)
        if start + 8 >= len(pts):
            self.reason = "waiting_top"
            return None
        end = max(range(start + 1, len(pts)), key=lambda i: pts[i].z)
        if end <= start + 8:
            self.reason = "short_span"
            return None
        return start, end

    def _detect_ramp_event(self, pts: list[SlopeSample]) -> None:
        for i in range(10, len(pts)):
            base_i0 = max(0, i - 80)
            base = min(pts[base_i0:i], key=lambda p: p.z)
            z_gain = pts[i].z - base.z
            if z_gain < self.cfg.event_min_z_gain:
                continue
            local_pitch = max(p.pitch_abs for p in pts[max(0, i - 20):i + 1])
            if local_pitch < math.radians(self.cfg.event_min_pitch_abs_deg):
                continue
            self.ramp_event_since = base.t
            self.reason = "event"
            return
        self.reason = "no_event"

    def _make_ramp_fit(self, pts: list[SlopeSample], start: int, end: int) -> SlopeFit | None:
        low = pts[start]
        top = pts[end]
        z_gain = top.z - low.z
        xy_total = math.hypot(top.x - low.x, top.y - low.y)
        max_pitch = max(p.pitch_abs for p in pts[start:end + 1])
        if z_gain < self.cfg.min_z_gain:
            self.reason = f"z_gain_low:{z_gain:.3f}"
            return None
        if z_gain > self.cfg.max_z_gain:
            self.reason = f"z_gain_high:{z_gain:.3f}"
            return None
        if xy_total < self.cfg.min_xy_dist:
            self.reason = f"xy_short:{xy_total:.3f}"
            return None
        if max_pitch < math.radians(self.cfg.min_pitch_abs_deg):
            self.reason = f"pitch_low:{math.degrees(max_pitch):.1f}"
            return None

        seg = pts[start:end + 1]
        xy = np.asarray([[p.x, p.y] for p in seg], dtype=np.float64)
        z = np.asarray([p.z for p in seg], dtype=np.float64)
        pitch = np.asarray([p.pitch_abs for p in seg], dtype=np.float64)
        mask = (z >= low.z + 0.04) & (z <= top.z - 0.015) & (pitch >= math.radians(5.0))
        if int(mask.sum()) < max(self.cfg.min_fit_points, 12):
            mask = (z >= low.z + 0.02) & (z <= top.z)
        if int(mask.sum()) < self.cfg.min_fit_points:
            self.reason = f"fit_points_low:{int(mask.sum())}"
            return None

        ramp_xy = xy[mask]
        center = ramp_xy.mean(axis=0)
        _, _, vh = np.linalg.svd(ramp_xy - center, full_matrices=False)
        axis = vh[0]
        low_to_top = np.asarray([top.x - low.x, top.y - low.y], dtype=np.float64)
        if float(np.dot(axis, low_to_top)) < 0.0:
            axis = -axis
        ramp_yaw = math.atan2(float(axis[1]), float(axis[0]))
        root_yaw = ramp_yaw + math.pi / 2.0

        proj = (ramp_xy - center) @ axis
        lo_p, hi_p = np.percentile(proj, [4.0, 96.0])
        low_xy = center + axis * lo_p
        top_xy = center + axis * hi_p
        fitted_low = SlopeSample(low.t, float(low_xy[0]), float(low_xy[1]), low.z, low.yaw, low.pitch_abs)
        fitted_top = SlopeSample(top.t, float(top_xy[0]), float(top_xy[1]), top.z, top.yaw, top.pitch_abs)

        top_dx, top_dy = _rot(root_yaw, self.cfg.ramp_x, self.cfg.ramp_top_y)
        low_dx, low_dy = _rot(root_yaw, self.cfg.ramp_x, self.cfg.ramp_low_y)
        root_top = (fitted_top.x - top_dx, fitted_top.y - top_dy)
        root_low = (fitted_low.x - low_dx, fitted_low.y - low_dy)
        root_x = 0.5 * (root_top[0] + root_low[0])
        root_y = 0.5 * (root_top[1] + root_low[1])
        residual = math.hypot(root_top[0] - root_low[0], root_top[1] - root_low[1])
        if residual > self.cfg.weak_residual:
            self.reason = f"residual_high:{residual:.3f}"
            return None
        source = "slope_strong" if residual <= self.cfg.strong_residual else "slope_weak"
        score = z_gain - 2.0 * residual
        self.reason = source
        return SlopeFit(
            root_x=root_x,
            root_y=root_y,
            root_yaw=root_yaw,
            ramp_yaw=ramp_yaw,
            low=fitted_low,
            top=fitted_top,
            z_gain=z_gain,
            xy_dist=float(hi_p - lo_p),
            residual=residual,
            score=score,
            source=source,
        )
