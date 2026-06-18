#!/usr/bin/env python3
"""
Drive up the Challenge2 ramp while holding the startup yaw.

This script is intentionally standalone and owns /cmd_vel while running.
Do not run it together with Nav2, move_actions, orient_actions,
weapon_detect alignment, merlin_stairs, or keyboard drive tools.

用法: python3 ramp_drive_yaw_hold.py
所有配置见下方 "配置区域", 直接改值即可, 不需要传参.
"""

import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

# ════════════════════════════════════════════════════════════════════
# 配置区域 — 直接改这里, 不需要传参
# ════════════════════════════════════════════════════════════════════

# ── 通信 ──────────────────────────────────────────────────────────
ODOM_TOPIC = "/odin1/odometry_highfreq"    # 里程计订阅话题
CMD_VEL_TOPIC = "/cmd_vel"                 # 速度指令发布话题
CONTROL_HZ = 30.0                          # 控制循环频率 (Hz)
# 里程计 ~400Hz, 控制 30Hz 已足够。增大 → CPU 开销↑

# ── 行驶控制 ──────────────────────────────────────────────────────
FORWARD_SPEED_MPS = 0.20    # 前进速度 (m/s)
#   坡长≈1.5m → 爬坡 ≈ 1.5/0.20 = 7.5s, 余量充裕

YAW_KP = 3.0                # 偏航 P 控制器增益
#   测试中 ±4° 摆动, Kp=1.5→仅 0.105 rad/s 修正远不够
#   提升到 3.0→4° 时输出 0.21 rad/s, 响应快一倍
#   若振荡再降回 2.0

MAX_WZ_RADPS = 0.5          # 最大偏航角速度 (rad/s) ≈ 29°/s
#   |wz| 硬限幅。防止急转导致侧滑或机械过载

YAW_ABORT_DEG = 12.0        # 偏航偏离中止阈值 (度)
#   |yaw_error| ≥ 12° → 中止。防打滑/碰撞后方向失控继续行驶

TIMEOUT_SEC = 30.0          # 全局超时 (秒)
#   爬坡 7.5s + 登顶确认 max(0.5,2.0) + 安全余量 = 30s

ODOM_TIMEOUT_SEC = 0.5      # 里程计超时 (秒)
#   超过此时间未收到里程计 → odom_timeout 退出

# ── 上坡检测: 基础阈值 ────────────────────────────────────────────
PLATFORM_HEIGHT_M = 0.35    # 登顶判定高度 (m)
#   实际平台高 400mm (图册), 留 50mm 容错:
#     - odom z 可能有 ±20mm 噪声
#     - 平台实际高度可能有 ±10mm 施工误差
#     - 地毯压缩 ~10mm
#   设 350mm 确保登顶可靠触发, 不会在坡中间误判
#   标准路径 A: z_rise≥0.35m + pitch归平 持续0.5s → 登顶
#   快速路径 B: z_rise≥0.35m + dz/dt稳定 持续2.0s → 登顶

PITCH_TOL_RAD = 0.12        # 俯仰归平容差 (rad) ≈ 6.9°
#   登顶后 pitchΔ 应降至近 0, |pitchΔ| ≤ 6.9° → 标准路径触发
#   爬坡中 pitchΔ ≈ +15° (坡角), 远大于 tol→不会误判登顶 ✅

SUMMIT_CONFIRM_SEC = 0.5    # 标准登顶确认时间 (秒)
#   z≥0.35m + 俯仰归平 持续此时间 → 判定登顶 (路径 A)

Z_RISE_THR_M = 0.03         # z 上升判定阈值 (m)
#   z_sm > 3cm → "车在上升"。防地面噪声误判爬坡

CLIMB_PITCH_RATIO = 1.5     # 爬坡俯仰判定倍数
#   爬坡阈值 = 0.12×1.5=0.18rad(10.3°)
#   坡角 15° > 10.3° → 上坡必定触发爬坡状态 ✅

# ── 上坡检测: 滤波与防抖 ──────────────────────────────────────────
BASELINE_FRAMES = 20        # 基线校准帧数
#   启动时取前 N 帧 odom 均值做 ground 参考
#   @400Hz → 20帧仅 0.05s

SMOOTH_WINDOW = 10          # z_rise 平滑窗大小
#   z_sm = 最近 N 帧 z_rise 滑动均值

SUSTAIN_FRAMES = 5          # 状态切换持续帧数
#   GROUND↔CLIMBING 切换需连续 N 帧满足条件 (防抖)
#   @ 30Hz → 5帧 ≈ 0.17s

FAST_SUMMIT_RATIO = 1.0     # 快速登顶高度倍数
#   ⚠️ 设 1.0 (与标准路径同高), 确保快速路径可达
#   若设 >1.0 (如旧值 1.5→0.525m), 平台只有 0.40m,
#   快速路径永不可达 → 标准路径不归平则必定超时退出 ❌

FAST_SUMMIT_SEC = 2.0       # 快速登顶稳定时间 (秒)
#   z 稳定(平滑 dz/dt<0.01m/s) 持续此时间 → 快速判定登顶

# ── 登顶后蠕动 ──────────────────────────────────────────────────────
CREEP_DIST_M = 0.1          # 登顶后再前进距离 (m)
#   确保车屁股完全上平台, 不会卡在坡边沿
#   用 odom xy 距离检测, 准 → 提前退出, 不准 → 超时兜底

CREEP_MAX_M = 0.2            # 蠕动最大距离 (m)
#   超时兜底: 若 odom 不准导致 dist 一直 <0.1m,
#   最多走 CREEP_MAX_M / FORWARD_SPEED_MPS 秒后强制退出
#   确保不走超, 不会撞到前方

# ════════════════════════════════════════════════════════════════════


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_rpy(x: float, y: float, z: float, w: float):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class RampDriveYawHold(Node):
    GROUND = 0
    CLIMBING = 1
    SUMMIT = 2
    CREEPING = 3

    def __init__(self):
        super().__init__("ramp_drive_yaw_hold")

        # ── 从上方配置区域读取 (直接赋值, 无参数系统) ──
        self._odom_topic = ODOM_TOPIC
        self._cmd_vel_topic = CMD_VEL_TOPIC
        self._control_hz = CONTROL_HZ
        self._forward_speed = FORWARD_SPEED_MPS
        self._yaw_kp = YAW_KP
        self._max_wz = MAX_WZ_RADPS
        self._yaw_abort = math.radians(YAW_ABORT_DEG)
        self._timeout_sec = TIMEOUT_SEC
        self._odom_timeout_sec = ODOM_TIMEOUT_SEC
        self._platform_z = PLATFORM_HEIGHT_M
        self._pitch_tol = PITCH_TOL_RAD
        self._summit_confirm_sec = SUMMIT_CONFIRM_SEC
        self._baseline_frames = BASELINE_FRAMES
        self._climb_pitch_ratio = CLIMB_PITCH_RATIO
        self._smooth_window = SMOOTH_WINDOW
        self._sustain_frames = SUSTAIN_FRAMES
        self._z_rise_thr = Z_RISE_THR_M
        self._fast_summit_ratio = FAST_SUMMIT_RATIO
        self._fast_summit_sec = FAST_SUMMIT_SEC

        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self.create_subscription(Odometry, self._odom_topic, self._odom_cb, 20)

        self._x = 0.0
        self._y = 0.0
        self._z = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._last_odom_time = 0.0
        self._last_frame_time = 0.0

        self._z_buf = deque(maxlen=self._baseline_frames)
        self._pitch_buf = deque(maxlen=self._baseline_frames)
        self._yaw_sin_buf = deque(maxlen=self._baseline_frames)
        self._yaw_cos_buf = deque(maxlen=self._baseline_frames)
        self._baseline_ok = False
        self._z_ground = 0.0
        self._pitch_ground = 0.0
        self._target_yaw = 0.0

        self._z_smooth_buf = deque(maxlen=self._smooth_window)
        self._dz_dt_buf = deque(maxlen=20)
        self._z_rise_smooth = 0.0
        self._dz_dt_smooth = 0.0
        self._climb_counter = 0
        self._ground_counter = 0
        self._state = self.GROUND
        self._confirm_t0 = 0.0
        self._fast_summit_t0 = 0.0
        self._start_time = 0.0
        self._done = False

        # ── 登顶蠕动跟踪 ──
        self._creep_start_x = 0.0
        self._creep_start_y = 0.0
        self._creep_start_time = 0.0

        self.create_timer(1.0 / max(1.0, self._control_hz), self._control_loop)
        self.create_timer(1.0, self._status_timer)

        self.get_logger().info("ramp_drive_yaw_hold ready")
        self.get_logger().info(
            f"odom={self._odom_topic} cmd_vel={self._cmd_vel_topic} "
            f"vx={self._forward_speed:.3f} yaw_abort={math.degrees(self._yaw_abort):.1f}deg"
        )

    @property
    def done(self) -> bool:
        return self._done

    def stop(self) -> None:
        self._publish_cmd(0.0, 0.0)

    def _odom_cb(self, msg: Odometry) -> None:
        now = time.monotonic()
        self._last_odom_time = now
        dt = now - self._last_frame_time if self._last_frame_time else 0.0
        self._last_frame_time = now

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        x, y = float(p.x), float(p.y)
        _, pitch, yaw = quaternion_to_rpy(float(q.x), float(q.y), float(q.z), float(q.w))
        z = float(p.z)

        if not all(math.isfinite(v) for v in (x, y, z, pitch, yaw)):
            self._finish("invalid_odom")
            return

        self._x = x
        self._y = y
        self._z = z
        self._pitch = pitch
        self._yaw = yaw

        if not self._baseline_ok:
            self._z_buf.append(z)
            self._pitch_buf.append(pitch)
            self._yaw_sin_buf.append(math.sin(yaw))
            self._yaw_cos_buf.append(math.cos(yaw))
            if len(self._z_buf) >= self._baseline_frames:
                self._z_ground = sum(self._z_buf) / len(self._z_buf)
                self._pitch_ground = sum(self._pitch_buf) / len(self._pitch_buf)
                self._target_yaw = math.atan2(sum(self._yaw_sin_buf), sum(self._yaw_cos_buf))
                self._baseline_ok = True
                self._start_time = time.monotonic()
                self.get_logger().info(
                    f"baseline z={self._z_ground:.3f}m "
                    f"pitch={math.degrees(self._pitch_ground):+.1f}deg "
                    f"target_yaw={math.degrees(self._target_yaw):+.1f}deg"
                )
            return

        # 登顶/蠕动阶段不再更新上坡检测
        if self._state >= self.SUMMIT:
            return

        self._update_ramp_state(dt)

    def _update_ramp_state(self, dt: float) -> None:
        z_rise = self._z - self._z_ground
        pitch_delta = self._pitch - self._pitch_ground
        pd_abs = abs(pitch_delta)

        self._z_smooth_buf.append(z_rise)
        z_sm = sum(self._z_smooth_buf) / len(self._z_smooth_buf)

        dz_dt = 0.0
        if dt > 0.0:
            dz_dt = (z_sm - self._z_rise_smooth) / dt
        self._z_rise_smooth = z_sm
        self._dz_dt_buf.append(dz_dt)
        self._dz_dt_smooth = sum(self._dz_dt_buf) / len(self._dz_dt_buf)

        climb_pitch_thr = self._pitch_tol * self._climb_pitch_ratio
        z_at_platform = z_rise >= self._platform_z
        pitch_level = pd_abs <= self._pitch_tol

        if z_at_platform and pitch_level:
            if self._confirm_t0 == 0.0:
                self._confirm_t0 = time.monotonic()
            elif time.monotonic() - self._confirm_t0 >= self._summit_confirm_sec:
                self._state = self.SUMMIT
                self.get_logger().info(
                    f"⬆️登顶(标准) z_rise={z_rise:.3f}m pitchΔ={math.degrees(pitch_delta):+.1f}°"
                )
            self._fast_summit_t0 = 0.0
            return
        self._confirm_t0 = 0.0

        z_well_above = z_rise >= self._platform_z * self._fast_summit_ratio
        z_plateaued = abs(self._dz_dt_smooth) < 0.01
        if z_well_above and z_plateaued:
            if self._fast_summit_t0 == 0.0:
                self._fast_summit_t0 = time.monotonic()
            elif time.monotonic() - self._fast_summit_t0 >= self._fast_summit_sec:
                self._state = self.SUMMIT
                self.get_logger().info(
                    f"⬆️登顶(快速) z_rise={z_rise:.3f}m dz/dt_s={self._dz_dt_smooth:.4f}m/s"
                )
            return
        self._fast_summit_t0 = 0.0

        on_slope = pd_abs > climb_pitch_thr and z_sm > self._z_rise_thr
        on_flat = z_sm < self._z_rise_thr * 0.5 and pd_abs < self._pitch_tol

        if on_slope:
            self._climb_counter += 1
            self._ground_counter = 0
        else:
            self._climb_counter = 0
            self._ground_counter = self._ground_counter + 1 if on_flat else 0

        if on_slope and self._climb_counter >= self._sustain_frames:
            self._state = self.CLIMBING
        elif on_flat and self._ground_counter >= self._sustain_frames:
            self._state = self.GROUND

    def _control_loop(self) -> None:
        if self._done:
            return

        now = time.monotonic()
        if not self._baseline_ok:
            self._publish_cmd(0.0, 0.0)
            return

        if now - self._last_odom_time > self._odom_timeout_sec:
            self._finish("odom_timeout")
            return

        elapsed = now - self._start_time
        if elapsed >= self._timeout_sec:
            self._finish("timeout")
            return

        # ── 蠕动阶段: 再往前走一点确保完全上坡 ──
        if self._state == self.CREEPING:
            self._creep_tick(now)
            return

        # ── 刚登顶 → 启动蠕动 ──
        if self._state == self.SUMMIT:
            self._start_creep(now)
            return

        # ── 正常上坡: 偏航保持 ──
        yaw_error = normalize_angle(self._target_yaw - self._yaw)
        if abs(yaw_error) >= self._yaw_abort:
            self._finish(f"yaw_abort_{math.degrees(yaw_error):+.1f}deg")
            return

        wz = clamp(self._yaw_kp * yaw_error, -self._max_wz, self._max_wz)
        self._publish_cmd(self._forward_speed, wz)

    def _publish_cmd(self, vx: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self._cmd_pub.publish(msg)

    def _start_creep(self, now: float) -> None:
        """登顶后启动蠕动: 再前进 CREEP_DIST_M, odom 不准则超时退出."""
        self._creep_start_x = self._x
        self._creep_start_y = self._y
        self._creep_start_time = now
        self._state = self.CREEPING
        creep_timeout = CREEP_MAX_M / self._forward_speed
        self.get_logger().info(
            f"🚶蠕动前进 {CREEP_DIST_M:.1f}m timeout={creep_timeout:.1f}s ..."
        )
        self._creep_tick(now)

    def _creep_tick(self, now: float) -> None:
        """蠕动控制 tick: odom 距离优先, 超时兜底."""
        dx = self._x - self._creep_start_x
        dy = self._y - self._creep_start_y
        dist = math.sqrt(dx * dx + dy * dy)
        elapsed = now - self._creep_start_time
        creep_cap = CREEP_MAX_M / self._forward_speed  # 最多走 CREEP_MAX_M

        if dist >= CREEP_DIST_M:
            self.get_logger().info(f"✅蠕动完成 dist={dist:.3f}m")
            self._finish("summit_ok")
        elif elapsed >= creep_cap:
            self.get_logger().warn(
                f"⚠️蠕动超时 dist={dist:.3f}m<{CREEP_DIST_M}m 里程计可能不准, 强制退出"
            )
            self._finish("summit_creep_timeout")
        else:
            yaw_error = normalize_angle(self._target_yaw - self._yaw)
            if abs(yaw_error) >= self._yaw_abort:
                self._finish(
                    f"creep_yaw_abort_{math.degrees(yaw_error):+.1f}deg"
                )
                return
            wz = clamp(self._yaw_kp * yaw_error, -self._max_wz, self._max_wz)
            self._publish_cmd(self._forward_speed, wz)

    def _finish(self, reason: str) -> None:
        if self._done:
            return
        self._done = True
        self.stop()
        self.get_logger().info(
            f"🏁{reason} z_rise={self._z - self._z_ground:+.3f}m "
            f"pitchΔ={math.degrees(self._pitch - self._pitch_ground):+.1f}° "
            f"yawErr={math.degrees(normalize_angle(self._target_yaw - self._yaw)):+.1f}°"
        )

    def _status_timer(self) -> None:
        if self._done:
            return
        if not self._baseline_ok:
            self.get_logger().info(f"⏳基线 {len(self._z_buf)}/{self._baseline_frames}")
            return

        state_icon = {
            self.GROUND: "🚩GROUND",
            self.CLIMBING: "⬆️CLIMB",
            self.SUMMIT: "🏁SUMMIT",
            self.CREEPING: "🚶CREEP",
        }
        s = state_icon.get(self._state, f"?({self._state})")
        yaw_error = normalize_angle(self._target_yaw - self._yaw)
        elapsed = time.monotonic() - self._start_time

        if self._state == self.CREEPING:
            dx = self._x - self._creep_start_x
            dy = self._y - self._creep_start_y
            dist = math.sqrt(dx * dx + dy * dy)
            self.get_logger().info(
                f"{s} dist={dist:.3f}/{CREEP_DIST_M:.1f}m "
                f"yawErr={math.degrees(yaw_error):+.1f}° "
                f"t={elapsed:.0f}s"
            )
        else:
            self.get_logger().info(
                f"{s} z_rise={self._z - self._z_ground:+.3f}m "
                f"pitchΔ={math.degrees(self._pitch - self._pitch_ground):+.1f}° "
                f"yawErr={math.degrees(yaw_error):+.1f}° "
                f"t={elapsed:.0f}s"
            )


def main(args=None):
    rclpy.init(args=args)
    node = RampDriveYawHold()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("keyboard interrupt")
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
