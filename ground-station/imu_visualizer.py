#!/usr/bin/env python3
"""
Realtime BMI088 IMU board orientation visualizer.

Reads the PIC24 debug UART stream (`task_bmi088`) and renders a 3D PCB
that follows the board's orientation live.

Firmware emits lines like:
    [bmi088] A=    12,   -34,  5432  G=    -2,     1,     0

Accel range = +/-6 g, gyro range = +/-2000 dps. Yaw is unobservable
(no magnetometer) and will drift slowly — expected.

Usage:
    python imu_visualizer.py
    python imu_visualizer.py --port /dev/tty.usbserial-BG03U9VQ --log imu.csv

Requires: pyserial, matplotlib, numpy
"""

import argparse
import csv
import math
import queue
import re
import sys
import threading
import time

import numpy as np
import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

DEFAULT_PORT = "/dev/tty.usbserial-BG03U9VQ"
DEFAULT_BAUD = 9600

ACCEL_LSB_TO_G = 6.0 / 32768.0
GYRO_LSB_TO_DPS = 2000.0 / 32768.0

LINE_RE = re.compile(
    r"\[bmi088\]\s*A=\s*(-?\d+),\s*(-?\d+),\s*(-?\d+)\s+"
    r"G=\s*(-?\d+),\s*(-?\d+),\s*(-?\d+)"
)


# ---------- quaternion math ----------

def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_norm(q):
    n = np.linalg.norm(q)
    return q / n if n > 0 else np.array([1.0, 0.0, 0.0, 0.0])


def quat_from_axis_angle(axis, angle):
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-12 or angle == 0:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = axis / n
    s = math.sin(angle / 2.0)
    return np.array([math.cos(angle / 2.0), axis[0] * s, axis[1] * s, axis[2] * s])


def quat_to_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def quat_to_euler(q):
    """Return roll (X), pitch (Y), yaw (Z) in degrees, ZYX convention."""
    w, x, y, z = q
    sinr = 2 * (w * x + y * z)
    cosr = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    siny = 2 * (w * z + x * y)
    cosy = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def slerp(q1, q2, t):
    q1 = quat_norm(q1)
    q2 = quat_norm(q2)
    dot = float(np.dot(q1, q2))
    if dot < 0:
        q2 = -q2
        dot = -dot
    if dot > 0.9995:
        return quat_norm(q1 + t * (q2 - q1))
    theta_0 = math.acos(dot)
    theta = theta_0 * t
    sin_theta = math.sin(theta)
    sin_theta_0 = math.sin(theta_0)
    s1 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s2 = sin_theta / sin_theta_0
    return quat_norm(s1 * q1 + s2 * q2)


# ---------- serial reader ----------

def reader_thread(ser, q_out, stop_evt):
    while not stop_evt.is_set():
        try:
            raw = ser.readline()
        except serial.SerialException as e:
            print(f"[serial] {e}", file=sys.stderr)
            break
        if not raw:
            continue
        try:
            line = raw.decode("utf-8", errors="replace").strip()
        except Exception:
            continue
        m = LINE_RE.search(line)
        if m:
            ax, ay, az, gx, gy, gz = (int(v) for v in m.groups())
            q_out.put((time.time(), ax, ay, az, gx, gy, gz))
        elif line:
            print(line)


# ---------- main ----------

def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    p.add_argument("--alpha", type=float, default=0.02,
                   help="complementary filter blend toward accel reference (0..1)")
    p.add_argument("--log", default=None, help="optional CSV log path")
    p.add_argument("--bias-samples", type=int, default=20,
                   help="number of initial samples averaged for gyro bias")
    args = p.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except serial.SerialException as e:
        print(f"Failed to open {args.port}: {e}", file=sys.stderr)
        sys.exit(1)
    print(f"[imu] reading from {args.port} @ {args.baud}")

    log_writer = None
    log_fp = None
    if args.log:
        log_fp = open(args.log, "a", newline="")
        log_writer = csv.writer(log_fp)
        log_writer.writerow(["t", "ax_g", "ay_g", "az_g",
                             "gx_dps", "gy_dps", "gz_dps",
                             "roll_deg", "pitch_deg", "yaw_deg"])

    q_in = queue.Queue(maxsize=2000)
    stop_evt = threading.Event()
    t = threading.Thread(target=reader_thread, args=(ser, q_in, stop_evt),
                         daemon=True)
    t.start()

    # state
    state = {
        "q": np.array([1.0, 0.0, 0.0, 0.0]),
        "gyro_bias": np.array([0.0, 0.0, 0.0]),
        "bias_buf": [],
        "biased": False,
        "last_t": None,
        "accel_g": np.array([0.0, 0.0, 1.0]),
        "gyro_dps": np.array([0.0, 0.0, 0.0]),
    }

    # ---------- 3D scene ----------
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(-1, 1)
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlabel("X (world)")
    ax.set_ylabel("Y (world)")
    ax.set_zlabel("Z (world)")

    # PCB box, body frame (units: arbitrary, ~PCB shape)
    sx, sy, sz = 0.6, 0.6, 0.04
    verts_body = np.array([
        [-sx, -sy, -sz], [sx, -sy, -sz], [sx, sy, -sz], [-sx, sy, -sz],
        [-sx, -sy,  sz], [sx, -sy,  sz], [sx, sy,  sz], [-sx, sy,  sz],
    ])
    faces_idx = [
        [0, 1, 2, 3],  # bottom
        [4, 5, 6, 7],  # top
        [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7],
    ]
    face_colors = ["#225588", "#66aaff", "#888888", "#888888", "#888888", "#888888"]
    poly = Poly3DCollection([verts_body[f] for f in faces_idx],
                            facecolors=face_colors,
                            edgecolors="black", linewidths=0.5, alpha=0.9)
    ax.add_collection3d(poly)

    # body axes arrows
    arrow_len = 0.9
    body_x_line, = ax.plot([0, arrow_len], [0, 0], [0, 0], color="red", lw=2)
    body_y_line, = ax.plot([0, 0], [0, arrow_len], [0, 0], color="green", lw=2)
    body_z_line, = ax.plot([0, 0], [0, 0], [0, arrow_len], color="blue", lw=2)

    # static world axes (thin/dashed)
    ax.plot([0, 1.0], [0, 0], [0, 0], color="red", lw=0.8, ls=":")
    ax.plot([0, 0], [0, 1.0], [0, 0], color="green", lw=0.8, ls=":")
    ax.plot([0, 0], [0, 0], [0, 1.0], color="blue", lw=0.8, ls=":")

    overlay = ax.text2D(0.02, 0.95, "", transform=ax.transAxes,
                        family="monospace", fontsize=9,
                        verticalalignment="top")

    def drain_queue():
        new = 0
        while True:
            try:
                sample = q_in.get_nowait()
            except queue.Empty:
                break
            ts, ax_r, ay_r, az_r, gx_r, gy_r, gz_r = sample
            a = np.array([ax_r, ay_r, az_r], dtype=float) * ACCEL_LSB_TO_G
            g = np.array([gx_r, gy_r, gz_r], dtype=float) * GYRO_LSB_TO_DPS

            if not state["biased"]:
                state["bias_buf"].append(g)
                if len(state["bias_buf"]) >= args.bias_samples:
                    state["gyro_bias"] = np.mean(state["bias_buf"], axis=0)
                    state["biased"] = True
                    print(f"[imu] gyro bias = {state['gyro_bias']} dps")
                state["last_t"] = ts
                state["accel_g"] = a
                continue

            g_corr = g - state["gyro_bias"]
            dt = ts - state["last_t"] if state["last_t"] else 0.0
            state["last_t"] = ts
            if dt > 0 and dt < 1.0:
                omega_rad = np.deg2rad(g_corr) * dt
                angle = float(np.linalg.norm(omega_rad))
                if angle > 1e-9:
                    dq = quat_from_axis_angle(omega_rad, angle)
                    state["q"] = quat_norm(quat_mul(state["q"], dq))

            # accel-based reference quaternion: rotation that takes body +Z
            # to align with measured gravity (in body frame, gravity ~ accel
            # vector when static). We want world Z = a_norm in body frame.
            mag = float(np.linalg.norm(a))
            if 0.5 < mag < 1.5:
                a_norm = a / mag
                # Build a rotation that rotates body axes such that the
                # accelerometer-measured gravity (a_norm in body frame)
                # corresponds to world +Z.
                # Equivalent: roll/pitch from accel, yaw kept from current q.
                roll = math.atan2(a_norm[1], a_norm[2])
                pitch = math.atan2(-a_norm[0],
                                   math.sqrt(a_norm[1] ** 2 + a_norm[2] ** 2))
                _, _, yaw = quat_to_euler(state["q"])
                yaw_rad = math.radians(yaw)
                # ZYX (yaw, pitch, roll) -> quaternion
                cy, sy_ = math.cos(yaw_rad / 2), math.sin(yaw_rad / 2)
                cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
                cr, sr = math.cos(roll / 2), math.sin(roll / 2)
                q_ref = np.array([
                    cr * cp * cy + sr * sp * sy_,
                    sr * cp * cy - cr * sp * sy_,
                    cr * sp * cy + sr * cp * sy_,
                    cr * cp * sy_ - sr * sp * cy,
                ])
                state["q"] = slerp(state["q"], q_ref, args.alpha)

            state["accel_g"] = a
            state["gyro_dps"] = g_corr

            if log_writer is not None:
                roll_d, pitch_d, yaw_d = quat_to_euler(state["q"])
                log_writer.writerow([f"{ts:.3f}",
                                     f"{a[0]:.4f}", f"{a[1]:.4f}", f"{a[2]:.4f}",
                                     f"{g_corr[0]:.3f}", f"{g_corr[1]:.3f}",
                                     f"{g_corr[2]:.3f}",
                                     f"{roll_d:.2f}", f"{pitch_d:.2f}",
                                     f"{yaw_d:.2f}"])
            new += 1
        return new

    def update(_frame):
        drain_queue()
        R = quat_to_matrix(state["q"])
        verts_world = verts_body @ R.T
        poly.set_verts([verts_world[f] for f in faces_idx])

        bx = R @ np.array([arrow_len, 0, 0])
        by = R @ np.array([0, arrow_len, 0])
        bz = R @ np.array([0, 0, arrow_len])
        body_x_line.set_data_3d([0, bx[0]], [0, bx[1]], [0, bx[2]])
        body_y_line.set_data_3d([0, by[0]], [0, by[1]], [0, by[2]])
        body_z_line.set_data_3d([0, bz[0]], [0, bz[1]], [0, bz[2]])

        roll_d, pitch_d, yaw_d = quat_to_euler(state["q"])
        a = state["accel_g"]
        g = state["gyro_dps"]
        overlay.set_text(
            f"roll  {roll_d:7.2f} deg\n"
            f"pitch {pitch_d:7.2f} deg\n"
            f"yaw   {yaw_d:7.2f} deg  (drifts, no mag)\n"
            f"|a|   {np.linalg.norm(a):5.3f} g\n"
            f"a  = [{a[0]:+5.2f} {a[1]:+5.2f} {a[2]:+5.2f}] g\n"
            f"w  = [{g[0]:+6.1f} {g[1]:+6.1f} {g[2]:+6.1f}] dps"
        )
        return poly, body_x_line, body_y_line, body_z_line, overlay

    anim = FuncAnimation(fig, update, interval=33, blit=False,
                         cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        stop_evt.set()
        try:
            ser.close()
        except Exception:
            pass
        if log_fp:
            log_fp.close()


if __name__ == "__main__":
    main()
