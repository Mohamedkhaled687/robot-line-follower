#!/usr/bin/env python3
"""Offline validation of differential-drive kinematics and PID controller.

Runs the same models used by the VSI clients in a standalone loop
(no VSI dependency) and produces a validation plot.
"""
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


class DiffDriveRobot:
    def __init__(self, x, y, theta, axle_width=0.2):
        self.x = x
        self.y = y
        self.theta = theta
        self.L = axle_width

    def update(self, v, omega, dt):
        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        self.theta += omega * dt
        self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi


class PIDController:
    def __init__(self, kp, ki, kd, dt=0.001):
        self.kp, self.ki, self.kd, self.dt = kp, ki, kd, dt
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error):
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


class StraightLinePath:
    def __init__(self, start, end, num_points=2000):
        t = np.linspace(0, 1, num_points)
        self.points = np.column_stack([
            start[0] + t * (end[0] - start[0]),
            start[1] + t * (end[1] - start[1])
        ])

    def nearest_point(self, x, y):
        dists = np.hypot(self.points[:, 0] - x, self.points[:, 1] - y)
        idx = np.argmin(dists)
        px, py = self.points[idx]
        if idx < len(self.points) - 1:
            dx = self.points[idx + 1, 0] - px
            dy = self.points[idx + 1, 1] - py
        else:
            dx = px - self.points[idx - 1, 0]
            dy = py - self.points[idx - 1, 1]
        theta_path = np.arctan2(dy, dx)
        return px, py, theta_path


dt = 0.001
v_const = 0.5
robot = DiffDriveRobot(x=0.0, y=0.3, theta=0.0)
pid = PIDController(kp=2.0, ki=0.3, kd=1.0, dt=dt)
path = StraightLinePath(start=(0, 0), end=(10, 0))

xs, ys, es, ts = [], [], [], []
for step in range(15000):
    t = step * dt
    px, py, theta_path = path.nearest_point(robot.x, robot.y)
    e_lat = -np.sin(theta_path) * (robot.x - px) + np.cos(theta_path) * (robot.y - py)
    omega = -pid.compute(e_lat)
    robot.update(v_const, omega, dt)
    xs.append(robot.x)
    ys.append(robot.y)
    es.append(e_lat)
    ts.append(t)

fig, axes = plt.subplots(2, 1, figsize=(10, 8))
axes[0].plot(xs, ys, 'b-', label='Robot trajectory')
axes[0].plot(path.points[:, 0], path.points[:, 1], 'g--', label='Path')
axes[0].set_xlabel('X (m)')
axes[0].set_ylabel('Y (m)')
axes[0].legend()
axes[0].set_title('Trajectory')
axes[0].set_aspect('equal')

axes[1].plot(ts, es, 'r-')
axes[1].set_xlabel('Time (s)')
axes[1].set_ylabel('Lateral Error (m)')
axes[1].set_title('Lateral Error vs Time')
axes[1].axhline(y=0, color='k', linestyle='--', alpha=0.3)

plt.tight_layout()
outdir = 'results/figures'
os.makedirs(outdir, exist_ok=True)
outpath = os.path.join(outdir, 'kinematics_validation.png')
plt.savefig(outpath, dpi=150)
print(f"[OK] Saved plot to {outpath}")
print(f"Final lateral error: {es[-1]:.6f} m")
print(f"Max lateral error:   {max(abs(e) for e in es):.6f} m")

