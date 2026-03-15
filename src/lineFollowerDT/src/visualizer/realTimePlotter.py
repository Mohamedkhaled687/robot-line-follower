"""
realTimePlotter.py
------------------

Real-time dashboard for a line-following robot simulation,
implemented with PyQtGraph.  Shows:

  1) Trajectory (robot path vs reference path) — top-left
  2) Lateral error vs time                     — top-right
  3) Velocity commands vs time                 — bottom-left
  4) Robot heading animation                   — bottom-right

Public API:

    rtp = RealTimePlotter(buffer_size=2000, update_frequency=5)
    rtp.update_data(t, x_robot, y_robot, theta_robot,
                    x_path, y_path, theta_path,
                    v_cmd, omega_cmd, e_lat)
    rtp.close()
"""

from __future__ import annotations
import sys
from collections import deque
from typing import Deque

import numpy as np

try:
    from PyQt6 import QtWidgets, QtCore
except ModuleNotFoundError:
    from PySide6 import QtWidgets, QtCore          # type: ignore

import pyqtgraph as pg

pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')


class RealTimePlotter:

    def __init__(self, buffer_size: int = 2000, update_frequency: int = 5):
        self.buffer_size = buffer_size
        self.update_frequency = update_frequency
        self._update_counter = 0

        self.t_buf:          Deque[float] = deque(maxlen=buffer_size)
        self.x_robot_buf:    Deque[float] = deque(maxlen=buffer_size)
        self.y_robot_buf:    Deque[float] = deque(maxlen=buffer_size)
        self.theta_robot_buf:Deque[float] = deque(maxlen=buffer_size)
        self.x_path_buf:     Deque[float] = deque(maxlen=buffer_size)
        self.y_path_buf:     Deque[float] = deque(maxlen=buffer_size)
        self.e_lat_buf:      Deque[float] = deque(maxlen=buffer_size)
        self.v_cmd_buf:      Deque[float] = deque(maxlen=buffer_size)
        self.omega_cmd_buf:  Deque[float] = deque(maxlen=buffer_size)

        self._app = (QtWidgets.QApplication.instance()
                     or QtWidgets.QApplication(sys.argv))
        pg.setConfigOptions(antialias=True)

        self.win = pg.GraphicsLayoutWidget(
            title="Line-Following Robot — Real-Time Dashboard")
        self.win.resize(1300, 850)

        # ---- 1) Trajectory (top-left) ----
        self.traj_plot = self.win.addPlot(title="Trajectory (XY)")
        self.traj_plot.setLabel('bottom', 'X', units='m')
        self.traj_plot.setLabel('left', 'Y', units='m')
        self.traj_plot.addLegend()
        self.traj_plot.setAspectLocked(True)
        self.curve_robot = self.traj_plot.plot(
            pen=pg.mkPen('b', width=2), name="Robot")
        self.curve_path = self.traj_plot.plot(
            pen=pg.mkPen('g', width=2, style=QtCore.Qt.PenStyle.DashLine),
            name="Path")
        self.robot_dot = self.traj_plot.plot(
            pen=None, symbol='o', symbolBrush='r', symbolSize=8)

        # ---- 2) Lateral Error (top-right) ----
        self.win.nextColumn()
        self.err_plot = self.win.addPlot(title="Lateral Error vs Time")
        self.err_plot.setLabel('bottom', 'Time', units='s')
        self.err_plot.setLabel('left', 'e_lat', units='m')
        self.curve_elat = self.err_plot.plot(pen=pg.mkPen('r', width=2))
        self.err_plot.addLine(y=0, pen=pg.mkPen('k', style=QtCore.Qt.PenStyle.DashLine))

        # ---- 3) Velocity Commands (bottom-left) ----
        self.win.nextRow()
        self.cmd_plot = self.win.addPlot(title="Commands vs Time")
        self.cmd_plot.setLabel('bottom', 'Time', units='s')
        self.cmd_plot.setLabel('left', 'Value')
        self.cmd_plot.addLegend()
        self.curve_v = self.cmd_plot.plot(
            pen=pg.mkPen('m', width=2), name="v_cmd (m/s)")
        self.curve_omega = self.cmd_plot.plot(
            pen=pg.mkPen('c', width=2), name="omega_cmd (rad/s)")

        # ---- 4) Robot Heading Animation (bottom-right) ----
        self.win.nextColumn()
        self.anim_plot = self.win.addPlot(title="Robot (top view)")
        self.anim_plot.setLabel('bottom', 'X', units='m')
        self.anim_plot.setLabel('left', 'Y', units='m')
        self.anim_plot.setAspectLocked(True)
        self.anim_body = self.anim_plot.plot(
            pen=None, symbol='o', symbolBrush='b', symbolSize=14)
        self.anim_heading = self.anim_plot.plot(
            pen=pg.mkPen('r', width=3))
        self.anim_path_dot = self.anim_plot.plot(
            pen=None, symbol='x', symbolBrush='g', symbolSize=10)

        self.win.show()
        self._app.processEvents()

    def update_data(self, t, x_robot, y_robot, theta_robot,
                    x_path, y_path, theta_path,
                    v_cmd, omega_cmd, e_lat):
        self.t_buf.append(t)
        self.x_robot_buf.append(x_robot)
        self.y_robot_buf.append(y_robot)
        self.theta_robot_buf.append(theta_robot)
        self.x_path_buf.append(x_path)
        self.y_path_buf.append(y_path)
        self.e_lat_buf.append(e_lat)
        self.v_cmd_buf.append(v_cmd)
        self.omega_cmd_buf.append(omega_cmd)

        self._update_counter += 1
        if self._update_counter >= self.update_frequency:
            self._update_counter = 0
            self._redraw()

    def close(self):
        self.win.close()

    def _to_np(self, dq):
        return np.fromiter(dq, dtype=float, count=len(dq))

    def _redraw(self):
        t   = self._to_np(self.t_buf)
        xr  = self._to_np(self.x_robot_buf)
        yr  = self._to_np(self.y_robot_buf)
        xp  = self._to_np(self.x_path_buf)
        yp  = self._to_np(self.y_path_buf)
        el  = self._to_np(self.e_lat_buf)
        vc  = self._to_np(self.v_cmd_buf)
        wc  = self._to_np(self.omega_cmd_buf)

        # 1) Trajectory
        self.curve_robot.setData(xr, yr)
        self.curve_path.setData(xp, yp)
        if xr.size:
            self.robot_dot.setData([xr[-1]], [yr[-1]])

        # 2) Lateral error
        self.curve_elat.setData(t, el)

        # 3) Commands
        self.curve_v.setData(t, vc)
        self.curve_omega.setData(t, wc)

        # 4) Robot heading animation (zoomed view around current position)
        if xr.size:
            x, y = xr[-1], yr[-1]
            theta = self.theta_robot_buf[-1]
            L = 0.3
            self.anim_body.setData([x], [y])
            self.anim_heading.setData(
                [x, x + L * np.cos(theta)],
                [y, y + L * np.sin(theta)])
            if xp.size:
                self.anim_path_dot.setData([xp[-1]], [yp[-1]])
            self.anim_plot.setXRange(x - 1.0, x + 1.0, padding=0)
            self.anim_plot.setYRange(y - 1.0, y + 1.0, padding=0)

        self._app.processEvents()
