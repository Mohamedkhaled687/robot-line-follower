#!/usr/bin/env python3
"""Client 2 — Visualizer/Logger for Line-Following Robot.

Observes all CAN signals, logs time-series data, computes KPIs
(overshoot, settling time, steady-state error), saves CSV + PNG
results, and shows both a pygame 2D top-down view and a PyQtGraph
real-time dashboard during the simulation run.
"""
from __future__ import print_function
import struct
import sys
import os
import csv
import math
import argparse
import numpy as np

current_dir = os.getcwd()
sys.path.append(current_dir)

PythonGateways = 'pythonGateways/'
sys.path.append(PythonGateways)

import VsiCommonPythonApi as vsiCommonPythonApi
import VsiCanPythonGateway as vsiCanPythonGateway

src_vis_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, src_vis_dir)

from pygameVisualizer import PygameVisualizer

try:
    from realTimePlotter import RealTimePlotter
    _HAS_PYQTGRAPH = True
except Exception:
    _HAS_PYQTGRAPH = False


class KPILogger:
    """Accumulates time-series data and computes control-performance KPIs."""

    def __init__(self):
        self.times = []
        self.lateral_errors = []
        self.x_robots = []
        self.y_robots = []
        self.x_paths = []
        self.y_paths = []
        self.v_cmds = []
        self.omega_cmds = []

    def log(self, t, e_lat, x_r, y_r, x_p, y_p, v, omega):
        self.times.append(t)
        self.lateral_errors.append(e_lat)
        self.x_robots.append(x_r)
        self.y_robots.append(y_r)
        self.x_paths.append(x_p)
        self.y_paths.append(y_p)
        self.v_cmds.append(v)
        self.omega_cmds.append(omega)

    def compute_kpis(self):
        errors = np.array(self.lateral_errors)
        times = np.array(self.times)
        abs_err = np.abs(errors)

        overshoot = float(np.max(abs_err))

        threshold = 0.02
        settling_time = times[-1]
        for i in range(len(abs_err) - 1, -1, -1):
            if abs_err[i] >= threshold:
                if i < len(times) - 1:
                    settling_time = float(times[i + 1])
                break
        else:
            settling_time = float(times[0])

        n20 = max(1, len(errors) // 5)
        ss_error = float(np.mean(abs_err[-n20:]))

        return {
            'overshoot_m': overshoot,
            'settling_time_s': settling_time,
            'steady_state_error_m': ss_error,
        }

    def save_csv(self, filepath):
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        with open(filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time_s', 'e_lat', 'x_robot', 'y_robot',
                             'x_path', 'y_path', 'v_cmd', 'omega_cmd'])
            for i in range(len(self.times)):
                writer.writerow([
                    self.times[i], self.lateral_errors[i],
                    self.x_robots[i], self.y_robots[i],
                    self.x_paths[i], self.y_paths[i],
                    self.v_cmds[i], self.omega_cmds[i],
                ])
        print(f"[OK] Data saved to {filepath}")

    def save_plots(self, filepath_prefix):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        os.makedirs(os.path.dirname(filepath_prefix), exist_ok=True)

        fig, ax = plt.subplots(figsize=(10, 6))
        ax.plot(self.x_robots, self.y_robots, 'b-', linewidth=1.5, label='Robot')
        ax.plot(self.x_paths, self.y_paths, 'g--', linewidth=1.5, label='Path')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Trajectory vs Path')
        ax.legend()
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        fig.savefig(f"{filepath_prefix}_trajectory.png", dpi=150, bbox_inches='tight')
        plt.close(fig)

        fig, ax = plt.subplots(figsize=(10, 4))
        ax.plot(self.times, self.lateral_errors, 'r-', linewidth=1)
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Lateral Error (m)')
        ax.set_title('Lateral Error vs Time')
        ax.grid(True, alpha=0.3)
        fig.savefig(f"{filepath_prefix}_error.png", dpi=150, bbox_inches='tight')
        plt.close(fig)

        fig, ax = plt.subplots(figsize=(10, 4))
        ax.plot(self.times, self.v_cmds, 'm-', linewidth=1, label='v_cmd')
        ax.plot(self.times, self.omega_cmds, 'c-', linewidth=1, label='omega_cmd')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Command Value')
        ax.set_title('Control Commands vs Time')
        ax.legend()
        ax.grid(True, alpha=0.3)
        fig.savefig(f"{filepath_prefix}_commands.png", dpi=150, bbox_inches='tight')
        plt.close(fig)

        print(f"[OK] Plots saved to {filepath_prefix}_*.png")


class MySignals:
    def __init__(self):
        self.v_cmd = 0.0
        self.omega_cmd = 0.0
        self.x_robot = 0.0
        self.y_robot = 0.0
        self.theta_robot = 0.0
        self.x_path = 0.0
        self.y_path = 0.0
        self.theta_path = 0.0


class Visualizer:
    def __init__(self, args):
        self.componentId = 2
        self.localHost = args.server_url
        self.domain = args.domain
        self.portNum = 50103
        self.simulationStep = 0
        self.stopRequested = False
        self.totalSimulationTime = 0
        self.mySignals = MySignals()
        self.kpi_logger = KPILogger()
        self.output_tag = args.output_tag

        self.pygame_vis = PygameVisualizer(
            width=800, height=600, update_frequency=10)

        self.qt_plotter = None
        if _HAS_PYQTGRAPH:
            try:
                self.qt_plotter = RealTimePlotter(
                    buffer_size=2000, update_frequency=5)
            except Exception:
                pass

    def mainThread(self):
        dSession = vsiCommonPythonApi.connectToServer(
            self.localHost, self.domain, self.portNum, self.componentId)
        vsiCanPythonGateway.initialize(dSession, self.componentId)
        try:
            vsiCommonPythonApi.waitForReset()
            self.updateInternalVariables()
            if vsiCommonPythonApi.isStopRequested():
                raise Exception("stopRequested")

            nextExpectedTime = vsiCommonPythonApi.getSimulationTimeInNs()

            while vsiCommonPythonApi.getSimulationTimeInNs() < self.totalSimulationTime:
                self.updateInternalVariables()
                if vsiCommonPythonApi.isStopRequested():
                    raise Exception("stopRequested")

                for can_id, attr in [
                    (10, 'v_cmd'), (11, 'omega_cmd'),
                    (20, 'x_robot'), (21, 'y_robot'), (22, 'theta_robot'),
                    (23, 'x_path'),  (24, 'y_path'),  (25, 'theta_path'),
                ]:
                    recvData = vsiCanPythonGateway.recvVariableFromCanPacket(8, 0, 64, can_id)
                    val, recvData = self.unpackBytes('d', recvData)
                    setattr(self.mySignals, attr, val)

                s = self.mySignals
                t_sec = vsiCommonPythonApi.getSimulationTimeInNs() * 1e-9
                e_lat = (-math.sin(s.theta_path) * (s.x_robot - s.x_path)
                         + math.cos(s.theta_path) * (s.y_robot - s.y_path))

                self.kpi_logger.log(t_sec, e_lat, s.x_robot, s.y_robot,
                                    s.x_path, s.y_path, s.v_cmd, s.omega_cmd)

                self.pygame_vis.update_data(
                    t_sec, s.x_robot, s.y_robot, s.theta_robot,
                    s.x_path, s.y_path, s.theta_path,
                    s.v_cmd, s.omega_cmd, e_lat)

                if self.qt_plotter is not None:
                    self.qt_plotter.update_data(
                        t_sec, s.x_robot, s.y_robot, s.theta_robot,
                        s.x_path, s.y_path, s.theta_path,
                        s.v_cmd, s.omega_cmd, e_lat)

                print(f"\n+=visualizer+=  t={t_sec:.3f}s  e_lat={e_lat:.6f}")

                self.updateInternalVariables()
                if vsiCommonPythonApi.isStopRequested():
                    raise Exception("stopRequested")

                nextExpectedTime += self.simulationStep
                if vsiCommonPythonApi.getSimulationTimeInNs() >= nextExpectedTime:
                    continue
                if nextExpectedTime > self.totalSimulationTime:
                    remaining = self.totalSimulationTime - vsiCommonPythonApi.getSimulationTimeInNs()
                    vsiCommonPythonApi.advanceSimulation(remaining)
                    break
                vsiCommonPythonApi.advanceSimulation(
                    nextExpectedTime - vsiCommonPythonApi.getSimulationTimeInNs())

        except Exception as e:
            if str(e) == "stopRequested":
                print("Terminate signal received")
                vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)
            else:
                print(f"Error: {e}")
        except:
            vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)

        self.pygame_vis.close()
        if self.qt_plotter is not None:
            self.qt_plotter.close()

        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.abspath(os.path.join(script_dir, '..', '..', '..', '..'))
        results_dir = os.path.join(project_root, 'results')
        figures_dir = os.path.join(results_dir, 'figures')

        self.kpi_logger.save_csv(os.path.join(results_dir, f"{self.output_tag}.csv"))
        self.kpi_logger.save_plots(os.path.join(figures_dir, self.output_tag))

        kpis = self.kpi_logger.compute_kpis()
        print("\n" + "=" * 50)
        print(f"  KPI RESULTS ({self.output_tag})")
        print(f"  Overshoot:          {kpis['overshoot_m']:.6f} m")
        print(f"  Settling Time:      {kpis['settling_time_s']:.3f} s")
        print(f"  Steady-State Error: {kpis['steady_state_error_m']:.6f} m")
        print("=" * 50 + "\n")

    def packBytes(self, signalType, signal):
        return struct.pack(f'={signalType}', signal)

    def unpackBytes(self, signalType, packedBytes, signal=""):
        fmt_map = {'d': 8, 'f': 4, 'i': 4, 'q': 8}
        nb = fmt_map.get(signalType, 8)
        val = struct.unpack(f'={signalType}', packedBytes[0:nb])[0]
        return val, packedBytes[nb:]

    def updateInternalVariables(self):
        self.totalSimulationTime = vsiCommonPythonApi.getTotalSimulationTime()
        self.stopRequested = vsiCommonPythonApi.isStopRequested()
        self.simulationStep = vsiCommonPythonApi.getSimulationStep()


def main():
    p = argparse.ArgumentParser("LineFollower Visualizer")
    p.add_argument('--domain', default='AF_UNIX')
    p.add_argument('--server-url', default='localhost')
    p.add_argument('--output-tag', default='run_default',
                   help='Tag for output CSV and PNG files')
    args = p.parse_args()
    vis = Visualizer(args)
    vis.mainThread()


if __name__ == '__main__':
    main()
