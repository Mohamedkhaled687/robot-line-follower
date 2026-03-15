#!/usr/bin/env python3
"""Client 0 — Simulator (Plant + Environment) for Line-Following Robot.

Simulates differential-drive robot kinematics on a 2D plane.
Publishes robot pose and nearest path reference point over CAN.
Receives velocity commands (v, omega) from the Controller client.
"""
from __future__ import print_function
import struct
import sys
import os
import math
import argparse

import numpy as np

current_dir = os.getcwd()
sys.path.append(current_dir)

PythonGateways = 'pythonGateways/'
sys.path.append(PythonGateways)

import VsiCommonPythonApi as vsiCommonPythonApi
import VsiCanPythonGateway as vsiCanPythonGateway


# =============================================
#  Path Generators
# =============================================
class StraightLinePath:
    """Dense polyline representing a straight segment."""

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
        return px, py, np.arctan2(dy, dx)


class CurvedPath:
    """Dense polyline built from a chain of cubic Bezier segments."""

    def __init__(self, control_points_list, num_per_seg=300):
        all_pts = []
        for P0, P1, P2, P3 in control_points_list:
            for t in np.linspace(0, 1, num_per_seg):
                pt = ((1 - t)**3 * np.array(P0)
                      + 3 * (1 - t)**2 * t * np.array(P1)
                      + 3 * (1 - t) * t**2 * np.array(P2)
                      + t**3 * np.array(P3))
                all_pts.append(pt)
        self.points = np.array(all_pts)

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
        return px, py, np.arctan2(dy, dx)


# =============================================
#  Robot Kinematic Model
# =============================================
class DiffDriveRobot:
    """Differential-drive robot with unicycle kinematics.

    State update (Euler integration):
        x     += v * cos(theta) * dt
        y     += v * sin(theta) * dt
        theta += omega * dt
    """

    def __init__(self, x, y, theta, axle_width=0.2):
        self.x = x
        self.y = y
        self.theta = theta
        self.L = axle_width

    def update(self, v, omega, dt, noise_sigma=None):
        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        self.theta += omega * dt
        self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi
        if noise_sigma:
            self.x += np.random.normal(0, noise_sigma.get('x', 0))
            self.y += np.random.normal(0, noise_sigma.get('y', 0))
            self.theta += np.random.normal(0, noise_sigma.get('theta', 0))


# =============================================
#  Signal Container
# =============================================
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


# =============================================
#  Simulator Client
# =============================================
class Simulator:
    def __init__(self, args):
        self.componentId = 0
        self.localHost = args.server_url
        self.domain = args.domain
        self.portNum = 50101
        self.simulationStep = 0
        self.stopRequested = False
        self.totalSimulationTime = 0
        self.mySignals = MySignals()

        self.path_type = args.path_type
        self.spawn_offset = args.spawn_offset
        self.noise_x = args.noise_x
        self.noise_y = args.noise_y
        self.noise_theta = args.noise_theta
        self.dt_phys = 0.001

        if self.path_type == 'curved':
            self.path = CurvedPath([
                [(0, 0), (2, 2), (4, 2), (6, 0)],
                [(6, 0), (8, -2), (10, -2), (12, 0)],
            ])
        else:
            self.path = StraightLinePath(start=(0, 0), end=(15, 0))

        self.robot = DiffDriveRobot(
            x=0.0,
            y=self.spawn_offset,
            theta=np.random.uniform(-0.3, 0.3)
        )

        self.noise_sigma = None
        if self.noise_x > 0 or self.noise_y > 0 or self.noise_theta > 0:
            self.noise_sigma = {
                'x': self.noise_x,
                'y': self.noise_y,
                'theta': self.noise_theta,
            }

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

                # --- Receive velocity commands from Controller ---
                recvData = vsiCanPythonGateway.recvVariableFromCanPacket(8, 0, 64, 10)
                self.mySignals.v_cmd, recvData = self.unpackBytes('d', recvData, self.mySignals.v_cmd)

                recvData = vsiCanPythonGateway.recvVariableFromCanPacket(8, 0, 64, 11)
                self.mySignals.omega_cmd, recvData = self.unpackBytes('d', recvData, self.mySignals.omega_cmd)

                # --- Update robot kinematics ---
                self.robot.update(
                    self.mySignals.v_cmd,
                    self.mySignals.omega_cmd,
                    self.dt_phys,
                    self.noise_sigma
                )

                # --- Find nearest path point ---
                px, py, theta_path = self.path.nearest_point(self.robot.x, self.robot.y)

                # --- Update signals ---
                self.mySignals.x_robot = self.robot.x
                self.mySignals.y_robot = self.robot.y
                self.mySignals.theta_robot = self.robot.theta
                self.mySignals.x_path = px
                self.mySignals.y_path = py
                self.mySignals.theta_path = theta_path

                # --- Send robot state and path ref on CAN ---
                for can_id, value in [
                    (20, self.mySignals.x_robot),
                    (21, self.mySignals.y_robot),
                    (22, self.mySignals.theta_robot),
                    (23, self.mySignals.x_path),
                    (24, self.mySignals.y_path),
                    (25, self.mySignals.theta_path),
                ]:
                    packed = self.packBytes('d', value)
                    self.sendCanVariable(packed, 0, 64, can_id)

                print(f"\n+=simulator+=")
                print(f"  VSI time: {vsiCommonPythonApi.getSimulationTimeInNs()} ns")
                print(f"  Robot: ({self.robot.x:.4f}, {self.robot.y:.4f}, {self.robot.theta:.4f})")
                print(f"  Path:  ({px:.4f}, {py:.4f}, {theta_path:.4f})")
                print(f"  Cmd:   v={self.mySignals.v_cmd:.4f} w={self.mySignals.omega_cmd:.4f}\n")

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

    def sendCanVariable(self, packed_data, start_bit, bit_size, can_id):
        vsiCanPythonGateway.setCanId(can_id)
        vsiCanPythonGateway.setCanPayloadBits(packed_data, start_bit, bit_size)
        vsiCanPythonGateway.sendCanPacket()

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
    p = argparse.ArgumentParser("LineFollower Simulator")
    p.add_argument('--domain', default='AF_UNIX')
    p.add_argument('--server-url', default='localhost')
    p.add_argument('--path-type', default='straight', choices=['straight', 'curved'])
    p.add_argument('--spawn-offset', type=float, default=0.3)
    p.add_argument('--noise-x', type=float, default=0.0)
    p.add_argument('--noise-y', type=float, default=0.0)
    p.add_argument('--noise-theta', type=float, default=0.0)
    args = p.parse_args()
    sim = Simulator(args)
    sim.mainThread()


if __name__ == '__main__':
    main()
