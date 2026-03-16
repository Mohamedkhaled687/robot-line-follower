#!/usr/bin/env python3
"""Client 1 — PID Controller for Line-Following Robot.

Uses a PID control law that minimizes both lateral and heading error:
    omega = -(kp * e_lat  +  ki * integral(e_lat)  +  kd * e_heading)

The heading error serves as the derivative term (derivative on measurement),
since e_heading ~ de_lat/dt through the kinematic chain.  This avoids the
finite-difference derivative spike that causes overshoot.
"""
from __future__ import print_function
import struct
import sys
import os
import math
import argparse

current_dir = os.getcwd()
sys.path.append(current_dir)

PythonGateways = 'pythonGateways/'
sys.path.append(PythonGateways)

import VsiCommonPythonApi as vsiCommonPythonApi
import VsiCanPythonGateway as vsiCanPythonGateway


class PIDController:
    """Discrete PID with anti-windup and integral saturation.

    Supports an optional `derivative_override` so the caller can supply
    the heading error directly instead of relying on finite differences.
    """

    def __init__(self, kp, ki, kd, dt=0.001,
                 max_output=5.0, max_integral=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.max_output = max_output
        self.max_integral = max_integral
        self.integral = 0.0
        self.prev_error = None

    def compute(self, error, derivative_override=None):
        self.integral += error * self.dt
        self.integral = max(-self.max_integral,
                            min(self.max_integral, self.integral))

        if derivative_override is not None:
            derivative = derivative_override
        elif self.prev_error is None:
            derivative = 0.0
        else:
            derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        output = (self.kp * error
                  + self.ki * self.integral
                  + self.kd * derivative)

        if abs(output) > self.max_output:
            self.integral -= error * self.dt
            output = self.max_output if output > 0 else -self.max_output
        return output

    def reset(self):
        self.integral = 0.0
        self.prev_error = None


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


class Controller:
    def __init__(self, args):
        self.componentId = 1
        self.localHost = args.server_url
        self.domain = args.domain
        self.portNum = 50102
        self.simulationStep = 0
        self.stopRequested = False
        self.totalSimulationTime = 0
        self.mySignals = MySignals()

        self.v_const = args.v_const
        self.pid = PIDController(
            kp=args.kp, ki=args.ki, kd=args.kd, dt=0.001
        )

        self.converge_threshold = 0.03
        self.converge_hold_steps = 500
        self._converge_counter = 0
        self._converged = False

    @staticmethod
    def _normalize_angle(a):
        return (a + math.pi) % (2 * math.pi) - math.pi

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
                    (20, 'x_robot'), (21, 'y_robot'), (22, 'theta_robot'),
                    (23, 'x_path'),  (24, 'y_path'),  (25, 'theta_path'),
                ]:
                    recvData = vsiCanPythonGateway.recvVariableFromCanPacket(8, 0, 64, can_id)
                    val, recvData = self.unpackBytes('d', recvData)
                    setattr(self.mySignals, attr, val)

                s = self.mySignals
                e_lat = (-math.sin(s.theta_path) * (s.x_robot - s.x_path)
                         + math.cos(s.theta_path) * (s.y_robot - s.y_path))
                e_heading = self._normalize_angle(s.theta_robot - s.theta_path)

                if self._converged:
                    v_cmd = 0.0
                    omega_cmd = 0.0
                else:
                    omega_cmd = -self.pid.compute(e_lat, derivative_override=e_heading)
                    v_cmd = self.v_const

                    if abs(e_lat) < self.converge_threshold and abs(e_heading) < 0.05:
                        self._converge_counter += 1
                    else:
                        self._converge_counter = 0

                    if self._converge_counter >= self.converge_hold_steps:
                        self._converged = True
                        self.pid.reset()
                        print("\n*** CONVERGED — robot reached the path, stopping. ***\n")

                self.mySignals.v_cmd = v_cmd
                self.mySignals.omega_cmd = omega_cmd

                packed_v = self.packBytes('d', v_cmd)
                self.sendCanVariable(packed_v, 0, 64, 10)

                packed_w = self.packBytes('d', omega_cmd)
                self.sendCanVariable(packed_w, 0, 64, 11)

                print(f"\n+=controller+=")
                print(f"  VSI time: {vsiCommonPythonApi.getSimulationTimeInNs()} ns")
                print(f"  e_lat={e_lat:.6f}  e_head={e_heading:.4f}  v={v_cmd:.3f}  w={omega_cmd:.4f}\n")

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
    p = argparse.ArgumentParser("LineFollower Controller")
    p.add_argument('--domain', default='AF_UNIX')
    p.add_argument('--server-url', default='localhost')
    p.add_argument('--kp', type=float, default=2.0)
    p.add_argument('--ki', type=float, default=0.1)
    p.add_argument('--kd', type=float, default=3.0)
    p.add_argument('--v-const', type=float, default=0.5)
    args = p.parse_args()
    ctrl = Controller(args)
    ctrl.mainThread()


if __name__ == '__main__':
    main()
