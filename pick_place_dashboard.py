#!/usr/bin/env python3
"""
pick_place_dashboard.py  --  Inverse kinematics + control dashboard
for the CoppeliaSim EDU 4.1.0 waste-sorting pick & place cell.

  scene_builder.lua     builds the plant, conveyor, arm, camera, proximity sensor
  robot_controller.lua  low-level robot / conveyor / gripper code inside the sim
  pick_place_dashboard.py   (this file)
        * ArmKinematics   closed-form IK / FK of the 5-DOF arm (tool pointing down)
        * CellController  state machine: wait item -> camera -> IK -> pick -> sort -> place
        * Dashboard       Tk GUI: start / pause / stop / E-stop, live monitoring
        * MockLink        built-in simulator so the whole thing runs without CoppeliaSim

Usage
    python pick_place_dashboard.py                 # real CoppeliaSim, 127.0.0.1:19997
    python pick_place_dashboard.py --port 19999
    python pick_place_dashboard.py --mock          # try the dashboard without CoppeliaSim
    python pick_place_dashboard.py --mock --no-gui --duration 90   # headless self-test

Requirements: Python 3.8+ with tkinter.  For a real simulation copy the legacy
remote-API client files next to this script (see README.md):
    sim.py, simConst.py, remoteApi.dll / remoteApi.so / remoteApi.dylib
"""
import argparse
import colorsys
import math
import random
import struct
import sys
import threading
import time
from collections import deque

CLASS_NAMES = {1: 'PLASTIC', 2: 'METAL', 3: 'ORGANIC'}
CLASS_COLORS = {1: '#2f5bff', 2: '#f2c200', 3: '#1fb34a'}
STATE_COLORS = {'IDLE': '#888888', 'WAIT_ITEM': '#3b82f6', 'DETECT': '#8b5cf6', 'APPROACH': '#f59e0b',
                'DESCEND': '#f59e0b', 'GRAB': '#ef4444', 'LIFT': '#f59e0b', 'TRANSIT': '#10b981',
                'RELEASE': '#ef4444', 'RETURN': '#10b981', 'PAUSED': '#eab308',
                'ESTOP': '#dc2626', 'STOPPED': '#888888', 'DISCONNECTED': '#555555'}


# =============================================================================
#  1.  KINEMATICS
# =============================================================================
class IKError(Exception):
    """Target not reachable / violates a joint limit."""


class ArmKinematics:
    """
    Arm geometry (all read from the simulation, see scene_cfg signal):

        J1  yaw   about +z at the base                       q1
        J2  pitch about +y (positive tilts the arm forward)  q2   shoulder, height shoulder_z
        J3  pitch                                             q3   elbow,   after link L1
        J4  pitch                                             q4   wrist,   after link L2
        J5  roll about the tool axis                          q5
        tool of length Lt hangs below the wrist (suction cup, TCP = Arm_Tip)

    With every pitch angle measured from "straight up", the planar chain is

        r = L1 sin q2 + L2 sin(q2+q3) + Lt sin(q2+q3+q4)
        z = L1 cos q2 + L2 cos(q2+q3) + Lt cos(q2+q3+q4)

    and for a vertical tool (suction cup facing down) q2+q3+q4 = pi, which makes
    the wrist centre simply  (r, z+Lt) and gives a closed-form solution:

        D  = (r^2 + zw^2 - L1^2 - L2^2) / (2 L1 L2)         zw = z + Lt - shoulder_z
        q3 = atan2(+/- sqrt(1-D^2), D)                       (+ = elbow up)
        q2 = atan2(r, zw) - atan2(L2 sin q3, L1 + L2 cos q3)
        q4 = pi - q2 - q3
        q1 = atan2(y - by, x - bx)
    """

    def __init__(self, base_x, base_y, shoulder_z, l1, l2, lt, limits):
        self.bx, self.by, self.sz = base_x, base_y, shoulder_z
        self.l1, self.l2, self.lt = l1, l2, lt
        self.limits = limits            # list of (lo, hi) for the 5 joints

    # ---------------------------------------------------------------- FK ----
    def fk(self, q):
        q1, q2, q3, q4 = q[0], q[1], q[2], q[3]
        a2, a3 = q2, q2 + q3
        phi = q2 + q3 + q4
        r = self.l1 * math.sin(a2) + self.l2 * math.sin(a3) + self.lt * math.sin(phi)
        z = self.sz + self.l1 * math.cos(a2) + self.l2 * math.cos(a3) + self.lt * math.cos(phi)
        return (self.bx + r * math.cos(q1), self.by + r * math.sin(q1), z)

    # ---------------------------------------------------------------- IK ----
    def ik(self, x, y, z, roll=0.0, elbow_up=True, q1_hint=None):
        dx, dy = x - self.bx, y - self.by
        r = math.hypot(dx, dy)
        q1 = math.atan2(dy, dx) if r > 1e-6 else (q1_hint if q1_hint is not None else 0.0)
        zw = z + self.lt - self.sz
        d = (r * r + zw * zw - self.l1 ** 2 - self.l2 ** 2) / (2.0 * self.l1 * self.l2)
        if abs(d) > 1.0:
            raise IKError('target (%.3f, %.3f, %.3f) out of reach (|D|=%.2f)' % (x, y, z, abs(d)))
        s = math.sqrt(max(0.0, 1.0 - d * d))
        q3 = math.atan2(s if elbow_up else -s, d)
        q2 = math.atan2(r, zw) - math.atan2(self.l2 * math.sin(q3), self.l1 + self.l2 * math.cos(q3))
        q4 = math.pi - q2 - q3
        q = [q1, q2, q3, q4, roll]
        for i, (v, (lo, hi)) in enumerate(zip(q, self.limits)):
            if v < lo - 1e-9 or v > hi + 1e-9:
                raise IKError('joint %d = %.1f deg outside [%.0f, %.0f] for target (%.3f, %.3f, %.3f)'
                              % (i + 1, math.degrees(v), math.degrees(lo), math.degrees(hi), x, y, z))
        return q

    # ------------------------------------------------------------- paths ----
    def cartesian_path(self, p0, p1, step=0.02, roll=0.0):
        """Joint waypoints from tip position p0 to p1.  The motion is interpolated in
        cylindrical coordinates about the base axis (angle, radius, height): vertical and
        radial moves are straight lines, a change of angle becomes an arc - which avoids the
        singularity a straight line through the base axis would have."""
        a0 = math.atan2(p0[1] - self.by, p0[0] - self.bx)
        a1 = math.atan2(p1[1] - self.by, p1[0] - self.bx)
        r0 = math.hypot(p0[0] - self.bx, p0[1] - self.by)
        r1 = math.hypot(p1[0] - self.bx, p1[1] - self.by)
        if r0 < 1e-3: a0 = a1
        if r1 < 1e-3: a1 = a0
        length = max(abs(a1 - a0) * 0.5 * (r0 + r1), abs(r1 - r0), abs(p1[2] - p0[2]))
        n = max(1, int(math.ceil(length / step)))
        out, hint = [], a0
        for i in range(1, n + 1):
            t = i / float(n)
            a = a0 + (a1 - a0) * t
            r = r0 + (r1 - r0) * t
            z = p0[2] + (p1[2] - p0[2]) * t
            q = self.ik(self.bx + r * math.cos(a), self.by + r * math.sin(a), z, roll, True, hint)
            q[0] = a if abs(a) <= math.pi + 1e-9 else q[0]
            hint = q[0]
            out.append(q)
        return out

    @staticmethod
    def joint_path(q0, q1, step=0.05):
        n = max(1, int(math.ceil(max(abs(b - a) for a, b in zip(q0, q1)) / step)))
        return [[a + (b - a) * i / float(n) for a, b in zip(q0, q1)] for i in range(1, n + 1)]


# =============================================================================
#  2.  LINKS  (real CoppeliaSim legacy remote API  /  built-in mock)
# =============================================================================
class BaseLink:
    name = 'base'
    def connect(self): raise NotImplementedError
    def close(self): pass
    def sim_start(self): pass
    def sim_stop(self): pass
    def sim_pause(self): pass
    def set_int(self, n, v): raise NotImplementedError
    def set_float(self, n, v): raise NotImplementedError
    def set_str(self, n, b): raise NotImplementedError
    def get_int(self, n): raise NotImplementedError
    def get_float(self, n): raise NotImplementedError
    def get_str(self, n): raise NotImplementedError
    def get_image(self): raise NotImplementedError   # -> (w, h, rgb bytes, row 0 = image top) or None

    # convenience
    def get_floats(self, n):
        b = self.get_str(n)
        if not b or len(b) % 4: return None
        return list(struct.unpack('<%df' % (len(b) // 4), b))

    def set_floats(self, n, values):
        self.set_str(n, struct.pack('<%df' % len(values), *values))


class RemoteApiLink(BaseLink):
    """Legacy remote API (the only one CoppeliaSim 4.1 has).  Port 19997 is the always-on
    'continuous remote API server service' of CoppeliaSim, so the dashboard can also start/stop
    the simulation."""
    name = 'CoppeliaSim'

    def __init__(self, host='127.0.0.1', port=19997):
        try:
            import sim as _sim
        except ImportError:
            raise SystemExit("Cannot import 'sim' (legacy remote API client).\n"
                             "Copy sim.py, simConst.py and the remoteApi library from\n"
                             "  <CoppeliaSim>/programming/remoteApiBindings/python/python/  and  .../lib/lib/\n"
                             "next to this script, or run with --mock.")
        self.s, self.host, self.port, self.cid = _sim, host, port, -1
        self.lock = threading.RLock()
        self._streams = set()
        self._cam = -1

    def connect(self):
        s = self.s
        with self.lock:
            s.simxFinish(-1)
            self.cid = s.simxStart(self.host, self.port, True, True, 3000, 5)
            self._streams.clear()
            if self.cid == -1: return False
            r, self._cam = s.simxGetObjectHandle(self.cid, 'Belt_Camera', s.simx_opmode_blocking)
            if r != s.simx_return_ok: self._cam = -1
        return True

    def close(self):
        with self.lock:
            if self.cid != -1: self.s.simxFinish(self.cid)
            self.cid = -1

    def sim_start(self):
        with self.lock: self.s.simxStartSimulation(self.cid, self.s.simx_opmode_blocking)
    def sim_stop(self):
        with self.lock: self.s.simxStopSimulation(self.cid, self.s.simx_opmode_blocking)
    def sim_pause(self):
        with self.lock: self.s.simxPauseSimulation(self.cid, self.s.simx_opmode_blocking)

    def set_int(self, n, v):
        with self.lock: self.s.simxSetIntegerSignal(self.cid, n, int(v), self.s.simx_opmode_oneshot)
    def set_float(self, n, v):
        with self.lock: self.s.simxSetFloatSignal(self.cid, n, float(v), self.s.simx_opmode_oneshot)
    def set_str(self, n, b):
        with self.lock: self.s.simxSetStringSignal(self.cid, n, bytes(b), self.s.simx_opmode_oneshot)

    def _get(self, key, call):
        s = self.s
        with self.lock:
            if self.cid == -1: return None
            mode = s.simx_opmode_buffer if key in self._streams else s.simx_opmode_streaming
            res = call(mode)
            self._streams.add(key)
        return res[1] if res[0] == s.simx_return_ok else None

    def get_int(self, n): return self._get(n, lambda m: self.s.simxGetIntegerSignal(self.cid, n, m))
    def get_float(self, n): return self._get(n, lambda m: self.s.simxGetFloatSignal(self.cid, n, m))
    def get_str(self, n):
        v = self._get(n, lambda m: self.s.simxGetStringSignal(self.cid, n, m))
        return None if v is None else bytes(v)

    def get_image(self):
        s = self.s
        if self._cam == -1: return None
        with self.lock:
            mode = s.simx_opmode_buffer if 'img' in self._streams else s.simx_opmode_streaming
            r, res, img = s.simxGetVisionSensorImage(self.cid, self._cam, 0, mode)
            self._streams.add('img')
        if r != s.simx_return_ok or not img: return None
        w, h = res
        raw = bytes(v & 255 for v in img)                 # OpenGL order: first row = bottom
        rows = [raw[i * w * 3:(i + 1) * w * 3] for i in range(h)]
        return w, h, b''.join(reversed(rows))


# ----------------------------------------------------------------- mock sim --
class MockSim(threading.Thread):
    """Tiny re-implementation of robot_controller.lua + scene geometry (same signal protocol),
    so dashboard and IK can be exercised without CoppeliaSim."""
    DT = 0.05
    VMAX = [1.8, 1.4, 1.6, 2.2, 3.5]
    ITEM = 0.08

    def __init__(self, speed=1.0):
        super().__init__(daemon=True)
        self.speed, self.run_flag, self.lock = speed, True, threading.RLock()
        self.sig = {}
        bx, by = -0.60, 0.55
        self.limits = [(-3.1, 3.1), (-1.4, 1.9), (0.0, 2.8), (-1.0, 3.0), (-3.1, 3.1)]
        self.kin = ArmKinematics(bx, by, 0.55, 0.40, 0.35, 0.15, self.limits)
        cfg = [bx, by, 0.55, 0.40, 0.35, 0.15, 0.40, self.ITEM, -0.60, 0.0, -0.60, 0.0, 0.5]
        for ang in (30, 90, 150):
            cfg += [bx + 0.55 * math.cos(math.radians(ang)), by + 0.55 * math.sin(math.radians(ang)), 0.44]
        self.cfg = cfg
        self.sig['scene_cfg'] = struct.pack('<%df' % len(cfg), *cfg)
        self.sig['joint_limits'] = struct.pack('<10f', *[v for lh in self.limits for v in lh])
        self.q = [0.0] * 5
        self.wps, self.wpi, self.last_id, self.done_id = [], 0, 0, 0
        self.items, self.n_items, self.held, self.station = [], 0, None, None
        self.spawn_t, self.steps, self.simt, self.missed, self.spawned = 2.0, 0, 0.0, 0, 0
        self.last_discard = 0
        self.rng = random.Random(3)
        for k, v in dict(arm_done_id=0, grip_state=0, prox_detect=0, belt_moving=0, estop=0).items():
            self.sig[k] = v

    # signal helpers
    def gi(self, n, d): return self.sig.get(n, d)

    def run(self):
        while self.run_flag:
            t0 = time.time()
            with self.lock: self.step(self.DT)
            time.sleep(max(0.0, self.DT / self.speed - (time.time() - t0)))

    def step(self, dt):
        s = self.sig
        estop = self.gi('estop', 0) == 1
        d = self.gi('discard_item', 0)
        if d != self.last_discard:
            self.last_discard = d
            if self.station: self.items.remove(self.station); self.station = None
        # ---- arm
        pid = self.gi('arm_path_id', 0)
        if pid != self.last_id:
            self.last_id = pid
            b = s.get('arm_path', b'')
            n = len(b) // 20
            v = struct.unpack('<%df' % (5 * n), b[:20 * n]) if n else ()
            self.wps = [list(v[i * 5:i * 5 + 5]) for i in range(n)]
            self.wpi = 0
            if not self.wps: self.done_id = pid
        if not estop and self.wpi < len(self.wps):
            remain = dt * max(0.05, self.gi('arm_speed_scale', 1.0))
            while remain > 1e-9 and self.wpi < len(self.wps):
                tg = self.wps[self.wpi]
                need = max(abs(tg[i] - self.q[i]) / self.VMAX[i] for i in range(5))
                if need <= remain:
                    self.q = list(tg); remain -= need; self.wpi += 1
                else:
                    f = remain / need
                    self.q = [self.q[i] + (tg[i] - self.q[i]) * f for i in range(5)]
                    remain = 0
            if self.wpi >= len(self.wps): self.done_id = self.last_id
        elif self.wpi >= len(self.wps):
            self.done_id = self.last_id
        s['arm_done_id'] = self.done_id
        s['arm_q'] = struct.pack('<5f', *self.q)
        tip = self.kin.fk(self.q)
        # ---- gripper
        cmd = self.gi('grip_cmd', 0)
        if not estop:
            if cmd == 1 and self.held is None:
                best, bd = None, 0.075
                for it in self.items:
                    if it['state'] == 'belt':
                        dd = math.dist((it['x'], it['y'], it['z']), tip)
                        if dd < bd: best, bd = it, dd
                if best:
                    best['state'] = 'held'; self.held = best
                    if self.station is best: self.station = None
            elif cmd == 0 and self.held is not None:
                self.held['state'] = 'dropped'; self.held['t'] = 0; self.held = None
        if self.held:
            self.held.update(x=tip[0], y=tip[1], z=tip[2] - self.ITEM / 2)
        s['grip_state'] = 1 if self.held else 0
        # ---- conveyor / spawner
        run, speed = self.gi('belt_run', 1) == 1, float(self.gi('belt_speed', 0.12))
        halted = estop or not run or self.station is not None
        moving = False
        for it in list(self.items):
            if it['state'] == 'belt' and not halted:
                it['x'] += speed * dt; moving = True
                if it['x'] > 0.40: it['state'] = 'missed'; it['t'] = 0; self.missed += 1
            elif it['state'] in ('missed', 'dropped'):
                it['t'] += dt
                if it['t'] > 1.5: self.items.remove(it)
        if not estop and self.gi('spawn_enable', 1) == 1:
            self.spawn_t -= dt
            if self.spawn_t <= 0 and not any(i['state'] == 'belt' and abs(i['x'] + 1.8) < 0.25 for i in self.items):
                c = self.rng.choice((1, 2, 3))
                self.items.append(dict(x=-1.8, y=(self.rng.random() - 0.5) * 0.16, z=0.44, cls=c, state='belt', t=0))
                self.spawned += 1
                self.spawn_t = float(self.gi('spawn_interval', 6.0))
        # ---- proximity sensor
        if self.station is None:
            for it in self.items:
                if it['state'] == 'belt' and it['x'] + 0.04 >= -0.60 >= it['x'] - 0.04:
                    self.station = it
        elif self.station['state'] != 'belt':
            self.station = None
        s['prox_detect'] = 1 if self.station else 0
        if self.station:
            st = self.station
            s['station_gt'] = struct.pack('<4f', st['x'], st['y'], st['z'], st['cls'])
        s['belt_moving'] = 1 if moving else 0
        self.steps += 1; self.simt += dt
        s['hb'] = self.steps; s['sim_time'] = self.simt
        s['cnt_spawned'] = self.spawned; s['cnt_missed'] = self.missed

    def render(self, w=128, h=128):
        img = bytearray(bytes((44, 44, 48)) * (w * h))
        size = 0.5
        cx, cy = self.cfg[10], self.cfg[11]
        for it in self.items:
            col = {1: (26, 77, 242), 2: (242, 204, 26), 3: (26, 179, 51)}[it['cls']]
            u0 = int((it['x'] - self.ITEM / 2 - (cx - size / 2)) / size * w)
            u1 = int((it['x'] + self.ITEM / 2 - (cx - size / 2)) / size * w)
            v0 = int((it['y'] - self.ITEM / 2 - (cy - size / 2)) / size * h)
            v1 = int((it['y'] + self.ITEM / 2 - (cy - size / 2)) / size * h)
            for v in range(max(0, v0), min(h, v1)):
                for u in range(max(0, u0), min(w, u1)):
                    img[(v * w + u) * 3:(v * w + u) * 3 + 3] = bytes(col)
        return w, h, bytes(img)


class MockLink(BaseLink):
    name = 'MOCK'

    def __init__(self, speed=1.0):
        self.speed, self.m, self.connected = speed, None, False
        self.pending = {}

    def connect(self): self.connected = True; return True
    def sim_start(self):
        self.sim_stop()
        self.m = MockSim(self.speed)
        for k, v in self.pending.items(): self.m.sig[k] = v
        self.m.start()
    def sim_stop(self):
        if self.m:
            self.m.run_flag = False; self.m.join(timeout=1.0); self.m = None
    def close(self): self.sim_stop()

    def _set(self, n, v):
        if self.m:
            with self.m.lock: self.m.sig[n] = v
    def set_int(self, n, v): self._set(n, int(v))
    def set_float(self, n, v): self._set(n, float(v))
    def set_str(self, n, b): self._set(n, bytes(b))
    def _get(self, n):
        if not self.m: return None
        with self.m.lock: return self.m.sig.get(n)
    def get_int(self, n): return self._get(n)
    def get_float(self, n): return self._get(n)
    def get_str(self, n): return self._get(n)
    def get_image(self):
        if not self.m: return None
        with self.m.lock: return self.m.render()


# =============================================================================
#  3.  CONTROLLER  (state machine running in its own thread)
# =============================================================================
class Abort(Exception): pass
class CellError(Exception): pass


class CellController(threading.Thread):
    Z_SAFE = 0.78            # tip height for travelling over the bins
    HOVER = 0.12             # hover above item top before descending
    READY_ANGLE = -45.0      # waiting pose is off to the side so it never hides the camera
    READY_R, READY_Z = 0.50, 0.80

    def __init__(self, link, log, use_ground_truth=False):
        super().__init__(daemon=True)
        self.link, self.log, self.use_gt = link, log, use_ground_truth
        self.lock = threading.RLock()
        self.kin = None
        self.cfg = None
        self.state = 'DISCONNECTED'
        self.active = False            # robot enabled (Start pressed)
        self.stop_after_cycle = False
        self.paused = False
        self.estopped = False
        self.abort = False
        self.quit = False
        self.path_id = 0
        self.q_cmd = [0.0] * 5
        self.held_cls = None
        self.stats = dict(sorted={1: 0, 2: 0, 3: 0}, failed=0, ik_errors=0, cycles=0,
                          cycle_times=deque(maxlen=20), t_start=None, last_cycle=0.0)
        self.last_det = None           # (cls, x, y, gt)
        self.last_error = ''

    # ----------------------------------------------------------- set-up -----
    def load_scene(self):
        cfg = self.link.get_floats('scene_cfg')
        lim = self.link.get_floats('joint_limits')
        if not cfg or len(cfg) < 22 or not lim: return False
        self.cfg = cfg
        limits = [(lim[2 * i], lim[2 * i + 1]) for i in range(5)]
        self.kin = ArmKinematics(cfg[0], cfg[1], cfg[2], cfg[3], cfg[4], cfg[5], limits)
        self.belt_top, self.item = cfg[6], cfg[7]
        self.pick_x, self.pick_y = cfg[8], cfg[9]
        self.cam = (cfg[10], cfg[11], cfg[12])
        self.drops = {1: tuple(cfg[13:16]), 2: tuple(cfg[16:19]), 3: tuple(cfg[19:22])}
        a = math.radians(self.READY_ANGLE)
        self.ready = (cfg[0] + self.READY_R * math.cos(a), cfg[1] + self.READY_R * math.sin(a), self.READY_Z)
        done = self.link.get_int('arm_done_id')
        self.path_id = done if done is not None else 0
        q = self.link.get_floats('arm_q')
        self.q_cmd = q if q else [0.0] * 5
        return True

    # ---------------------------------------------------------- commands ----
    def set_state(self, s):
        if s != self.state: self.state = s

    def cmd_start(self):
        with self.lock:
            if self.estopped: self.log('E-STOP is active - reset it first'); return
            self.stop_after_cycle = False
            self.paused = False
            self.link.set_int('estop', 0)
            if not self.active:
                self.active = True
                self.stats['t_start'] = self.stats['t_start'] or time.time()
                self.log('robot started')

    def cmd_pause(self):
        with self.lock:
            if not self.active: return
            self.paused = not self.paused
            self.link.set_int('estop', 1 if self.paused else 0)   # freezes arm + belt inside the sim
            self.log('paused' if self.paused else 'resumed')

    def cmd_stop(self):
        with self.lock:
            if self.active:
                self.stop_after_cycle = True
                self.log('stop requested - finishing the current cycle')

    def cmd_estop(self):
        with self.lock:
            self.estopped, self.abort = True, True
            self.link.set_int('estop', 1)
            self.log('*** EMERGENCY STOP ***')

    def cmd_reset_estop(self):
        with self.lock:
            if not self.estopped: return
            self.estopped, self.abort, self.paused, self.active = False, False, False, False
            self.link.set_int('estop', 0)
            self.set_state('STOPPED')
            self.log('E-stop released - press START to resume')

    # ------------------------------------------------------- primitives -----
    def _check(self):
        if self.abort or self.quit: raise Abort()
        while self.paused and not self.abort and not self.quit: time.sleep(0.05)
        if self.abort or self.quit: raise Abort()

    def _wait(self, cond, timeout, what):
        t0 = time.time()
        while True:
            self._check()
            if cond(): return True
            if self.paused: t0 = time.time()
            if time.time() - t0 > timeout:
                return False
            time.sleep(0.02)

    def _send_path(self, qs):
        self.path_id += 1
        self.link.set_floats('arm_path', [v for q in qs for v in q])
        self.link.set_int('arm_path_id', self.path_id)
        if not self._wait(lambda: self.link.get_int('arm_done_id') == self.path_id, 90.0, 'arm'):
            raise CellError('arm did not finish its motion (timeout)')
        self.q_cmd = list(qs[-1])

    def _tip(self): return self.kin.fk(self.q_cmd)

    def move_to(self, x, y, z, step=0.02):
        qs = self.kin.cartesian_path(self._tip(), (x, y, z), step)   # raises IKError before moving
        self._send_path(qs)

    def go_ready(self):
        q_t = self.kin.ik(*self.ready)
        self._send_path(self.kin.joint_path(self.q_cmd, q_t))

    def _grip(self, on):
        self.link.set_int('grip_cmd', 1 if on else 0)
        ok = self._wait(lambda: self.link.get_int('grip_state') == (1 if on else 0), 2.0, 'grip')
        if not on: time.sleep(0.4)
        return ok

    # ------------------------------------------------------- perception -----
    def detect(self):
        im = self.link.get_image()
        if not im: return None
        w, h, rgb = im
        cnt, su, sv = {1: 0, 2: 0, 3: 0}, {1: 0, 2: 0, 3: 0}, {1: 0, 2: 0, 3: 0}
        for v in range(h):
            row = v * w * 3
            for u in range(w):
                r, g, b = rgb[row + u * 3] / 255.0, rgb[row + u * 3 + 1] / 255.0, rgb[row + u * 3 + 2] / 255.0
                mx = max(r, g, b)
                if mx < 0.2 or (mx - min(r, g, b)) / mx < 0.45: continue
                hue = colorsys.rgb_to_hsv(r, g, b)[0]
                c = 1 if 0.5 <= hue <= 0.72 else 2 if 0.09 <= hue <= 0.20 else 3 if 0.25 <= hue <= 0.45 else 0
                if c:
                    cnt[c] += 1; su[c] += u; sv[c] += v
        cls = max(cnt, key=cnt.get)
        if cnt[cls] < 60: return None
        u, v = su[cls] / cnt[cls], sv[cls] / cnt[cls]
        cx, cy, size = self.cam
        x = cx + ((u + 0.5) / w - 0.5) * size      # image right  = +x
        y = cy + ((v + 0.5) / h - 0.5) * size      # image down   = +y   (camera rotated 180 deg about x)
        return cls, x, y, (u, v)

    # ------------------------------------------------------------- cycle ----
    def place(self, cls):
        dx, dy, dz = self.drops[cls]
        tx, ty, _ = self._tip()
        self.set_state('TRANSIT')
        self.move_to(tx, ty, self.Z_SAFE)
        self.move_to(dx, dy, self.Z_SAFE, step=0.02)
        self.move_to(dx, dy, dz)
        self.set_state('RELEASE')
        self._grip(False)
        self.held_cls = None
        self.stats['sorted'][cls] += 1
        self.set_state('RETURN')
        self.move_to(dx, dy, self.Z_SAFE)
        self.go_ready_cart()

    def go_ready_cart(self):
        self.move_to(*self.ready)

    def one_cycle(self):
        L = self.link
        t_cycle = time.time()
        self.set_state('WAIT_ITEM')
        L.set_int('belt_run', 1)
        while not self._wait(lambda: L.get_int('prox_detect') == 1, 0.5, 'item'):
            if self.stop_after_cycle: return False
        self.set_state('DETECT')
        time.sleep(0.3)
        det = self.detect()
        gt = L.get_floats('station_gt')
        if self.use_gt and gt and gt[3] > 0:
            det = (int(gt[3]), gt[0], gt[1], None)
        if det is None:
            self.log('camera could not classify the item -> discarded')
            self.stats['failed'] += 1
            L.set_int('discard_item', (L.get_int('discard_item') or 0) + 1)
            return True
        cls, x, y, pix = det
        self.last_det = (cls, x, y, pix)       # pix = (u, v) image pixel, None when ground truth is used
        err = ''
        if gt and gt[3] > 0:
            err = '  (camera error %.1f mm%s)' % (1000 * math.hypot(x - gt[0], y - gt[1]),
                                                   '' if int(gt[3]) == cls else ', CLASS MISMATCH')
        self.log('detected %s at (%.3f, %.3f)%s' % (CLASS_NAMES[cls], x, y, err))

        z_top = self.belt_top + self.item
        try:
            self.set_state('APPROACH')
            self.move_to(x, y, z_top + self.HOVER)
            self.set_state('DESCEND')
            self.move_to(x, y, z_top + 0.004, step=0.01)
        except IKError as e:
            self.stats['ik_errors'] += 1
            self.log('IK: %s -> item discarded' % e)
            self.discard_station()
            self.go_ready()
            return True
        self.set_state('GRAB')
        if not self._grip(True):
            self.log('grip failed -> retry 6 mm lower')
            self.move_to(x, y, z_top - 0.002, step=0.006)
            if not self._grip(True):
                self.stats['failed'] += 1
                self.log('grip failed twice -> item discarded')
                self.discard_station()
                self.move_to(x, y, z_top + self.HOVER)
                self.go_ready_cart()
                return True
        self.held_cls = cls
        self.set_state('LIFT')
        self.move_to(x, y, z_top + self.HOVER)
        self.place(cls)
        dt = time.time() - t_cycle
        self.stats['cycles'] += 1
        self.stats['cycle_times'].append(dt)
        self.stats['last_cycle'] = dt
        return True

    def discard_station(self):
        self.link.set_int('grip_cmd', 0)
        self.link.set_int('discard_item', (self.link.get_int('discard_item') or 0) + 1)

    # -------------------------------------------------------------- run -----
    def run(self):
        while not self.quit:
            if not self.active or self.estopped or self.kin is None:
                if self.estopped: self.set_state('ESTOP')
                time.sleep(0.1)
                continue
            try:
                self.abort = False
                self.link.set_int('spawn_enable', self.spawn_wanted if hasattr(self, 'spawn_wanted') else 1)
                if self.link.get_int('grip_state') == 1 and self.held_cls:
                    self.log('recovering: placing the held item')
                    self.place(self.held_cls)
                elif self.link.get_int('grip_state') == 1:
                    self.link.set_int('grip_cmd', 0)
                else:
                    self.go_ready_cart() if self._near_ready() else self.go_ready()
                while self.active and not self.stop_after_cycle:
                    if not self.one_cycle(): break
                self.link.set_int('belt_run', 0)
                self.set_state('STOPPED')
                self.active = False
                self.log('robot stopped (cycle finished)')
            except Abort:
                self.set_state('ESTOP' if self.estopped else 'STOPPED')
                self.active = False
                self.log('motion aborted')
            except (IKError, CellError) as e:
                self.last_error = str(e)
                self.log('ERROR: %s' % e)
                self.set_state('STOPPED'); self.active = False
            except Exception as e:           # connection problems etc.
                self.last_error = repr(e)
                self.log('UNEXPECTED ERROR: %r' % e)
                self.set_state('STOPPED'); self.active = False

    def _near_ready(self):
        t = self._tip()
        return math.dist(t, self.ready) < 0.3 and t[2] > 0.5


# =============================================================================
#  4.  DASHBOARD
# =============================================================================
class Dashboard:
    def __init__(self, link, ctrl, log_q):
        import tkinter as tk
        from tkinter import ttk
        self.tk, self.ttk = tk, ttk
        self.link, self.ctrl, self.log_q = link, ctrl, log_q
        self.root = tk.Tk()
        self.root.title('Waste Sorting Cell - Pick & Place Dashboard')
        self.root.geometry('1260x900')
        self.root.configure(bg='#14181f')
        self._style()
        self.last_hb, self.last_hb_t, self.sim_running = None, 0.0, False
        self.cam_photo = None
        self.cam_t = 0.0
        self.build()
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)
        self.root.after(200, self.refresh)

    # ------------------------------------------------------------ styling ---
    def _style(self):
        st = self.ttk.Style()
        try: st.theme_use('clam')
        except Exception: pass
        bg, fg = '#14181f', '#e6e9ef'
        st.configure('.', background=bg, foreground=fg, fieldbackground='#1d232d')
        st.configure('TLabelframe', background=bg, foreground='#8aa0c0')
        st.configure('TLabelframe.Label', background=bg, foreground='#8aa0c0', font=('Segoe UI', 10, 'bold'))
        st.configure('TButton', font=('Segoe UI', 10, 'bold'), padding=6)
        st.configure('Big.TButton', font=('Segoe UI', 12, 'bold'), padding=10)
        st.configure('TCheckbutton', background=bg, foreground=fg)
        st.configure('TLabel', background=bg, foreground=fg)
        st.configure('Horizontal.TProgressbar', troughcolor='#242c38', background='#3b82f6')

    def build(self):
        tk, ttk = self.tk, self.ttk
        r = self.root
        top = ttk.Frame(r); top.pack(fill='x', padx=10, pady=(10, 4))
        self.lbl_link = ttk.Label(top, text='LINK: -', font=('Segoe UI', 10, 'bold')); self.lbl_link.pack(side='left', padx=8)
        self.lbl_sim = ttk.Label(top, text='SIM: -', font=('Segoe UI', 10, 'bold')); self.lbl_sim.pack(side='left', padx=8)
        self.lbl_state = tk.Label(top, text='STATE', width=16, bg='#555', fg='white', font=('Segoe UI', 13, 'bold'))
        self.lbl_state.pack(side='right', padx=8)

        main = ttk.Frame(r); main.pack(fill='both', expand=True, padx=10)
        left = ttk.Frame(main); left.pack(side='left', fill='y')
        mid = ttk.Frame(main); mid.pack(side='left', fill='both', expand=True, padx=10)
        right = ttk.Frame(main); right.pack(side='left', fill='y')

        # --- controls
        c = ttk.LabelFrame(left, text='CONNECTION / SIMULATION'); c.pack(fill='x', pady=4)
        ttk.Button(c, text='Connect', command=self.on_connect).pack(fill='x', padx=6, pady=2)
        row = ttk.Frame(c); row.pack(fill='x', padx=6, pady=2)
        ttk.Button(row, text='Start sim', command=self.on_sim_start).pack(side='left', expand=True, fill='x')
        ttk.Button(row, text='Pause sim', command=lambda: self.link.sim_pause()).pack(side='left', expand=True, fill='x')
        ttk.Button(row, text='Stop sim', command=self.on_sim_stop).pack(side='left', expand=True, fill='x')

        k = ttk.LabelFrame(left, text='ROBOT'); k.pack(fill='x', pady=4)
        ttk.Button(k, text='START', style='Big.TButton', command=self.ctrl.cmd_start).pack(fill='x', padx=6, pady=3)
        ttk.Button(k, text='PAUSE / RESUME', command=self.ctrl.cmd_pause).pack(fill='x', padx=6, pady=3)
        ttk.Button(k, text='STOP (after cycle)', command=self.ctrl.cmd_stop).pack(fill='x', padx=6, pady=3)
        tk.Button(k, text='E-STOP', bg='#c62828', fg='white', activebackground='#e53935', font=('Segoe UI', 14, 'bold'),
                  relief='flat', command=self.ctrl.cmd_estop).pack(fill='x', padx=6, pady=(8, 3), ipady=6)
        ttk.Button(k, text='Reset E-STOP', command=self.ctrl.cmd_reset_estop).pack(fill='x', padx=6, pady=3)

        s = ttk.LabelFrame(left, text='PROCESS SETTINGS'); s.pack(fill='x', pady=4)
        self.v_belt = tk.DoubleVar(value=0.12)
        self.v_arm = tk.DoubleVar(value=1.0)
        self.v_spawn = tk.DoubleVar(value=6.0)
        self.v_spawn_en = tk.IntVar(value=1)
        self.v_belt_en = tk.IntVar(value=1)
        for text, var, lo, hi, cb in (('Belt speed [m/s]', self.v_belt, 0.02, 0.30, self.apply_settings),
                                      ('Arm speed scale', self.v_arm, 0.2, 2.0, self.apply_settings),
                                      ('Spawn interval [s]', self.v_spawn, 2.0, 15.0, self.apply_settings)):
            ttk.Label(s, text=text).pack(anchor='w', padx=6)
            tk.Scale(s, variable=var, from_=lo, to=hi, resolution=0.01 if hi < 1 else 0.1, orient='horizontal',
                     bg='#14181f', fg='#e6e9ef', troughcolor='#242c38', highlightthickness=0,
                     command=lambda _v, cb=cb: cb()).pack(fill='x', padx=6)
        ttk.Checkbutton(s, text='Spawn waste items', variable=self.v_spawn_en, command=self.apply_settings).pack(anchor='w', padx=6)
        ttk.Checkbutton(s, text='Conveyor enabled', variable=self.v_belt_en, command=self.apply_settings).pack(anchor='w', padx=6)

        # --- camera + map
        cam = ttk.LabelFrame(mid, text='CAMERA  (top-down, belt station)'); cam.pack(fill='x')
        self.cv_cam = tk.Canvas(cam, width=384, height=384, bg='#000', highlightthickness=0); self.cv_cam.pack(padx=6, pady=6)
        mp = ttk.LabelFrame(mid, text='PLAN VIEW'); mp.pack(fill='both', expand=True, pady=6)
        self.cv_map = tk.Canvas(mp, width=384, height=250, bg='#0e1116', highlightthickness=0); self.cv_map.pack(padx=6, pady=6)

        # --- status
        st = ttk.LabelFrame(right, text='STATUS'); st.pack(fill='x', pady=4)
        self.status_vars = {}
        for key in ('Robot state', 'Gripper', 'Proximity sensor', 'Conveyor', 'Sim time', 'Last detection', 'Cycle time (last / avg)', 'Throughput'):
            f = ttk.Frame(st); f.pack(fill='x', padx=6, pady=1)
            ttk.Label(f, text=key, width=22).pack(side='left')
            v = tk.StringVar(value='-'); self.status_vars[key] = v
            ttk.Label(f, textvariable=v, font=('Consolas', 10, 'bold'), width=24).pack(side='left')

        cn = ttk.LabelFrame(right, text='SORTED COUNTERS'); cn.pack(fill='x', pady=4)
        self.cnt_vars = {}
        for cls in (1, 2, 3):
            f = ttk.Frame(cn); f.pack(fill='x', padx=6, pady=1)
            tk.Label(f, text='  ', bg=CLASS_COLORS[cls]).pack(side='left')
            ttk.Label(f, text=CLASS_NAMES[cls], width=10).pack(side='left', padx=4)
            v = tk.StringVar(value='0'); self.cnt_vars[cls] = v
            ttk.Label(f, textvariable=v, font=('Consolas', 14, 'bold')).pack(side='left')
        for key in ('Spawned', 'Missed (fell off belt)', 'Failed / IK errors'):
            f = ttk.Frame(cn); f.pack(fill='x', padx=6, pady=1)
            ttk.Label(f, text=key, width=22).pack(side='left')
            v = tk.StringVar(value='0'); self.cnt_vars[key] = v
            ttk.Label(f, textvariable=v, font=('Consolas', 10, 'bold')).pack(side='left')

        jf = ttk.LabelFrame(right, text='JOINTS  (deg)'); jf.pack(fill='x', pady=4)
        self.jvars, self.jbars = [], []
        for i in range(5):
            f = ttk.Frame(jf); f.pack(fill='x', padx=6, pady=1)
            ttk.Label(f, text='J%d' % (i + 1), width=3).pack(side='left')
            pb = ttk.Progressbar(f, length=120, maximum=100); pb.pack(side='left', padx=4)
            v = tk.StringVar(value='0.0'); ttk.Label(f, textvariable=v, width=8, font=('Consolas', 10)).pack(side='left')
            self.jbars.append(pb); self.jvars.append(v)
        self.tip_var = tk.StringVar(value='TCP  x=-  y=-  z=-')
        ttk.Label(jf, textvariable=self.tip_var, font=('Consolas', 10)).pack(anchor='w', padx=6, pady=3)

        lg = ttk.LabelFrame(r, text='LOG'); lg.pack(fill='x', padx=10, pady=8)
        self.txt = tk.Text(lg, height=7, bg='#0e1116', fg='#c9d1d9', font=('Consolas', 9), relief='flat')
        self.txt.pack(fill='x', padx=4, pady=4)

    # ------------------------------------------------------------ actions ---
    def on_connect(self):
        ok = self.link.connect()
        self.log_q.append('connected to %s' % self.link.name if ok else 'connection FAILED (is CoppeliaSim running?)')
        if ok and not self.ctrl.kin: self.ctrl.set_state('IDLE')

    def on_sim_start(self):
        self.link.sim_start(); self.log_q.append('simulation start requested')
        self.ctrl.kin = None            # force re-reading of the scene configuration

    def on_sim_stop(self):
        self.ctrl.cmd_stop(); self.ctrl.abort = True
        self.link.sim_stop(); self.ctrl.active = False
        self.log_q.append('simulation stopped')

    def apply_settings(self):
        L = self.link
        L.set_float('belt_speed', self.v_belt.get())
        L.set_float('arm_speed_scale', self.v_arm.get())
        L.set_float('spawn_interval', self.v_spawn.get())
        self.ctrl.spawn_wanted = self.v_spawn_en.get()
        L.set_int('spawn_enable', self.v_spawn_en.get())
        if not self.ctrl.active:
            L.set_int('belt_run', self.v_belt_en.get())

    def on_close(self):
        self.ctrl.quit = True
        try:
            self.link.set_int('belt_run', 0); self.link.close()
        except Exception: pass
        self.root.destroy()

    # ------------------------------------------------------------ refresh ---
    def refresh(self):
        tk = self.tk
        L, C = self.link, self.ctrl
        try:
            hb = L.get_int('hb')
            now = time.time()
            if hb is not None and hb != self.last_hb:
                self.last_hb, self.last_hb_t, self.sim_running = hb, now, True
            elif now - self.last_hb_t > 1.5:
                self.sim_running = False
            self.lbl_link.config(text='LINK: %s' % L.name, foreground='#4ade80')
            self.lbl_sim.config(text='SIM: %s' % ('RUNNING' if self.sim_running else 'not running'),
                                foreground='#4ade80' if self.sim_running else '#f87171')
            if self.sim_running and C.kin is None and C.load_scene():
                C.set_state('IDLE'); self.log_q.append('scene geometry loaded: L1=%.2f L2=%.2f Lt=%.2f' % (C.kin.l1, C.kin.l2, C.kin.lt))
                self.apply_settings()
            if not self.sim_running and not C.estopped:
                C.set_state('DISCONNECTED' if not C.active else C.state)

            st = C.state
            self.lbl_state.config(text=st, bg=STATE_COLORS.get(st, '#555'))
            sv = self.status_vars
            sv['Robot state'].set(st)
            grip = L.get_int('grip_state'); prox = L.get_int('prox_detect'); mv = L.get_int('belt_moving')
            sv['Gripper'].set('HOLDING ITEM' if grip else 'empty')
            sv['Proximity sensor'].set('ITEM AT STATION' if prox else 'clear')
            sv['Conveyor'].set('RUNNING' if mv else 'stopped / holding')
            t = L.get_float('sim_time'); sv['Sim time'].set('%.1f s' % t if t is not None else '-')
            if C.last_det:
                cls, x, y, _ = C.last_det
                sv['Last detection'].set('%s (%.2f,%.2f)' % (CLASS_NAMES[cls], x, y))
            ct = C.stats['cycle_times']
            if ct:
                sv['Cycle time (last / avg)'].set('%.1f / %.1f s' % (C.stats['last_cycle'], sum(ct) / len(ct)))
                if C.stats['t_start']:
                    sv['Throughput'].set('%.1f items/min' % (C.stats['cycles'] / max(1e-3, (now - C.stats['t_start']) / 60.0)))
            for cls in (1, 2, 3): self.cnt_vars[cls].set(str(C.stats['sorted'][cls]))
            self.cnt_vars['Spawned'].set(str(L.get_int('cnt_spawned') or 0))
            self.cnt_vars['Missed (fell off belt)'].set(str(L.get_int('cnt_missed') or 0))
            self.cnt_vars['Failed / IK errors'].set('%d / %d' % (C.stats['failed'], C.stats['ik_errors']))

            q = L.get_floats('arm_q')
            if q and C.kin:
                for i in range(5):
                    lo, hi = C.kin.limits[i]
                    self.jbars[i]['value'] = 100.0 * (q[i] - lo) / (hi - lo)
                    self.jvars[i].set('%7.1f' % math.degrees(q[i]))
                tip = C.kin.fk(q)
                self.tip_var.set('TCP  x=%6.3f  y=%6.3f  z=%6.3f' % tip)
                self.draw_map(q, tip)
            if now - self.cam_t > 0.4:
                self.cam_t = now
                self.draw_camera()
        except Exception as e:            # never let the GUI die
            self.log_q.append('GUI refresh error: %r' % e)
        while self.log_q:
            self.txt.insert('end', self.log_q.popleft() + '\n'); self.txt.see('end')
            if int(self.txt.index('end-1c').split('.')[0]) > 400: self.txt.delete('1.0', '100.0')
        self.root.after(100, self.refresh)

    def draw_camera(self):
        tk = self.tk
        im = self.link.get_image()
        cv = self.cv_cam
        cv.delete('all')
        if not im:
            cv.create_text(192, 192, text='no camera image', fill='#888'); return
        w, h, rgb = im
        photo = tk.PhotoImage(data=b'P6 %d %d 255\n' % (w, h) + rgb)
        z = max(1, 384 // w)
        self.cam_photo = photo.zoom(z, z)
        cv.create_image(0, 0, image=self.cam_photo, anchor='nw')
        d = self.ctrl.last_det
        if d and d[3] and self.ctrl.state in ('DETECT', 'APPROACH', 'DESCEND', 'GRAB'):
            u, v = d[3]
            cv.create_oval(u * z - 8, v * z - 8, u * z + 8, v * z + 8, outline='#ff2d55', width=2)
            cv.create_text(u * z + 12, v * z - 12, text=CLASS_NAMES[d[0]], fill='white', anchor='w', font=('Segoe UI', 9, 'bold'))

    def draw_map(self, q, tip):
        C = self.ctrl
        cv = self.cv_map
        cv.delete('all')
        W, H = 384, 250
        sc = 105.0
        ox, oy = 170, 110              # world (x,y) = (-0.6, 0.55) near here
        def P(x, y): return ox + (x + 0.6) * sc, oy + (0.55 - y) * sc * 1.0 + 0
        # belt
        x0, y0 = P(-2.0, 0.25); x1, y1 = P(0.4, -0.25)
        cv.create_rectangle(x0, y0, x1, y1, fill='#2a2a30', outline='#555')
        px, py = P(C.pick_x, C.pick_y)
        cv.create_line(px, py - 28, px, py + 28, fill='#ef4444', dash=(3, 3))
        for cls in (1, 2, 3):
            dx, dy, _ = C.drops[cls]
            bx, by = P(dx, dy)
            cv.create_rectangle(bx - 16, by - 16, bx + 16, by + 16, outline=CLASS_COLORS[cls], width=3)
        rx, ry = P(C.kin.bx, C.kin.by)
        tx, ty = P(tip[0], tip[1])
        cv.create_oval(rx - 10, ry - 10, rx + 10, ry + 10, fill='#475569', outline='')
        cv.create_line(rx, ry, tx, ty, fill='#f97316', width=5)
        cv.create_oval(tx - 5, ty - 5, tx + 5, ty + 5, fill='#fde047', outline='')
        if self.link.get_int('grip_state'):
            cv.create_rectangle(tx - 7, ty - 7, tx + 7, ty + 7, outline='white')

    def run(self):
        self.root.mainloop()


# =============================================================================
#  5.  MAIN
# =============================================================================
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=19997, help='remote API port (default 19997, CoppeliaSim continuous server)')
    ap.add_argument('--mock', action='store_true', help='use the built-in simulator instead of CoppeliaSim')
    ap.add_argument('--mock-speed', type=float, default=1.0, help='mock simulation speed factor')
    ap.add_argument('--use-ground-truth', action='store_true',
                    help='use item pose from the simulator instead of the camera (debug / camera calibration)')
    ap.add_argument('--no-gui', action='store_true', help='headless run (prints log)')
    ap.add_argument('--duration', type=float, default=60.0, help='headless run time [s]')
    a = ap.parse_args()

    log_q = deque()
    def log(msg):
        line = '[%s] %s' % (time.strftime('%H:%M:%S'), msg)
        log_q.append(line)
        if a.no_gui: print(line, flush=True); log_q.clear()

    link = MockLink(a.mock_speed) if a.mock else RemoteApiLink(a.host, a.port)
    ctrl = CellController(link, log, a.use_ground_truth)
    ok = link.connect()
    log('connected to %s' % link.name if ok else 'could not connect to CoppeliaSim on %s:%d - use the Connect button' % (a.host, a.port))
    if not a.mock and not ok and a.no_gui: sys.exit(1)
    ctrl.start()

    if a.no_gui:
        if a.mock: link.sim_start()
        t0 = time.time()
        while time.time() - t0 < 3.0 and not ctrl.load_scene(): time.sleep(0.1)
        link.set_float('belt_speed', 0.12)
        ctrl.set_state('IDLE')
        ctrl.cmd_start()
        time.sleep(a.duration)
        st = ctrl.stats
        print('RESULT sorted=%s failed=%d ik_errors=%d cycles=%d avg_cycle=%.1fs' % (
            {CLASS_NAMES[k]: v for k, v in st['sorted'].items()}, st['failed'], st['ik_errors'], st['cycles'],
            (sum(st['cycle_times']) / len(st['cycle_times'])) if st['cycle_times'] else 0))
        ctrl.quit = True
        link.close()
        return

    gui = Dashboard(link, ctrl, log_q)
    gui.run()


if __name__ == '__main__':
    main()
