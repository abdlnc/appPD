#!/usr/bin/env python3
"""
Tests for the field-boundary ENFORCEMENT and COVER driving -- the parts of
motor_node and control_node the boundary work added.

    source ~/miniforge3/etc/profile.d/conda.sh && conda activate ros2_humble
    source ~/ros2_ws/install/setup.bash
    python3 ~/fence_enforcement_test.py

No ROS graph, no robot: the real methods are called on node objects built
without their constructors (so nothing touches GPIO or the network).

motor_node is the one place every drive command passes through, so its
rules are tested directly and exhaustively: a gap here is a robot that can
drive off the field.
"""
import sys
import types
import time
import importlib.util

ok = True


def chk(label, cond, extra=""):
    global ok
    ok &= bool(cond)
    print(f"{'PASS' if cond else 'FAIL'}  {label:62s} {extra}")


# ---- load the two nodes as part of their package, with ROS stubbed ----
for m in ('rclpy', 'rclpy.node', 'rclpy.qos', 'sensor_msgs', 'sensor_msgs.msg',
          'std_msgs', 'std_msgs.msg', 'geometry_msgs', 'geometry_msgs.msg',
          'RPi', 'RPi.GPIO'):
    sys.modules.setdefault(m, types.ModuleType(m))
sys.modules['rclpy.node'].Node = object
sys.modules['rclpy.qos'].QoSProfile = lambda **k: None
sys.modules['rclpy.qos'].DurabilityPolicy = types.SimpleNamespace(
    TRANSIENT_LOCAL=1)


class S:
    def __init__(self, data=""):
        self.data = data


sys.modules['std_msgs.msg'].String = S
sys.modules['sensor_msgs.msg'].LaserScan = object
sys.modules['sensor_msgs.msg'].CompressedImage = object
sys.modules['geometry_msgs.msg'].Twist = lambda *a, **k: types.SimpleNamespace(
    linear=types.SimpleNamespace(x=0.0), angular=types.SimpleNamespace(z=0.0))
sys.modules['RPi'].GPIO = sys.modules['RPi.GPIO']
PKG = '/home/admin/ros2_ws/src/agv_control/agv_control'
pkg = types.ModuleType('agv_control')
pkg.__path__ = [PKG]
sys.modules['agv_control'] = pkg


def load(name):
    spec = importlib.util.spec_from_file_location(f'agv_control.{name}',
                                                  f'{PKG}/{name}.py')
    mod = importlib.util.module_from_spec(spec)
    sys.modules[f'agv_control.{name}'] = mod
    spec.loader.exec_module(mod)
    return mod


mn = load('motor_node')
cn = load('control_node')
quiet = types.SimpleNamespace(info=lambda m: None, warn=lambda m: None,
                              error=lambda m: None)

# =========================================================== motor_node
print("-- motor_node: what the fence lets through")


def motor(mode="AUTO", boundary_file=True):
    o = object.__new__(mn.MotorNode)
    o.mode = mode
    o.fence = "OFF"
    o.fence_time = 0.0
    o.fence_allow = "R"
    o.fence_depth = None
    o.fence_band_min = None
    o._fence_logged = None
    o._bf_exists = boundary_file
    o._bf_checked = time.time() + 1e9          # never re-check the disk
    o.get_logger = lambda: quiet
    return o


def feed(o, text):
    mn.MotorNode.fence_cb(o, S(text))


def lim(o, x):
    return mn.MotorNode.fence_limit(o, x)[0]


o = motor()
feed(o, "OFF")
chk("no boundary: forward and reverse pass", lim(o, 0.8) == 0.8
    and lim(o, -0.5) == -0.5)
feed(o, "IN|12.0|L")
chk("inside the stop line: unrestricted", lim(o, 1.0) == 1.0)
feed(o, "LINE|L|0.4|R")
chk("at the line, facing out: FORWARD blocked", lim(o, 0.8) == 0.0)
chk("  ... reverse allowed (the way back in)", lim(o, -0.4) == -0.4)
feed(o, "LINE|R|0.4|F")
chk("at the line, facing IN: reverse blocked", lim(o, -0.4) == 0.0)
chk("  ... forward allowed (the way back in)", lim(o, 0.5) == 0.5)
o = motor()            # a fresh crossing: reusing o would carry over its
                       # 0.4 m reference, and 3.6 m would (rightly) trip
                       # NEVER DEEPER instead of testing the default
feed(o, "OUT|L|3.6")
chk("outside, direction not reported: forward blocked (default R)",
    lim(o, 0.5) == 0.0 and lim(o, -0.4) == -0.4)
feed(o, "NOFIX")
chk("boundary active, no fix: EVERYTHING blocked",
    lim(o, 0.5) == 0.0 and lim(o, -0.4) == 0.0)

o = motor()
feed(o, "IN|5.0|L")
o.fence_time = time.time() - 5.0                  # fence_node went quiet
chk("fence node silent WITH a boundary saved: everything blocked",
    lim(o, 0.5) == 0.0 and lim(o, -0.5) == 0.0)
o = motor(boundary_file=False)
o.fence_time = time.time() - 5.0
chk("fence node silent, NO boundary saved: unrestricted", lim(o, 0.5) == 0.5)

for m in ("MANUAL", "STOP"):
    o = motor(mode=m)
    feed(o, "NOFIX")
    chk(f"{m}: not fenced (a person is driving / STOP stops anyway)",
        lim(o, 0.5) == 0.5)
for m in ("AUTO", "NAV", "COVER"):
    o = motor(mode=m)
    feed(o, "LINE|L|0.5|R")
    chk(f"{m}: fenced", lim(o, 0.5) == 0.0)

print("\n-- motor_node: NEVER DEEPER")
o = motor()
feed(o, "LINE|L|0.3|R")                  # crossed at 0.3 m past the line
feed(o, "LINE|L|1.5|R")                  # 1.2 m deeper: still allowed
chk("1.2 m deeper than where it crossed: the way back is allowed",
    lim(o, -0.4) == -0.4)
feed(o, f"LINE|L|{0.3 + mn.FENCE_DEPTH_WORSEN_M + 0.1:.1f}|R")
chk("more than the limit deeper: EVERY direction blocked",
    lim(o, -0.4) == 0.0 and lim(o, 0.5) == 0.0, f"band_min {o.fence_band_min}")
feed(o, "IN|0.5|L")                       # noisy IN, right by the line
feed(o, "LINE|L|2.5|R")
chk("a noisy IN near the line does NOT reset the reference",
    o.fence_band_min == 0.3 and lim(o, -0.4) == 0.0,
    f"band_min {o.fence_band_min}")
feed(o, f"IN|{mn.FENCE_BAND_RESET_M + 0.5:.1f}|L")   # clearly back inside
chk("clearly back inside: the reference is forgotten",
    o.fence_band_min is None)

# =========================================================== control_node
print("\n-- control_node: COVER follows the route, avoidance still first")


def ctrl(mode="COVER"):
    o = object.__new__(cn.AGVControl)
    o.mode = mode
    o.is_avoiding = False
    o.blocked_hold = False
    o.dead_end = False
    o._blocked = False
    o._was_blocked = False
    o._clear_scans = 0
    o._last_log = 1e9
    o._cam_time = 0.0
    o._cam_conf = 0.0
    o._cam_count = 0
    o._was_cautious = False
    o._scan_blinded = False
    o._blind_log = 0.0
    o._gap_sign = 0.0
    o._gap_logged = 1e9
    o._last_turn = 0.0
    o._last_turn_end = 0.0
    o._turn_repeats = 0
    o.fence = "OFF"
    o.fence_time = time.time()
    o.fence_turn = "?"
    o.fence_margin = 0.0
    o.fence_depth = None
    o.fence_band_min = None
    o.fence_allow = "R"
    o.fence_blocked = False
    o._nofix_said = False
    o.route_steer = 0.0
    o.route_cap = 0.0
    o.route_hold = True
    o.route_time = 0.0
    o.zones = {k: 9999.0 for k in
               ('front', 'front_r', 'right', 'back', 'left', 'front_l')}
    o.prof = dict(cn.ENV_PROFILES["OUTDOOR"])
    o.sent = []
    o.actions = []
    o.set_cmd = lambda lx, az: o.sent.append((round(lx, 2), round(az, 2)))
    o.obstacle_pub = types.SimpleNamespace(publish=lambda m: None)
    o.publish_action = lambda t: o.actions.append(t)
    o.get_logger = lambda: quiet
    o.started = []
    cn.threading = types.SimpleNamespace(
        Thread=lambda target=None, args=(), daemon=None:
            types.SimpleNamespace(start=lambda: o.started.append(target.__name__)))
    return o


def scan(o, ranges=None):
    n = 720
    msg = types.SimpleNamespace(angle_min=0.0,
                                angle_increment=6.283185307 / n,
                                ranges=ranges or [float('inf')] * n,
                                intensities=[6] * n)
    cn.AGVControl.scan_cb(o, msg)


o = ctrl()
scan(o)
chk("COVER with no route yet: holds still", o.sent[-1] == (0.0, 0.0))
o = ctrl()
cn.AGVControl.route_cb(o, S("0.300,0.80"))
scan(o)
chk("COVER, route clear: follows the route's steer and speed cap",
    o.sent[-1] == (0.8, 0.3), f"{o.sent[-1]}")
o = ctrl()
cn.AGVControl.route_cb(o, S("0.300,0.80"))
o.route_time = time.time() - 5.0
scan(o)
chk("COVER, route gone stale: holds still (never 'carry on')",
    o.sent[-1] == (0.0, 0.0))
o = ctrl()
cn.AGVControl.route_cb(o, S("HOLD|no GPS fix"))
scan(o)
chk("COVER, route says HOLD: holds still", o.sent[-1] == (0.0, 0.0))
o = ctrl()
cn.AGVControl.route_cb(o, S("0.000,1.00"))
r = [float('inf')] * 720
for k in range(int(720 * 170 / 360), int(720 * 190 / 360)):
    r[k] = 0.5                      # a wall straight ahead (lidar mounted 180)
scan(o, r)
chk("COVER, obstacle dead ahead: avoidance wins over the route",
    o.started == ['avoid'] or (o.sent and o.sent[-1][1] != 0.0),
    f"started {o.started} last {o.sent[-1] if o.sent else None}")

print("\n-- control_node: the boundary")
for mode in ("AUTO", "COVER"):
    o = ctrl(mode)
    cn.AGVControl.route_cb(o, S("0.000,1.00"))
    cn.AGVControl.fence_cb(o, S("LINE|R|0.4|R"))
    scan(o)
    chk(f"{mode}: at the stop line, the boundary recovery starts",
        o.started == ['fence_recover'], f"{o.started}")
o = ctrl("AUTO")
cn.AGVControl.fence_cb(o, S("NOFIX"))
scan(o)
chk("NOFIX: holds still and says why", o.sent[-1] == (0.0, 0.0)
    and "NO GPS FIX" in o.actions)
o = ctrl("AUTO")
cn.AGVControl.fence_cb(o, S("IN|8.0|L"))
scan(o)
chk("inside the stop line: AUTO cruises as before",
    o.sent[-1] == (cn.ENV_PROFILES['OUTDOOR']['fwd'], 0.0), f"{o.sent[-1]}")
o = ctrl("AUTO")
o.fence_blocked = True
o.zones = {k: 1.0 for k in o.zones}
pub = []
o.obstacle_pub = types.SimpleNamespace(publish=lambda m: pub.append(m.data))
cn.AGVControl.publish_obstacle(o)
chk("stop line ahead + obstacle behind is reported as NO PATH (|BLOCKED)",
    pub and pub[-1].split("|")[3] == "BLOCKED"
    and pub[-1].split("|")[4] == "FENCE", f"{pub[-1] if pub else None}")
chk("back-off exceeds every heading baseline (else recovery deadlocks)",
    cn.FENCE_BACKOFF_M >= 5.0 + 1.0)

print("\n-- control_node: the DANGER level (regression fixed 2026-10-19)")
o = ctrl("AUTO")
pub = []
o.obstacle_pub = types.SimpleNamespace(publish=lambda m: pub.append(m.data))
o.zones = {k: 9999.0 for k in o.zones}
o.zones['front'] = cn.OVERHANG_FWD + o.prof['front'] - 0.05
cn.AGVControl.publish_obstacle(o)
chk("an obstacle just inside the trigger distance reads DANGER",
    pub and pub[-1].startswith("DANGER"), f"{pub[-1] if pub else None}")

print("\nALL PASS" if ok else "\nFAILURES")
sys.exit(0 if ok else 1)
