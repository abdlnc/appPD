#!/usr/bin/env python3
"""
Unit tests for the AUTO turn-direction decision in control_node.

    python3 ~/turn_direction_test.py

Needs no ROS graph, no Lidar and no robot: control_node is loaded as a
module, avoid() is driven directly, and time.sleep is replaced by a fake
clock so a 7-second maneuver costs nothing to test.

WHAT THIS GUARDS. The robot used to decide each avoidance from scratch,
which let two consecutive attempts contradict each other -- it would swing
left, then swing right and undo it, and sit there oscillating in front of
the same obstacle. The "4 back-to-back attempts all go the same way" test
is that bug; the "old code did contradict itself" test keeps the original
behaviour on record so the fix cannot be quietly reverted without a
failure. Anything that makes those two fail has brought the oscillation
back.
"""

import sys, types, importlib.util, math

for m in ('rclpy','rclpy.node','sensor_msgs','sensor_msgs.msg','std_msgs',
          'std_msgs.msg','geometry_msgs','geometry_msgs.msg'):
    sys.modules.setdefault(m, types.ModuleType(m))
sys.modules['rclpy.node'].Node = object
class S:
    def __init__(self): self.data = ""
sys.modules['std_msgs.msg'].String = S
sys.modules['sensor_msgs.msg'].LaserScan = object
sys.modules['geometry_msgs.msg'].Twist = lambda *a, **k: types.SimpleNamespace(
    linear=types.SimpleNamespace(x=0.0), angular=types.SimpleNamespace(z=0.0))
# control_node does "from . import scan_filter", so it has to be loaded as
# part of a package rather than as a loose file.
PKG = '/home/admin/ros2_ws/src/agv_control/agv_control'
pkg = types.ModuleType('agv_control'); pkg.__path__ = [PKG]
sys.modules['agv_control'] = pkg
spec = importlib.util.spec_from_file_location(
    'agv_control.control_node', PKG + '/control_node.py')
cn = importlib.util.module_from_spec(spec)
sys.modules['agv_control.control_node'] = cn
spec.loader.exec_module(cn)

CLOCK = [1000.0]
cn.time = types.SimpleNamespace(time=lambda: CLOCK[0],
                                sleep=lambda s: CLOCK.__setitem__(0, CLOCK[0]+s))

ZONES = ('front','front_r','right','back','left','front_l')
def Z(**k): return {**{z: 9999.0 for z in ZONES}, **k}

def robot(env="OUTDOOR"):
    """A real AGVControl with its methods, minus __init__ (no ROS graph)."""
    o = object.__new__(cn.AGVControl)
    o.mode = "AUTO"; o.is_avoiding = False
    o.prof = dict(cn.ENV_PROFILES[env])
    o._last_turn = 0.0; o._last_turn_end = 0.0; o._turn_repeats = 0
    o._last_action = None
    o.cmds = []
    o.publish_action = lambda t: None
    o.set_cmd = lambda lx, az: o.cmds.append((round(lx, 2), round(az, 2)))
    o.get_logger = lambda: types.SimpleNamespace(warn=lambda m: None,
                                                 info=lambda m: None)
    return o

def run_avoid(o, z):
    cn.AGVControl.avoid(o, z)
    turns = [az for lx, az in o.cmds if az != 0.0]
    o.cmds.clear()
    return turns[0] if turns else 0.0

def name(s): return "LEFT" if s > 0 else ("RIGHT" if s < 0 else "none")

ok = True
def chk(label, cond, extra=""):
    global ok; ok &= bool(cond)
    print(f"{'PASS' if cond else 'FAIL'}  {label:58s} {extra}")

print(f"TURN_COMMIT_TIME={cn.TURN_COMMIT_TIME}s  "
      f"TURN_SIDE_MARGIN={cn.TURN_SIDE_MARGIN}m  "
      f"ESCALATE_AFTER={cn.TURN_ESCALATE_AFTER}\n")

print("-- the reported fault: consecutive attempts must not contradict")
o = robot()
# obstacle ahead, more room on the right -> should go right, and KEEP going
# right even though reversing changes what the side zones see
seq = [Z(front=0.4, front_l=0.5, front_r=1.6, left=0.6, right=1.8, back=2.0),
       Z(front=0.4, front_l=0.6, front_r=1.2, left=1.9, right=0.7, back=2.0),
       Z(front=0.4, front_l=0.5, front_r=1.5, left=0.8, right=1.1, back=2.0),
       Z(front=0.4, front_l=0.7, front_r=1.4, left=1.5, right=0.9, back=2.0)]
dirs = [run_avoid(o, z) for z in seq]
print("   directions chosen:", [name(d) for d in dirs])
chk("4 back-to-back attempts all go the same way",
    len(set(dirs)) == 1, f"{name(dirs[0])}")
chk("and it is the side with more room (RIGHT)", dirs[0] < 0)

print("\n-- the OLD code on the same sequence, for comparison")
olds = []
for z in seq:
    if z['front_r'] < 0.28: olds.append(+1.0)
    elif z['front_l'] < 0.28: olds.append(-1.0)
    else: olds.append(+1.0 if z['right'] < z['left'] else -1.0)
print("   directions chosen:", [name(d) for d in olds])
chk("old code did contradict itself (this was the bug)",
    len(set(olds)) > 1, f"{len(set(olds))} different directions")

print("\n-- commitment expires once it has driven clear")
o = robot()
run_avoid(o, Z(front=0.4, front_l=0.5, front_r=1.6, left=0.6, right=1.8, back=2.0))
first = o._last_turn
CLOCK[0] += cn.TURN_COMMIT_TIME + 1.0          # drove clear for a while
second = run_avoid(o, Z(front=0.4, front_l=1.6, front_r=0.5,
                        left=1.8, right=0.6, back=2.0))
chk("a new obstacle later gets a fresh decision", second != first,
    f"{name(first)} then {name(second)}")

print("\n-- direction comes from the obstacle's side, not the broadside")
o = robot()
d = run_avoid(o, Z(front=0.4, front_l=0.45, front_r=2.0, back=2.0))
chk("obstacle front-LEFT  -> turns RIGHT", d < 0, name(d))
o = robot()
d = run_avoid(o, Z(front=0.4, front_l=2.0, front_r=0.45, back=2.0))
chk("obstacle front-RIGHT -> turns LEFT", d > 0, name(d))

print("\n-- noise must not flip the choice")
o = robot()
d1 = run_avoid(o, Z(front=0.4, front_l=1.00, front_r=1.02, left=1.00,
                    right=1.03, back=2.0))
CLOCK[0] += cn.TURN_COMMIT_TIME + 1.0
d2 = run_avoid(o, Z(front=0.4, front_l=1.02, front_r=1.00, left=1.03,
                    right=1.00, back=2.0))
chk("readings 2cm apart give the same direction twice", d1 == d2,
    f"{name(d1)} / {name(d2)}")

print("\n-- escalation when the same escape keeps failing")
o = robot()
blocked = Z(front=0.4, front_l=0.5, front_r=1.6, left=0.6, right=1.8, back=2.0)
spans = []
for i in range(5):
    t0 = CLOCK[0]
    run_avoid(o, blocked)
    spans.append(round(CLOCK[0] - t0, 2))
print("   maneuver durations:", spans, "s")
chk("later attempts take longer than the first", spans[-1] > spans[0],
    f"{spans[0]}s -> {spans[-1]}s")
chk("but never past the cap",
    spans[-1] <= spans[0] * cn.TURN_ESCALATE_MAX + 0.01)
chk("escalation never flips direction", o._last_turn == -1.0, name(o._last_turn))

print("\n-- blocked rear still skips the reverse")
o = robot()
t0 = CLOCK[0]
run_avoid(o, Z(front=0.4, front_l=0.5, front_r=1.6, back=0.1))
blocked_rear = CLOCK[0] - t0
o2 = robot()
t0 = CLOCK[0]
run_avoid(o2, Z(front=0.4, front_l=0.5, front_r=1.6, back=2.0))
clear_rear = CLOCK[0] - t0
# with the rear blocked: stop + settle + turn, and no reverse leg
expect = cn.STOP_BEFORE_REVERSE + cn.SETTLE_TIME + cn.TURN_TIME
chk("rear blocked: reverse leg skipped",
    abs(blocked_rear - expect) < 0.01, f"{blocked_rear:.2f}s vs {expect:.2f}s")
chk("rear clear: reverse leg included",
    clear_rear > blocked_rear + cn.REVERSE_TIME - 0.01,
    f"{clear_rear:.2f}s vs {blocked_rear:.2f}s")

print("\n-- symmetric case is stable rather than arbitrary")
o = robot()
d1 = run_avoid(o, Z(front=0.4, back=2.0))      # open ground, nothing either side
CLOCK[0] += cn.TURN_COMMIT_TIME + 1.0
d2 = run_avoid(o, Z(front=0.4, back=2.0))
chk("symmetric obstacle: same direction each time", d1 == d2, name(d1))

print("\nALL PASS" if ok else "\nFAILURES")
sys.exit(0 if ok else 1)
