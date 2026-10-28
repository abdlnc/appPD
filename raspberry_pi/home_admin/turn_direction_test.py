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
# A simulated live lidar: while LIDAR['on'], every fake sleep also marks a
# fresh scan as arrived, the way the real one does at 10Hz. Tests can swap
# what the robot sees mid-manoeuvre through LIDAR['next'], or kill the
# lidar by switching it off.
LIDAR = {'robot': None, 'on': True, 'next': None, 'after': 0.0}
EVENTS = []          # (time, fn(robot)) fired once the fake clock passes time
def _sleep(dt):
    CLOCK[0] += dt
    o = LIDAR['robot']
    for ev in [e for e in EVENTS if CLOCK[0] >= e[0]]:
        EVENTS.remove(ev)
        ev[1](o)
    if o is None or not LIDAR['on']:
        return
    if LIDAR['next'] is not None and CLOCK[0] >= LIDAR['after']:
        o._scan_ranges = LIDAR['next']
        LIDAR['next'] = None
    o._scan_time = CLOCK[0]
cn.time = types.SimpleNamespace(time=lambda: CLOCK[0], sleep=_sleep)

STEP = 0.5          # degrees per ray, as on the C1
N = 720
def scan_with(points):
    """(bearing_deg, range_m) pairs -> a /scan ranges list. Bearing is in
    the robot frame, 0 = straight ahead, CCW positive, as control_node sees
    it after ANGLE_OFFSET_DEG."""
    r = [math.inf] * N
    for b, d in points:
        k = int(round(((b - cn.ANGLE_OFFSET_DEG) % 360.0) / STEP)) % N
        r[k] = min(r[k], d)
    return r

def rear_wall(d):
    """A wall square behind the robot, d metres from the lidar."""
    pts = []
    b = 120.0
    while b <= 240.0:
        c = -math.cos(math.radians(b))
        if c > 1e-6:
            pts.append((b, d / c))
        b += STEP
    return scan_with(pts)

ZONES = ('front','front_r','right','back','left','front_l')
def Z(**k): return {**{z: 9999.0 for z in ZONES}, **k}

def robot(env="OUTDOOR"):
    """A real AGVControl with its methods, minus __init__ (no ROS graph)."""
    o = object.__new__(cn.AGVControl)
    o.mode = "AUTO"; o.is_avoiding = False; o.dead_end = False
    o.zones = Z()
    o.prof = dict(cn.ENV_PROFILES[env])
    o._last_turn = 0.0; o._last_turn_end = 0.0; o._turn_repeats = 0
    o._attempts = 0; o._turn_why = ""
    o._last_action = None
    o.cmds = []
    o.published = []        # what _stop_now() sent immediately
    o.cmd = (0.0, 0.0)
    o.publish_action = lambda t: None
    def set_cmd(lx, az):
        o.cmd = (round(lx, 2), round(az, 2))
        o.cmds.append(o.cmd)
    o.set_cmd = set_cmd
    o.pub = types.SimpleNamespace(publish=lambda m: o.published.append(m))
    o.get_logger = lambda: types.SimpleNamespace(warn=lambda m: None,
                                                 info=lambda m: None)
    return o

def run_avoid(o, z, scan=None):
    # The reverse leg now reads the live scan, not z['back']. Unless a test
    # supplies its own, build one with a wall where z says the rear is.
    o._scan_ranges = scan if scan is not None else rear_wall(z['back'])
    o._scan_geom = (0.0, math.radians(STEP))
    o._scan_time = CLOCK[0]
    LIDAR['robot'] = o
    o.zones = dict(z)            # the live zones, as the trigger saw them
    cn.AGVControl.avoid(o, z)
    # The direction is the FORWARD turn's. From the 2nd attempt the reverse
    # is steered the OTHER way on purpose (three-point turn, see DEAD ENDS),
    # so the first steering command is no longer the turn direction.
    turns = [az for lx, az in o.cmds if lx > 0 and az != 0.0]
    if not turns:                # forward leg refused: its settle command
        turns = [az for lx, az in o.cmds if lx == 0 and az != 0.0][-1:]
    o.last_cmds = list(o.cmds)   # kept: the reverse tests read these
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
# A wall 0.10m behind the bumper (0.50m from the centre-mounted lidar).
# This used to say back=0.1, which is inside the robot itself -- and under
# MIN_VALID_DIST, so the live scan would never even see it.
run_avoid(o, Z(front=0.4, front_l=0.5, front_r=1.6, back=0.5))
blocked_rear = CLOCK[0] - t0
o2 = robot()
t0 = CLOCK[0]
run_avoid(o2, Z(front=0.4, front_l=0.5, front_r=1.6, back=2.0))
clear_rear = CLOCK[0] - t0
# with the rear blocked: stop + settle + turn, and no reverse leg
expect = cn.STOP_BEFORE_REVERSE + cn.SETTLE_TIME + cn.TURN_TIME
# The turn is now WATCHED (polled every TURN_POLL, like the reverse), so it
# can run up to one poll past TURN_TIME.
chk("rear blocked: reverse leg skipped",
    -0.01 < blocked_rear - expect <= cn.TURN_POLL + 0.01,
    f"{blocked_rear:.2f}s vs {expect:.2f}s")
chk("rear clear: reverse leg included",
    clear_rear > blocked_rear + cn.REVERSE_TIME - 0.01,
    f"{clear_rear:.2f}s vs {blocked_rear:.2f}s")

print("\n-- symmetric case is stable rather than arbitrary")
o = robot()
d1 = run_avoid(o, Z(front=0.4, back=2.0))      # open ground, nothing either side
CLOCK[0] += cn.TURN_COMMIT_TIME + 1.0
d2 = run_avoid(o, Z(front=0.4, back=2.0))
chk("symmetric obstacle: same direction each time", d1 == d2, name(d1))

print("\n-- reversing: the robot must not back into anything")
blocked = Z(front=0.4, front_l=0.5, front_r=1.6, left=0.6, right=1.8, back=2.0)

def reverse_speeds(o):
    # Read what the LAST manoeuvre commanded. (An earlier draft read o.cmds,
    # which run_avoid clears -- so every "did not reverse" check passed no
    # matter what. Each such check below now has a control case beside it
    # proving it can fail.)
    return sorted({lx for lx, az in o.last_cmds if lx < 0})

o = robot()
run_avoid(o, blocked)
chk("a normal reverse runs at REV_SPEED",
    reverse_speeds(o) == [-cn.REV_SPEED], f"{reverse_speeds(o)}")

o = robot()
for _ in range(cn.TURN_ESCALATE_AFTER + 1):
    o.cmds.clear()
    run_avoid(o, blocked)
chk("the escalated (max) reverse runs slower",
    reverse_speeds(o) == [-cn.REV_SPEED_ESCALATED],
    f"{reverse_speeds(o)} vs normal {-cn.REV_SPEED}")
chk("but not below the motors' stall floor", cn.REV_SPEED_ESCALATED >= 0.40)

# Someone steps behind 0.3s into the reverse, 0.15m from the bumper.
o = robot()
t0 = CLOCK[0]
LIDAR['next'] = rear_wall(cn.OVERHANG_REAR + 0.15)
LIDAR['after'] = t0 + cn.STOP_BEFORE_REVERSE + 0.3
run_avoid(o, blocked)
took = CLOCK[0] - t0
full = (cn.STOP_BEFORE_REVERSE + cn.REVERSE_TIME + cn.STOP_AFTER_REVERSE
        + cn.SETTLE_TIME + cn.TURN_TIME)
chk("someone stepping behind mid-reverse cuts it short",
    took < full - 0.5, f"{took:.2f}s vs {full:.2f}s uninterrupted")
chk("and the stop goes out at once, not on the next tick",
    (0.0, 0.0) in o.published, f"published {o.published}")

# The lidar dies mid-reverse: no fresh scans, so stop.
o = robot()
t0 = CLOCK[0]
EVENTS.append((t0 + cn.STOP_BEFORE_REVERSE + 0.2,
               lambda r: LIDAR.__setitem__('on', False)))
run_avoid(o, blocked)
LIDAR['on'] = True
took = CLOCK[0] - t0
chk("a dead lidar mid-reverse stops it",
    took < full - 0.3, f"{took:.2f}s vs {full:.2f}s")

# Emergency stop mid-reverse: leave the loop at once.
o = robot()
t0 = CLOCK[0]
EVENTS.append((t0 + cn.STOP_BEFORE_REVERSE + 0.2,
               lambda r: setattr(r, 'mode', 'STOP')))
run_avoid(o, blocked)
took = CLOCK[0] - t0
chk("an emergency stop mid-reverse ends it",
    took < cn.STOP_BEFORE_REVERSE + 0.2 + cn.STOP_AFTER_REVERSE + 0.2,
    f"{took:.2f}s")

# Something at a rear CORNER: inside the robot's width, but outside the
# 60-degree back zone the old check used. 0.05m behind the bumper, at the
# corner. The old check saw a clear back zone and would have reversed.
corner = scan_with([(math.degrees(math.atan2(0.30, -0.45)), math.hypot(0.30, 0.45))])
b = math.degrees(math.atan2(0.30, -0.45))
chk("(that point really is outside the old back zone)", not (150 <= b <= 210),
    f"bearing {b:.1f} deg")
o = robot()
run_avoid(o, Z(front=0.4, front_l=0.5, front_r=1.6, back=9999.0),
          scan=scan_with([]))
chk("(control: same scene, corner empty -> it DOES reverse)",
    reverse_speeds(o) == [-cn.REV_SPEED], f"{reverse_speeds(o)}")
o = robot()
run_avoid(o, Z(front=0.4, front_l=0.5, front_r=1.6, back=9999.0), scan=corner)
chk("an obstacle at a rear corner blocks the reverse",
    not reverse_speeds(o), f"reverse cmds {reverse_speeds(o)}")

# No scan has ever arrived: never reverse blind.
o = robot()
LIDAR['on'] = False
run_avoid(o, blocked)
o._scan_time = 0.0
o.cmds.clear()
cn.AGVControl.avoid(o, blocked)
o.last_cmds = list(o.cmds)
LIDAR['on'] = True
chk("with no fresh scan it does not reverse at all", not reverse_speeds(o),
    f"{reverse_speeds(o)}")

# A person walking IN from the side, not there at the trigger: the
# corridor watch sees them as soon as they are behind the body.
o = robot()
t0 = CLOCK[0]
LIDAR['next'] = scan_with([(170.0, cn.OVERHANG_REAR + 0.10)])  # one leg
LIDAR['after'] = t0 + cn.STOP_BEFORE_REVERSE + 0.5
run_avoid(o, blocked)
chk("a person stepping in mid-reverse is caught too",
    CLOCK[0] - t0 < full - 0.3, f"{CLOCK[0]-t0:.2f}s")

# ================================================== 2026-10-28 additions
def front_wall(d):
    """A wall square ahead of the robot, d metres from the lidar."""
    pts = []
    b = -60.0
    while b <= 60.0:
        c = math.cos(math.radians(b))
        if c > 1e-6:
            pts.append((b, d / c))
        b += STEP
    return scan_with(pts)


def both(front_d, rear_d):
    f, r = front_wall(front_d), rear_wall(rear_d)
    return [min(a, b) for a, b in zip(f, r)]


print("\n-- the forward turn is watched (indoors it used to hit the wall)")
o = robot("INDOOR")
o.cmds.clear()
stamps = []
_orig = o.set_cmd
def set_cmd_t(lx, az):
    stamps.append((CLOCK[0], lx, az))
    _orig(lx, az)
o.set_cmd = set_cmd_t
# Wall 0.40m ahead of the bumper; 0.3s into the turn it is 0.10m away.
t0 = CLOCK[0]
start_turn = t0 + cn.STOP_BEFORE_REVERSE + cn.SETTLE_TIME
LIDAR['next'] = both(cn.OVERHANG_FWD + 0.10, 2.0)
LIDAR['after'] = start_turn + 0.3
run_avoid(o, Z(front=0.8, back=2.0), scan=both(cn.OVERHANG_FWD + 0.40, 0.3))
fwd = [t for t, lx, az in stamps if lx > 0]
stop_after = [t for t, lx, az in stamps if lx == 0 and az == 0 and fwd
              and t > fwd[0]]
ran = (stop_after[0] - fwd[0]) if fwd and stop_after else None
chk("turn starts with room ahead", bool(fwd))
chk("and is cut short when the wall comes within TURN_STOP_GAP",
    ran is not None and ran < cn.TURN_TIME - 0.5, f"ran {ran}")
chk("the stop goes out at once (published, not left for the tick)",
    (0.0, 0.0) in o.published)

o = robot("INDOOR")
run_avoid(o, Z(front=0.5, back=0.45),
          scan=both(cn.OVERHANG_FWD + 0.10, cn.OVERHANG_REAR + 0.05))
chk("no room ahead (0.10m) and none behind: does not drive forward at all",
    not any(lx > 0 for lx, az in o.last_cmds), f"{o.last_cmds}")
o2 = robot("INDOOR")
run_avoid(o2, Z(front=0.5, back=0.45),
          scan=both(cn.OVERHANG_FWD + 0.40, cn.OVERHANG_REAR + 0.05))
chk("(control: with 0.40m ahead it does turn forward)",
    any(lx > 0 for lx, az in o2.last_cmds))

print("\n-- dead ends: three-point turn, then a limit")
dead = Z(front=0.6, front_l=0.9, front_r=1.6, left=0.9, right=0.9,
         back=2.0)
o = robot("INDOOR")
run_avoid(o, dead, scan=both(cn.OVERHANG_FWD + 0.40, 2.0))
rev = [(lx, az) for lx, az in o.last_cmds if lx < 0]
chk("1st attempt: straight reverse, as before",
    rev and all(az == 0.0 for _, az in rev), f"{rev}")
turn1 = [az for lx, az in o.last_cmds if lx > 0][0]
run_avoid(o, dead, scan=both(cn.OVERHANG_FWD + 0.40, 2.0))
rev = [(lx, az) for lx, az in o.last_cmds if lx < 0]
chk("2nd attempt: reverse steered the OTHER way (turns round backing up)",
    rev and all(az == -turn1 for _, az in rev), f"{rev} vs turn {turn1}")
tight = dict(dead, right=cn.OVERHANG_SIDE + 0.10)
o3 = robot("INDOOR")
run_avoid(o3, tight, scan=both(cn.OVERHANG_FWD + 0.40, 2.0))
run_avoid(o3, tight, scan=both(cn.OVERHANG_FWD + 0.40, 2.0))
rev = [(lx, az) for lx, az in o3.last_cmds if lx < 0]
chk("...but straight if a side is within KTURN_SIDE_ROOM (tail swing)",
    rev and all(az == 0.0 for _, az in rev), f"{rev}")

o = robot("INDOOR")
for i in range(cn.DEAD_END_ATTEMPTS):
    run_avoid(o, dead, scan=both(cn.OVERHANG_FWD + 0.40, 2.0))
chk(f"{cn.DEAD_END_ATTEMPTS} attempts: still trying", not o.dead_end)
o.cmds.clear()
run_avoid(o, dead, scan=both(cn.OVERHANG_FWD + 0.40, 2.0))
chk(f"attempt {cn.DEAD_END_ATTEMPTS + 1}: gives up -- DEAD END hold",
    o.dead_end and not o.is_avoiding)
chk("and does not move at all", all(lx == 0 for lx, az in o.last_cmds),
    f"{o.last_cmds}")

msgs = []
o.zones = dict(dead); o._blocked = False; o._was_blocked = False
o.fence_blocked = False
o.obstacle_pub = types.SimpleNamespace(publish=lambda m: msgs.append(m.data))
cn.AGVControl.publish_obstacle(o)
chk("reported as NO PATH, reason DEADEND (the installed app shows NO PATH)",
    msgs and msgs[-1].endswith("|BLOCKED|DEADEND"), f"{msgs}")

held = []
o.pub = types.SimpleNamespace(publish=lambda m: held.append(m))
o.blocked_hold = False; o.cmd = "drive!"
cn.AGVControl.tick(o)
chk("tick() publishes zeros while held", held and held[-1] != "drive!")

class _M:
    def __init__(self, d): self.data = d
cn.AGVControl.mode_cb(o, _M("MANUAL"))
chk("a mode change clears the hold and the attempt count",
    not o.dead_end and o._turn_repeats == 0 and o._last_turn == 0.0)

print("\n-- which side: where it can GO (2026-10-28, the open door)")


def scene(left_r, right_r, post=None, front_r=0.85):
    """Rays every STEP degrees: front_r within +/-10 deg, left_r on the left
    (10-180), right_r on the right; post = (bearing_from, bearing_to, r)."""
    pts = []
    b = -180.0
    while b < 180.0:
        r = front_r if abs(b) < 10 else (left_r if b > 0 else right_r)
        if post and post[0] <= b <= post[1]:
            r = min(r, post[2])
        pts.append((b, r))
        b += STEP
    return scan_with(pts)


def choose(o, scan, z):
    o._scan_ranges = scan
    o._scan_geom = (0.0, math.radians(STEP))
    o._scan_time = CLOCK[0]
    return cn.AGVControl.pick_turn(o, z)


# A post at the LEFT front corner (FL 0.60) -- the old rule turns RIGHT,
# away from it -- but the right is a pocket closed at 1.0m and the left is
# open beyond the post.
door = scene(3.5, 1.0, post=(40, 45, 0.60))
zd = Z(front=0.85, front_l=0.60, front_r=1.0, left=3.5, right=1.0, back=3.5)
o = robot("INDOOR")
o.set_cmd = lambda lx, az: None
opn = None
o._scan_ranges = door; o._scan_geom = (0.0, math.radians(STEP))
o._scan_time = CLOCK[0]
opn = cn.AGVControl.side_openness(o)
chk("openness: left ~3m (past the post), right ~1m (pocket)",
    opn[0] > 2.5 and opn[1] < 1.1, f"{opn[0]:.2f} / {opn[1]:.2f}")
d, rep_ = choose(o, door, zd)
chk("fresh choice: LEFT, toward where it can go (old rule: RIGHT)",
    d == +1.0, f"{name(d)} -- {o._turn_why}")

# nothing decisive either way: the corner rule decides, as before
even = scene(2.0, 2.2)
o = robot("INDOOR")
d, _ = choose(o, even, Z(front=0.85, front_l=0.5, front_r=1.0,
                         left=2.0, right=2.2))
chk("sides about equally open: the corner rule still decides (RIGHT)",
    d == -1.0, f"{name(d)}")

print("\n-- the commitment gives way only to decisive evidence")
o = robot("INDOOR")
o._last_turn = -1.0; o._last_turn_end = CLOCK[0]; o._turn_repeats = 1
d, rep_ = choose(o, door, zd)
chk("committed RIGHT, left now 2m more open: switches to LEFT",
    d == +1.0 and rep_ == 0, f"{name(d)} -- {o._turn_why}")
# 15:33:16 replayed: committed LEFT, left corner 0.69, right corner 1.43
o = robot("INDOOR")
o._last_turn = +1.0; o._last_turn_end = CLOCK[0]; o._turn_repeats = 2
d, _ = choose(o, scene(0.75, 1.5),
              Z(front=0.68, front_l=0.69, front_r=1.43, left=0.70,
                right=0.75))
chk("15:33:16 replayed (L 0.69 vs R 1.43, committed LEFT): goes RIGHT",
    d == -1.0, f"{name(d)} -- {o._turn_why}")
o = robot("INDOOR")
o._last_turn = +1.0; o._last_turn_end = CLOCK[0]; o._turn_repeats = 1
d, rep_ = choose(o, scene(1.8, 2.1),
                 Z(front=0.8, front_l=0.9, front_r=1.1, left=1.8,
                   right=2.1))
chk("small differences (0.3m open, 0.2m corner): stays committed LEFT",
    d == +1.0 and rep_ == 2, f"{name(d)} rep {rep_}")

print("\n-- the dead-end count survives switching sides")
o = robot("INDOOR")
left_open = scene(3.5, 1.0, post=(40, 45, 0.60))
right_open = scene(1.0, 3.5, post=(-45, -40, 0.60))
zl = Z(front=0.6, front_l=0.6, front_r=1.0, left=3.5, right=1.0, back=3.5)
zr = Z(front=0.6, front_l=1.0, front_r=0.6, left=1.0, right=3.5, back=3.5)
dirs = []
for i in range(cn.DEAD_END_ATTEMPTS + 1):
    sc, zz = (left_open, zl) if i % 2 == 0 else (right_open, zr)
    run_avoid(o, zz, scan=[min(a, b) for a, b in
                           zip(sc, rear_wall(3.0))])
    dirs.append(o._last_turn)
chk("(test data: it really did switch sides)", len(set(dirs[:-1])) == 2,
    f"{[name(x) for x in dirs]}")
chk(f"still stops at DEAD END after {cn.DEAD_END_ATTEMPTS} attempts",
    o.dead_end)

print("\n-- the turn watch covers the side it turns toward")
o = robot("INDOOR")
o._scan_ranges = scan_with([(45.0, 0.55)])     # wall at the front-LEFT corner
o._scan_geom = (0.0, math.radians(STEP)); o._scan_time = CLOCK[0]
straight = cn.AGVControl.front_clearance(o, cn.TURN_WATCH_MARGIN)
turning_left = cn.AGVControl.front_clearance(
    o, cn.TURN_WATCH_MARGIN, extra_left=cn.TURN_SWING_EXTRA)
turning_right = cn.AGVControl.front_clearance(
    o, cn.TURN_WATCH_MARGIN, extra_right=cn.TURN_SWING_EXTRA)
chk("straight strip misses a wall at the front-left corner (the 15:33 hit)",
    straight == math.inf, f"{straight}")
chk("turning LEFT: the wall counts -- the corner would swing into it",
    turning_left < cn.TURN_STOP_GAP, f"{turning_left:.2f}")
chk("turning RIGHT: it does not (the corner swings away)",
    turning_right == math.inf, f"{turning_right}")

print("\nALL PASS" if ok else "\nFAILURES")
sys.exit(0 if ok else 1)
