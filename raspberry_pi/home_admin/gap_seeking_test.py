#!/usr/bin/env python3
"""
Tests for gap seeking and the OUTDOOR/INDOOR profiles in control_node.

    python3 ~/gap_seeking_test.py

No ROS graph, no Lidar, no robot. Scans are built by ray-casting against
wall segments given in metres, so each case reads as a floor plan rather
than as a list of ray indices: the robot sits at the origin facing +x with
+y to its LEFT.

WHAT THIS GUARDS. The robot used to treat any near return in its front arc
as a wall -- stop, reverse, turn, retry -- which had it shuffling in front
of doorways and row gaps it could have driven straight through. pick_gap()
looks for a heading along which a corridor as wide as the robot is clear,
and steers into it; the stop/reverse/turn maneuver is now only the
fallback for when no such heading exists.

ROBOT_WIDTH (0.60m) is a physical measurement of this machine, not a
tuning knob. If the chassis, wheels or guards change, re-measure it and
expect the corridor numbers below to move with it.

The corridor cases are the ones that matter: anything that makes a gap
WIDER than the robot read as impassable has brought the shuffling back,
and anything that makes a gap NARROWER than the robot read as passable
will wedge it.
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
PKG = '/home/admin/ros2_ws/src/agv_control/agv_control'
pkg = types.ModuleType('agv_control'); pkg.__path__ = [PKG]
sys.modules['agv_control'] = pkg
spec = importlib.util.spec_from_file_location('agv_control.control_node',
                                              PKG + '/control_node.py')
cn = importlib.util.module_from_spec(spec)
sys.modules['agv_control.control_node'] = cn
spec.loader.exec_module(cn)

N = 720
STEP = 360.0 / N

def scan_from_walls(segments):
    """segments: list of ((x1,y1),(x2,y2)) in metres, robot at origin facing
    +x, +y to the LEFT. Returns a ranges list, inf where nothing is hit."""
    ranges = [math.inf] * N
    for i in range(N):
        brg = i * STEP                      # ray bearing in the robot frame
        brg = brg if brg <= 180 else brg - 360.0
        a = math.radians(brg)
        dx, dy = math.cos(a), math.sin(a)
        best = math.inf
        for (x1, y1), (x2, y2) in segments:
            # ray (0,0)+t(dx,dy) against segment p1+u(p2-p1)
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            t = ((x1 * ey) - (y1 * ex)) / den
            u = ((x1 * dy) - (y1 * dx)) / den
            if t > 0 and 0.0 <= u <= 1.0:
                best = min(best, t)
        if best < math.inf:
            ranges[i] = best
    # back into /scan index order: scan_cb maps index -> deg via
    # (degrees(angle_min + k*inc) + 180) % 360, so with angle_min=0 and
    # inc=STEP, index k carries bearing (k*STEP + 180) % 360.
    out = [math.inf] * N
    for k in range(N):
        deg = (k * STEP + cn.ANGLE_OFFSET_DEG) % 360.0
        out[k] = ranges[int(round(deg / STEP)) % N]
    return out

def gap(ranges, last_sign=0.0):
    o = object.__new__(cn.AGVControl)
    o._gap_sign = last_sign
    steer, found = cn.AGVControl.pick_gap(o, ranges, 0.0, math.radians(STEP))
    return steer, found, o._gap_sign

def deg_of(steer): return steer * cn.GAP_STEER_FULL_DEG

ok = True
def chk(label, cond, extra=""):
    global ok; ok &= bool(cond)
    print(f"{'PASS' if cond else 'FAIL'}  {label:56s} {extra}")

need = cn.ROBOT_WIDTH + 2*cn.GAP_SIDE_MARGIN
print(f"ROBOT_WIDTH={cn.ROBOT_WIDTH}m  margin={cn.GAP_SIDE_MARGIN}m  "
      f"-> needs {need:.2f}m of clear space")
print(f"GAP_LOOKAHEAD={cn.GAP_LOOKAHEAD}m  ARC=+/-{cn.GAP_ARC_DEG}deg\n")

print("-- corridors: does it fit, and does it know?")
# The exact boundary (a 70cm gap for a 70cm requirement, i.e. zero
# clearance) is deliberately not asserted either way: which side of it
# floating point lands on is not a behaviour worth pinning, and a robot
# aimed at a gap its own width is not something to encourage.
for width in (1.20, 0.80, 0.72, 0.68, 0.60, 0.40):
    h = width / 2.0
    walls = [((-1,  h), (3,  h)), ((-1, -h), (3, -h))]
    steer, found, _ = gap(scan_from_walls(walls))
    fits = width > need
    chk(f"{width*100:.0f}cm corridor: {'fits' if fits else 'too narrow'}",
        found == fits, f"found={found} steer={deg_of(steer):+.0f}deg")

print("\n-- a doorway straight ahead")
# wall across at x=1.2 with a 0.9m opening centred on the robot
walls = [((1.2, 0.45), (1.2, 4.0)), ((1.2, -0.45), (1.2, -4.0))]
steer, found, _ = gap(scan_from_walls(walls))
chk("90cm doorway dead ahead: drives straight through",
    found and abs(deg_of(steer)) < 1e-9, f"steer={deg_of(steer):+.0f}deg")

# same doorway, offset 0.5m to the LEFT
walls = [((1.2, 0.95), (1.2, 4.0)), ((1.2, 0.05), (1.2, -4.0))]
steer, found, _ = gap(scan_from_walls(walls))
chk("90cm doorway offset left: steers LEFT into it",
    found and deg_of(steer) > 0, f"steer={deg_of(steer):+.0f}deg")

# offset to the RIGHT
walls = [((1.2, -0.05), (1.2, 4.0)), ((1.2, -0.95), (1.2, -4.0))]
steer, found, _ = gap(scan_from_walls(walls))
chk("90cm doorway offset right: steers RIGHT into it",
    found and deg_of(steer) < 0, f"steer={deg_of(steer):+.0f}deg")

print("\n-- a gap too small to fit must NOT be aimed at")
# The robot is 60cm; a 50cm hole is not an option. What it must not do is
# aim at it. Veering wide instead is correct and is what it does: within
# GAP_LOOKAHEAD a steep heading really is clear, and the next scan
# re-decides from there.
walls = [((1.2, 0.25), (1.2, 4.0)), ((1.2, -0.25), (1.2, -4.0))]   # 50cm
steer, found, _ = gap(scan_from_walls(walls))
chk("50cm opening: does not aim through it",
    (not found) or abs(deg_of(steer)) > 15.0,
    f"found={found} steer={deg_of(steer):+.0f}deg")
# And a dead end with no way round inside the lookahead gives up properly.
walls = [((1.0, 3.0), (1.0, 0.25)), ((1.0, -0.25), (1.0, -3.0)),
         ((0.0, 0.35), (1.0, 0.35)), ((0.0, -0.35), (1.0, -0.35))]
steer, found, _ = gap(scan_from_walls(walls))
chk("50cm hole at the end of a tight dead end: no gap at all", not found)

print("\n-- obstacle dead ahead with room to one side")
# a 0.3m-wide post centred ahead at 1.0m, open ground elsewhere
walls = [((1.0, 0.15), (1.0, -0.15))]
steer, found, _ = gap(scan_from_walls(walls))
chk("post ahead, open ground: steers around it",
    found and abs(deg_of(steer)) > 0, f"steer={deg_of(steer):+.0f}deg")

print("\n-- a dead end must fall through to the maneuver")
walls = [((1.0, 3.0), (1.0, -3.0))]       # solid wall across the path
steer, found, _ = gap(scan_from_walls(walls))
chk("solid wall at 1.0m: no gap found", not found)

print("\n-- open ground is unchanged: straight ahead, full speed")
steer, found, _ = gap([math.inf]*N)
chk("nothing in range: straight ahead", found and steer == 0.0)

print("\n-- beyond the lookahead is not a concern")
walls = [((cn.GAP_LOOKAHEAD + 0.5, 3.0), (cn.GAP_LOOKAHEAD + 0.5, -3.0))]
steer, found, _ = gap(scan_from_walls(walls))
chk(f"wall beyond {cn.GAP_LOOKAHEAD}m: still straight ahead",
    found and steer == 0.0)

print("\n-- hysteresis: symmetric twin gaps must not flip-flop")
# two posts, leaving an equal gap either side of centre
walls = [((1.0, 0.20), (1.0, -0.20))]
r = scan_from_walls(walls)
s_l, _, sign_l = gap(r, last_sign=+1.0)
s_r, _, sign_r = gap(r, last_sign=-1.0)
chk("prefers the side it chose last time",
    s_l > 0 and s_r < 0, f"after LEFT: {deg_of(s_l):+.0f}  "
                         f"after RIGHT: {deg_of(s_r):+.0f}")

print("\n-- the robot's own chassis must not look like a wall")
r = [0.10]*N        # everything inside MIN_VALID_DIST
steer, found, _ = gap(r)
chk("returns inside MIN_VALID_DIST ignored", found and steer == 0.0)


# ------------------- integration: the real scan_cb -------------------
# Everything above tests pick_gap in isolation. These drive the actual
# callback, so they also cover the profile speeds and the hand-off to the
# stop/reverse/turn maneuver.
print("\n-- integration: what scan_cb actually commands")

def drive(ranges, env="OUTDOOR"):
    """Returns (started_maneuver, commands_issued)."""
    o = object.__new__(cn.AGVControl)
    o.mode = "AUTO"; o.is_avoiding = False; o.blocked_hold = False
    o._blocked = False; o._was_blocked = False; o._clear_scans = 0
    o._last_log = 1e9
    o._cam_time = 0.0; o._cam_conf = 0.0; o._cam_count = 0
    o._was_cautious = False
    o._scan_blinded = False; o._blind_log = 0.0
    o._gap_sign = 0.0; o._gap_logged = 1e9
    o._last_turn = 0.0; o._last_turn_end = 0.0; o._turn_repeats = 0
    o.zones = {k: 9999.0 for k in
               ('front', 'front_r', 'right', 'back', 'left', 'front_l')}
    o.prof = dict(cn.ENV_PROFILES[env])
    o.cmd = sys.modules['geometry_msgs.msg'].Twist()
    sent = []
    o.set_cmd = lambda lx, az: sent.append((round(lx, 2), round(az, 2)))
    o.obstacle_pub = types.SimpleNamespace(publish=lambda m: None)
    o.action_pub = types.SimpleNamespace(publish=lambda m: None)
    o.publish_action = lambda t: None
    o.get_logger = lambda: types.SimpleNamespace(warn=lambda m: None,
                                                 info=lambda m: None)
    started = []
    cn.threading = types.SimpleNamespace(
        Thread=lambda target=None, args=(), daemon=None:
            types.SimpleNamespace(start=lambda: started.append(True)))
    msg = types.SimpleNamespace(angle_min=0.0,
                                angle_increment=math.radians(STEP),
                                ranges=ranges,
                                intensities=[6] * len(ranges))
    cn.AGVControl.scan_cb(o, msg)
    return bool(started), sent

for env, speed in (("OUTDOOR", 1.00), ("INDOOR", 0.50)):
    started, sent = drive([math.inf] * N, env)
    chk(f"{env}: open ground cruises at {speed:.2f} straight",
        not started and sent and sent[-1] == (speed, 0.0), f"{sent[-1]}")

# A wall across the path with open ground to the side: the OLD behaviour
# was to stop and reverse. It should now drive around.
walls = [((1.0, 0.30), (1.0, -3.0))]
started, sent = drive(scan_from_walls(walls))
chk("obstacle ahead, room to the left: steers, does not reverse",
    not started and sent and sent[-1][1] > 0, f"{sent[-1] if sent else None}")
chk("and it slows down to do it",
    sent and sent[-1][0] <= cn.GAP_SPEED, f"{sent[-1][0]}")

# A doorway it fits through: drive on through rather than stopping.
walls = [((1.0, 0.45), (1.0, 4.0)), ((1.0, -0.45), (1.0, -4.0))]
started, sent = drive(scan_from_walls(walls))
chk("90cm doorway ahead: drives through, no maneuver", not started)

# A wall with no way round inside the lookahead: maneuver is correct here.
walls = [((0.5, 3.0), (0.5, -3.0))]
started, sent = drive(scan_from_walls(walls))
chk("wall at 0.5m across everything: falls back to the maneuver", started)

# Too close to steer at all -- GAP_MIN_FRONT.
walls = [((0.25, 3.0), (0.25, -3.0))]
started, sent = drive(scan_from_walls(walls))
chk(f"obstacle inside GAP_MIN_FRONT ({cn.GAP_MIN_FRONT}m): maneuver, "
    f"no threading", started)

print("\nALL PASS" if ok else "\nFAILURES")
sys.exit(0 if ok else 1)
