#!/usr/bin/env python3
"""
COVER mode: the plan, and the lane-following state machine. Pure Python.

coverage_node.py is only the ROS wrapper around this. Keeping the logic here,
free of ROS, means the whole planner can be driven in a closed-loop
simulation (~/coverage_sim.py) with a robot that steers like the real one
and a GPS that wanders like the real one -- which is the only honest test of
a lane follower short of a field.

WHAT IT PRODUCES. Never a motor command. Each update returns either
("HOLD", why) or ("GO", steer, speed_cap): the direction it would like and
the fastest it wants to go. control_node turns that into motion THROUGH its
obstacle avoidance, so a detour always wins over the route.

THE PLAN.
  * Lanes run along the field's long axis (fewest turns), inside the stop
    line, LANE_SPACING_M apart -- set by GPS accuracy, not by the robot's
    width: at +/-2.5 m, lanes much closer than ~6 m would overlap by chance.
  * Each lane is cut short by a turning radius at both ends, so the turn
    onto the next lane happens INSIDE the stop line.
  * Wide spacing (>= turning diameter): straight into the next lane. Tight
    spacing: a loop-out turn (field_geometry.turn_waypoints).

FOLLOWING.
  * Steer at a point LOOKAHEAD_M ahead ON the lane line, not at the lane's
    end. After a detour that pulls the robot back onto the line rather than
    cutting a diagonal to the far end -- which is what "rejoin the lane"
    means in practice.
  * Heading comes from GPS positions HEADING_BASELINE_M apart (see
    field_geometry.TrackHeading). With no heading -- just started, or just
    reversed out of an avoidance manoeuvre -- it creeps straight ahead to
    build one, as nav_node does.

COVERAGE. A grid of COVER_CELL_M cells over the stop-line area, marked as
the robot drives (field_geometry.CoverageGrid). After the last lane, a
fill-in pass visits whatever was missed -- detours, cut corners -- nearest
first. Positions outside the stop-line area are never marked.

THE EDGE STRIP IS DELIBERATELY NOT COVERED. Lane ends are kept clear for
turning and the outer lanes kept off the stop line, both to stop GPS error
carrying the robot onto it; the fill-in pass does not chase cells within
FILL_EDGE_M of the line for the same reason. So a strip a few metres wide
round the edge reads as missed: ~87% coverage of the stop-line area with a
perfect GPS on an 80 x 40 m field. A headland lap round that strip was
tried and removed -- running parallel to the line, it tripped the fence at
every corner. It is the price of a fence that holds with +/-2.5 m GPS.

RETURN TO START. When the fill-in pass is done the robot drives back to
where COVER first started (the first GPS fix of the run) and stops there.
Same pure-pursuit steering, same obstacle avoidance and fence as the rest
of COVER -- it is just one more target. Three things shape it:
  * The target is kept RETURN_EDGE_M inside the stop line
    (field_geometry.point_well_inside). A start at the field's edge, or
    outside the stop line, is moved in to the nearest such point: the fence
    will not let it drive onto the line, so aiming there would end in a
    boundary recovery, not an arrival.
  * GPS decides "arrived", so it gets back only as close as GPS allows --
    within HOME_RADIUS_M of where the GPS SAYS the start is, which with the
    receiver measured on 2026-10-19 could be several metres from the real
    spot. Passing the start and then moving away also counts as arrived:
    with a drifting fix it would otherwise circle the point.
  * The start is often BEHIND it when coverage ends, and pursuit cannot
    turn round: aimed behind, sin(a) ~ 0 and it drives almost straight
    away. Full lock does not work either: the heading comes from positions
    5 m apart and a full-lock circle is only ~9 m round, so mid-turn the
    heading is meaningless -- in simulation the robot circled for minutes.
    A timed full-lock U-turn undershot (the heading it was timed from lags)
    and its straight run afterwards hit the stop line. What works: a start
    more than RETURN_UTURN_DEG off turns on a RETURN_TURN_RADIUS_M circle,
    wide enough that the 5 m heading stays usable (it lags ~30 deg there),
    until the start is ahead enough for pursuit to take over. It turns to
    whichever side the circle FITS inside the stop line (the start's side
    if both do), and keeps that side until the turn is done: on an outer
    lane, a 10 m circle toward the near edge crosses the stop line, and in
    simulation the boundary recoveries that followed ate the time allowance.
  * It gives up after RETURN_TIMEOUT -- blocked by an obstacle, or an
    L-shaped field putting the boundary in the straight-line way -- and
    stops where it is, saying so. Never a drive with no end.
A STOP mid-return resumes the return; the start point is saved with the
rest of the progress, so it is the ORIGINAL start, not where it resumed.

RESUME. to_dict()/load_dict() carry the lane, the phase and the grid, so a
STOP or E-stop mid-field resumes from the current lane, not from the start.
"""

import math

from . import field_geometry as fg

# ---------------------------------------------------------------- the plan
LANE_SPACING_M = 6.0        # >= 6 m: GPS is +/-2.5 m (see THE PLAN)
COVER_CELL_M = 3.0          # coverage grid resolution
# TODO: measure -- the circle the robot drives on full steering lock.
# Decides direct turns vs loop-outs, and how far lanes stop short of the
# stop line. 3.0 m is a placeholder, NOT a measurement.
TURN_DIAMETER_M = 3.0
# Lanes stop a turning radius PLUS this short of the stop line, so the
# U-turn onto the next lane happens clear of it. 1 m was not enough: a turn
# overshoots by the GPS lag and the +/-2.5 m error together, and in
# simulation most stop-line trips happened mid-turn -- side-on to the edge,
# exactly where the heading is least trustworthy. 3 m is ~2 sigma of GPS.
TURN_END_EXTRA_M = 3.0
# The outermost lanes keep at least this far inside the stop line. Centred
# lanes would sit only half a spacing (~3 m) in, and the follower's own
# tracking error under real GPS (2-3 m at the 95th percentile, in
# simulation) put the robot onto the line from there.
LANE_EDGE_M = 5.0

# ---------------------------------------------------------------- following
# Lookahead: far enough ahead that a heading several metres stale (the price
# of taking it from positions) still points the robot sensibly; close enough
# that it actually holds the lane. Tuned in coverage_sim.py with +/-2.5 m
# drifting GPS and Ackermann steering -- see the bench results there.
LOOKAHEAD_M = 10.0
# Heading baseline. Longer = less noise, more lag. With +/-2.5 m drifting GPS
# a 3 m baseline gave headings 10-15 degrees off, and pure pursuit steered
# faithfully by them -- the robot wandered off its lane into the stop line.
# 5 m roughly halves the error; the longer lookahead absorbs the extra lag.
HEADING_BASELINE_M = 5.0
# Steering is pure pursuit (field_geometry.pursuit_steer), which needs the
# turning RADIUS -- full lock = curvature 1/radius. Derived from the TODO
# diameter above; see pursuit_steer for why not nav_node's fixed-gain law.
LANE_END_M = 2.0            # within this of a lane's end (along it) = done
WAYPOINT_RADIUS_M = 2.5     # a turn / fill waypoint counts as reached
FILL_TIMEOUT_S = 90.0       # an unreachable fill point is given up on
FILL_MAX_POINTS = 200       # bound the fill-in pass
# Missed cells closer than this to the stop line are NOT visited by the
# fill-in pass. Reaching one means steering at the line itself, so every
# attempt ends in a boundary recovery -- in simulation the pass spent hours
# trying for a corner cell. They still count as missed in the coverage %;
# they are just not chased.
FILL_EDGE_M = WAYPOINT_RADIUS_M + 1.5
GPS_JUMP_M = 10.0           # a step this big between fixes is a spike:
                            # do not paint coverage along it

# speed caps sent with each GO -- control_node also applies its own limits
LANE_CAP = 1.00             # lanes: as fast as the ENV profile allows
TURN_CAP = 0.50             # turns: slower, they are the imprecise part
FILL_CAP = 0.60
CREEP_CAP = 0.40            # building a heading -- nav_node's TURN_SPEED.
                            # Blind straight-line driving lasts baseline/speed
                            # seconds, so not slower than it has to be.

# ---------------------------------------------------------------- reporting
# TODO: measure -- ground speed at the cruise command. Only used to REPORT
# soil-sample spacing and run time; nothing steers by it. 0.6 m/s is the
# simulator's figure (gps_node SIM_MAX_SPEED_MPS), not a measurement.
CRUISE_SPEED_MPS = 0.6
SOIL_INTERVAL_S = 60.0      # objective 1.3: one soil photo a minute

# ---------------------------------------------------------------- return
RETURN_TO_START = True      # False: stop wherever coverage finishes
RETURN_CAP = FILL_CAP
# The start point is moved at least this far inside the stop line -- the
# same clearance the fill-in pass keeps, for the same reason (FILL_EDGE_M).
RETURN_EDGE_M = FILL_EDGE_M
HOME_RADIUS_M = WAYPOINT_RADIUS_M   # this close (by GPS) = back at the start
# Within HOME_NEAR_M, getting HOME_PASSED_M further away than the closest
# it has been means it went past: call that arrived rather than loop round
# for a point GPS drift may have moved.
HOME_NEAR_M = HOME_RADIUS_M + 1.5
HOME_PASSED_M = 1.5
# Time allowed for the drive back: RETURN_SLACK x the straight-line time at
# cruise speed, and never less than RETURN_MIN_S. Generous on purpose --
# detours and fence recoveries take time; this only catches a robot that
# can no longer get there.
RETURN_SLACK = 4.0
RETURN_MIN_S = 180.0
# Start further off the nose than this: turn round first, on a circle of
# RETURN_TURN_RADIUS_M (see RETURN TO START). Steering for that circle is
# full lock x (turning radius / this), so it follows the TODO measurement.
RETURN_UTURN_DEG = 90.0
RETURN_TURN_RADIUS_M = 5.0

ACQUIRE, LANE, TURN, FILL, DONE = "ACQUIRE", "LANE", "TURN", "FILL", "DONE"
RETURN = "RETURN"


class CoveragePlan:
    def __init__(self, frame, poly, inset_poly, lanes, turns, spacing, angle):
        self.frame = frame
        self.poly = poly              # boundary, local x/y
        self.inset_poly = inset_poly  # stop line, local x/y
        self.lanes = lanes            # [(start, end)]
        self.turns = turns            # turns[i]: waypoints from lane i to i+1
        self.spacing = spacing
        self.angle = angle

    def lane_length(self):
        return sum(math.dist(a, b) for a, b in self.lanes)

    def turn_length(self):
        tot = 0.0
        for i, wps in enumerate(self.turns):
            p = self.lanes[i][1]
            for q in wps:
                tot += math.dist(p, q)
                p = q
        return tot

    def route_length(self):
        return self.lane_length() + self.turn_length()

    def summary(self, speed_mps=CRUISE_SPEED_MPS):
        L = self.route_length()
        loops = sum(1 for t in self.turns if len(t) > 1)
        soil = speed_mps * SOIL_INTERVAL_S
        return {
            "lanes": len(self.lanes),
            "spacing_m": round(self.spacing, 2),
            "lane_length_m": round(self.lane_length(), 1),
            "route_length_m": round(L, 1),
            "turns": len(self.turns),
            "loop_out_turns": loops,
            "soil_spacing_m": round(soil, 1),
            "soil_photos": int(L / soil) if soil > 0 else 0,
            "minutes": round(L / speed_mps / 60.0, 1) if speed_mps > 0 else 0,
            "area_m2": round(abs(fg.polygon_area(self.inset_poly)), 1),
        }

    def reversed(self):
        """The same route driven from the other end."""
        lanes = [(b, a) for a, b in reversed(self.lanes)]
        turns = []
        for i in range(len(lanes) - 1):
            turns.append(fg.turn_waypoints(lanes[i][0], lanes[i][1],
                                           lanes[i + 1][0], TURN_DIAMETER_M))
        return CoveragePlan(self.frame, self.poly, self.inset_poly, lanes,
                            turns, self.spacing, self.angle)

    def to_latlon(self):
        ll = self.frame.to_ll
        return {
            "lanes": [[list(ll(*a)), list(ll(*b))] for a, b in self.lanes],
            "turns": [[list(ll(*p)) for p in t] for t in self.turns],
            "summary": self.summary(),
        }


def make_plan(points_ll, inset_m, spacing=LANE_SPACING_M,
              turn_diameter=TURN_DIAMETER_M):
    """Plan lanes over a boundary given as lat/lon corners.
    Returns (plan, None) or (None, why)."""
    ok, why, clean = fg.validate_boundary(points_ll, inset_m)
    if not ok:
        return None, why
    frame = fg.LocalFrame(*clean[0])
    poly = [frame.to_xy(*p) for p in clean]
    inset = fg.inset_polygon(poly, inset_m)
    if inset is None:
        return None, "the stop line cannot be drawn for this shape"
    angle = fg.long_axis_angle(inset)
    margin = turn_diameter / 2.0 + TURN_END_EXTRA_M
    lanes, real = fg.lawnmower_lanes(inset, spacing, angle, end_margin=margin,
                                     side_margin=LANE_EDGE_M)
    if not lanes:
        return None, (f"no room for a lane once {margin:.1f} m is kept for "
                      f"turning at each end -- the field is too short")
    turns = [fg.turn_waypoints(lanes[i][0], lanes[i][1], lanes[i + 1][0],
                               turn_diameter)
             for i in range(len(lanes) - 1)]
    return CoveragePlan(frame, poly, inset, lanes, turns, real, angle), None


class CoverageFollower:
    """The lane-following state machine. See the module note."""

    def __init__(self, plan):
        self.plan = plan
        self.grid = fg.CoverageGrid(plan.inset_poly, COVER_CELL_M,
                                    plan.spacing)
        self.track = fg.TrackHeading(min_baseline=HEADING_BASELINE_M)
        self.phase = ACQUIRE
        self.lane = 0
        self.wp = 0                    # index into the current turn
        self.fill_target = None
        self.fill_since = 0.0
        self.fill_done = 0
        self.skipped = set()           # fill points given up on
        self.started = False           # chose which end to start from
        self.last = None               # last position, for painting coverage
        self.resume_phase = LANE
        self.home = None               # first fix of the run: where it started
        self.home_target = None        # home, moved well inside (RETURN)
        self.return_until = None       # RETURN gives up after this time
        self.return_best = math.inf    # closest it has got to home_target
        self.pos = None                # latest position, for status
        self.done_why = ""             # how the run ended, for the app
        self.turn_side = None          # RETURN turn-round: +1 left, -1 right

    # --------------------------------------------------------- persistence
    def to_dict(self):
        return {"phase": self.phase if self.phase != ACQUIRE
                else self.resume_phase,
                "lane": self.lane, "wp": self.wp,
                "fill_done": self.fill_done,
                "reversed": getattr(self, "is_reversed", False),
                "home": list(self.home) if self.home is not None else None,
                "done_why": self.done_why,
                "grid": self.grid.to_dict()}

    def load_dict(self, d):
        if d.get("reversed"):
            self.plan = self.plan.reversed()
            self.is_reversed = True
        self.lane = max(0, min(int(d.get("lane", 0)), len(self.plan.lanes) - 1))
        self.wp = int(d.get("wp", 0))
        self.fill_done = int(d.get("fill_done", 0))
        self.grid.load_dict(d.get("grid", {}))
        h = d.get("home")
        self.home = tuple(h) if h else None    # older saves: set at next fix
        self.done_why = d.get("done_why", "")
        ph = d.get("phase", LANE)
        self.resume_phase = ph if ph in (LANE, TURN, FILL, RETURN, DONE) \
            else LANE
        self.phase = ACQUIRE if self.resume_phase != DONE else DONE
        self.started = True

    def reset_heading(self):
        """After a STOP: the robot may have been moved by hand. Rebuild the
        heading before trusting it, and paint no coverage across the gap."""
        self.track.reset()
        self.last = None
        self.return_until = None       # a fresh time allowance on resume
        self.turn_side = None
        self.return_best = math.inf
        if self.phase not in (DONE, ACQUIRE):
            self.resume_phase = self.phase
            self.phase = ACQUIRE

    # --------------------------------------------------------- status
    def status(self):
        return {"phase": self.phase, "lane": min(self.lane + 1,
                                                 len(self.plan.lanes)),
                "lanes": len(self.plan.lanes),
                "coverage": round(self.grid.coverage_pct(), 1),
                "overlap": round(self.grid.overlap_pct(), 1),
                "fill_done": self.fill_done,
                "home_m": (round(math.dist(self.pos, self.home_target), 1)
                           if self.phase == RETURN and self.pos is not None
                           and self.home_target is not None else None),
                "note": self.done_why}

    # --------------------------------------------------------- update
    def _paint(self, p, lane_id):
        if self.last is not None and math.dist(self.last, p) <= GPS_JUMP_M:
            # only the parts inside the stop-line area count (see COVERAGE)
            if (fg.point_in_polygon(p, self.plan.inset_poly) and
                    fg.point_in_polygon(self.last, self.plan.inset_poly)):
                self.grid.mark_segment(self.last, p, lane_id)
        self.last = p

    def _coverage_finished(self, p):
        """After the fill-in pass: head back to the start, or stop here."""
        if RETURN_TO_START and self.home is not None:
            self.home_target = fg.point_well_inside(
                self.home, self.plan.inset_poly, RETURN_EDGE_M)
            if self.home_target is None:
                return self._finish("coverage complete -- the field is too "
                                    "small to drive back to the start safely")
            if math.dist(p, self.home_target) > HOME_RADIUS_M:
                self.phase = RETURN
                self.return_until = None
                self.return_best = math.inf
                return None
            return self._finish("coverage complete, back at the start")
        return self._finish("coverage complete")

    def _return_check(self, t, p):
        """RETURN: ("HOLD", why) once home or out of time, else None."""
        if self.home_target is None:              # resumed into RETURN
            self.home_target = fg.point_well_inside(
                self.home, self.plan.inset_poly, RETURN_EDGE_M)
            if self.home_target is None:
                return self._finish("coverage complete -- the field is too "
                                    "small to drive back to the start safely")
        d = math.dist(p, self.home_target)
        if self.return_until is None:
            allow = RETURN_SLACK * d / max(CRUISE_SPEED_MPS, 0.1)
            self.return_until = t + max(RETURN_MIN_S, allow)
        self.return_best = min(self.return_best, d)
        if d <= HOME_RADIUS_M:
            return self._finish("coverage complete, back at the start")
        if self.return_best <= HOME_NEAR_M and \
                d >= self.return_best + HOME_PASSED_M:
            return self._finish(
                f"coverage complete, back at the start (passed within "
                f"{self.return_best:.1f} m by GPS)")
        if t > self.return_until:
            return self._finish(
                f"coverage complete, but could not get back to the start "
                f"-- stopped {d:.0f} m short")
        return None

    def _turn_side(self, p, h, a):
        """Which way to turn round for the start: +1 left, -1 right. The
        start's side if its circle fits inside the stop line, else the side
        that fits, else the side with more room."""
        toward = -1 if a > 0 else 1                   # + a = start to the RIGHT
        room = {}
        for side in (toward, -toward):
            b = math.radians(h - 90.0 * side)         # compass: left = h - 90
            c = (p[0] + RETURN_TURN_RADIUS_M * math.sin(b),
                 p[1] + RETURN_TURN_RADIUS_M * math.cos(b))
            d = fg.dist_to_boundary(c, self.plan.inset_poly)
            room[side] = d if fg.point_in_polygon(c, self.plan.inset_poly) \
                else -d
            if room[side] >= RETURN_TURN_RADIUS_M:
                return side
        return max(room, key=room.get)

    def _finish(self, why):
        self.phase = DONE
        self.done_why = why
        return ("HOLD", why)

    def _go_to(self, p, h, target, cap):
        return ("GO", fg.pursuit_steer(p, h, target, TURN_DIAMETER_M / 2.0),
                cap)

    def update(self, t, p, direction):
        """One step. t: seconds; p: (x, y) local or None (no fix);
        direction: +1 forward, -1 reverse, 0 stopped (from /cmd_vel).
        Returns ("HOLD", why) or ("GO", steer, cap)."""
        if p is None:
            self.last = None
            return ("HOLD", "no GPS fix")
        self.pos = p
        if self.home is None:
            self.home = tuple(p)         # where this run started
        self.track.feed(t, p[0], p[1], direction)
        h = self.track.heading()

        if not self.started:
            # Start from whichever end of the route is nearer.
            a = self.plan.lanes[0][0]
            b = self.plan.lanes[-1][1]
            if math.dist(p, b) < math.dist(p, a) and len(self.plan.lanes) > 1:
                self.plan = self.plan.reversed()
                self.is_reversed = True
            self.started = True

        if self.phase == DONE:
            return ("HOLD", self.done_why or "coverage complete")

        if self.phase == RETURN or (self.phase == ACQUIRE and
                                    self.resume_phase == RETURN):
            # Arrived or out of time is decided by position alone -- before
            # the heading check, so creeping to rebuild a heading cannot
            # carry it past the start or past its time allowance.
            done = self._return_check(t, p)
            if done is not None:
                return done

        if h is None:
            # No heading yet: creep straight to build one (as nav_node does).
            self.last = None if self.phase == ACQUIRE else self.last
            return ("GO", 0.0, CREEP_CAP)
        if self.phase == ACQUIRE:
            self.phase = self.resume_phase

        if self.phase == LANE:
            a, b = self.plan.lanes[self.lane]
            self._paint(p, self.lane)
            L = math.dist(a, b)
            if fg.progress_along(a, b, p) * L >= L - LANE_END_M:
                if self.lane + 1 < len(self.plan.lanes):
                    self.phase, self.wp = TURN, 0
                else:
                    self.phase = FILL
                return self.update(t, p, direction)
            target, _ = fg.lookahead_point(a, b, p, LOOKAHEAD_M)
            return self._go_to(p, h, target, LANE_CAP)

        if self.phase == TURN:
            self._paint(p, self.lane)
            wps = self.plan.turns[self.lane]
            while self.wp < len(wps) and \
                    math.dist(p, wps[self.wp]) <= WAYPOINT_RADIUS_M:
                self.wp += 1
            if self.wp >= len(wps):
                self.lane += 1
                self.phase, self.wp = LANE, 0
                return self.update(t, p, direction)
            # Past the last waypoint and already on the next lane's line,
            # heading the right way: join the lane rather than circling back
            # for a point GPS noise made us miss.
            nxt_a, nxt_b = self.plan.lanes[self.lane + 1]
            if self.wp == len(wps) - 1 and \
                    abs(fg.cross_track(nxt_a, nxt_b, p)) < WAYPOINT_RADIUS_M and \
                    0.0 <= fg.progress_along(nxt_a, nxt_b, p) <= 0.5:
                self.lane += 1
                self.phase, self.wp = LANE, 0
                return self.update(t, p, direction)
            return self._go_to(p, h, wps[self.wp], TURN_CAP)

        if self.phase == FILL:
            self._paint(p, 1000 + self.fill_done)
            if self.fill_target is not None:
                if math.dist(p, self.fill_target) <= WAYPOINT_RADIUS_M:
                    self.fill_target = None
                    self.fill_done += 1
                elif t - self.fill_since > FILL_TIMEOUT_S:
                    self.skipped.add(self.fill_target)
                    self.fill_target = None
            if self.fill_target is None:
                missed = [m for m in self.grid.missed()
                          if m not in self.skipped and
                          fg.dist_to_boundary(m, self.plan.inset_poly)
                          >= FILL_EDGE_M]
                if not missed or self.fill_done >= FILL_MAX_POINTS:
                    self.fill_target = None
                    fin = self._coverage_finished(p)
                    return fin if fin is not None else \
                        self.update(t, p, direction)
                self.fill_target = min(missed, key=lambda m: math.dist(p, m))
                self.fill_since = t
            return self._go_to(p, h, self.fill_target, FILL_CAP)

        if self.phase == RETURN:
            # Transit, not coverage: nothing painted (it would only add to
            # the overlap figure). Arrival and time-out were checked above.
            self.last = None
            # Aim LOOKAHEAD_M along the straight line home, not at home
            # itself: pursuit's gain falls with distance, and aimed at a
            # start 30 m off it drove arcs so wide (in simulation, perfect
            # GPS) they ran into the stop line again and again. Re-aimed
            # from wherever it is each fix, so after a detour or a boundary
            # recovery it takes the shortest way, not back to an old line.
            hx, hy = self.home_target
            a = fg.norm180(fg.bearing_xy(hx - p[0], hy - p[1]) - h)
            if abs(a) > RETURN_UTURN_DEG:
                if self.turn_side is None:
                    self.turn_side = self._turn_side(p, h, a)
                s = min(1.0, TURN_DIAMETER_M / 2.0 / RETURN_TURN_RADIUS_M)
                return ("GO", s * self.turn_side, RETURN_CAP)
            self.turn_side = None
            d = math.dist(p, self.home_target)
            k = min(1.0, LOOKAHEAD_M / d)
            aim = (p[0] + (hx - p[0]) * k, p[1] + (hy - p[1]) * k)
            return ("GO", fg.pursuit_steer(p, h, aim, TURN_DIAMETER_M / 2.0),
                    RETURN_CAP)

        return ("HOLD", f"unknown phase {self.phase}")
