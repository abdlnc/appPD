#!/usr/bin/env python3
"""
AGV Motor Node  (ROS2 / rclpy)

The ONLY node that touches the GPIO. Everything else just sends commands.

Subscribes:
    /cmd_vel  (geometry_msgs/Twist)   drive + steer commands
    /mode     (std_msgs/String)       "AUTO" | "MANUAL" | "STOP" | "NAV"

Twist convention (normalized):
    linear.x   forward speed   -1.0 .. +1.0   (+ = forward, - = reverse)
    angular.z  steering        -1.0 .. +1.0   (+ = LEFT, - = RIGHT)

Safety:
  * Starts in STOP mode (nothing moves until a mode is set).
  * STOP mode halts the motors immediately and re-centers steering.
  * TWO steering servos are supported (SERVO_PIN / SERVO2_PIN), each with
    its own CENTER/LEFT/RIGHT calibration so they hold the same wheel angle
    instead of straining against each other. Set SERVO2_PIN = None for a
    single-servo robot.
  * Command watchdog: if no /cmd_vel arrives within CMD_TIMEOUT while moving
    (a publisher died / app disconnected), the motors are stopped automatically.
  * SPEED_SCALE caps the top speed (slower, safer pace).
  * REVERSE GUARD: on a direction flip (forward <-> reverse) the motors are
    held at ZERO for REVERSE_PAUSE seconds first -- a true full stop before
    reversing, so the rear gears are never slammed.
"""

import os
import subprocess
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import RPi.GPIO as GPIO   # provided by rpi-lgpio on the Pi 5

# ------------------------- GPIO PINS -------------------------
SERVO_PIN  = 18                      # front steering servo A
# Second steering servo. Set to None for a single-servo robot.
# GPIO19 chosen because it is free here and is one of the Pi's hardware-PWM
# capable pins (12/13/18/19), of which 12, 13 and 18 are already taken.
# The two servos MUST NOT share one signal wire: they need separate trims
# (see below), which is impossible if they get identical pulses.
SERVO2_PIN = 19                      # front steering servo B, or None
LEFT_FWD = 12;  RIGHT_FWD = 13       # rear drive (forward)
LEFT_BWD = 22;  RIGHT_BWD = 23       # rear drive (backward)

# ------------- STEERING CALIBRATION (measured, no gear skip) -------------
# Widened from the original 4.0/12.0 (testing more range on
# the swapped-in MG996R). CENTER unchanged -- that's the measured true-
# straight position, not something to guess at. This still sits close to
# the ~0.5-2.5ms standard hobby-servo pulse range (duty% * 20ms @ 50Hz), but
# if the servo ever strains/grinds harder at the new extremes (vs. just not
# moving) than before, back these off immediately -- that's the mechanical
# end-stop, not a power issue.
# ONE SET OF THREE NUMBERS PER SERVO, measured with ~/steering_calibrate.py.
#
# They are a property of the SERVO AND ITS HORN, not of the code: pressing a
# horn on one spline tooth over shifts centre by ~18 degrees, and two servos
# of the same model rarely centre at the same pulse. That is why servo B has
# its own three values rather than reusing servo A's -- with a shared trim,
# two servos linked to the same steering permanently push against each other,
# which shows up as buzzing, heat and a sagging supply.
#
# MIRRORED MOUNTING needs no special flag: if servo B faces the other way,
# its LEFT value simply comes out LOWER than its CENTER. steer() derives the
# direction from the numbers themselves.
#
# !!! THE VALUES BELOW ARE PLACEHOLDERS -- the full 180-degree span of a
# hobby servo (0.5 / 1.5 / 2.5 ms), NOT a measurement of this robot. A
# steering linkage typically only swings +/-25-35 degrees, so at full
# joystick these drive the servo into the steering's mechanical stop and
# stall it there. Run steering_calibrate.py (once per servo, --pin 18 then
# --pin 19) and replace all six numbers BEFORE driving.
# INTERIM limits (2026-09-21), still NOT a measurement: +/-4.2% duty
# (+/-0.83ms, roughly +/-75 deg) either side of the servos' own midpoint.
# Widened 1.5% -> 2.5% -> 4.2% on request, because at the narrower spans the
# WHEELS barely deflected: the linkage gears the horn's travel down, so horn
# degrees are not wheel degrees.
# This is ~83% of a 180-degree servo's full travel and leaves little margin
# before the linkage's mechanical stop -- the thing that jammed and locked a
# servo's gears earlier today. STEER_SLEW still paces the approach, so a
# full-lock command sweeps in over ~0.7s instead of slamming, but it cannot
# prevent a stall if the linkage stops before these values do.
# Back these off at the FIRST sign of strain or grinding, and replace them
# with steering_calibrate.py's measured output.
#
# BENCH-CHECKED 2026-09-21: both servos ramped through +/-60 and then +/-75
# on hardware PWM, one at a time, with no strain at either end and no jitter
# (servo rail at 5V -- see the jitter note below). So +/-75 is inside the
# linkage's travel on this robot. Still not a per-servo measurement.
# Narrow on purpose. The full 180-degree span that was here before let a
# full-joystick command drive a servo past what the steering linkage can
# move, which jammed it against the mechanical stop and locked the gears.
# These keep the excursion small enough that a jam is unlikely while the
# real limits are still unmeasured -- steering will feel restricted, and
# that is the trade being made deliberately.
# Replace all six with steering_calibrate.py's output, per servo.
# 2026-09-21: widened +/-75 -> +/-90 on request. This is the servo's FULL
# travel (0.5 / 1.5 / 2.5 ms) -- there is nothing wider to go to. +/-75 was
# bench-checked strain-free on both sides; +/-90 is not yet. If full lock
# strains, buzzes or grinds, go back to 11.7 / 3.3.
STEER_CENTER  = 7.5                  # servo A: 1.5ms, straight ahead
STEER_LEFT    = 12.5                 # servo A: angular.z = +1.0  (2.5ms)
STEER_RIGHT   = 2.5                  # servo A: angular.z = -1.0  (0.5ms)

STEER2_CENTER = 7.5                  # servo B: straight ahead
STEER2_LEFT   = 12.5                 # servo B: angular.z = +1.0  (2.5ms)
STEER2_RIGHT  = 2.5                  # servo B: angular.z = -1.0  (0.5ms)

# ----------- Steering write protection -----------
# motor_node re-sends the steering duty on EVERY /cmd_vel, which arrives
# 10-20x a second whether or not the stick has actually moved. Each rewrite
# of a SOFTWARE-generated PWM is a chance to emit a malformed pulse, and a
# servo obeys whatever pulse it is handed -- a plausible way to get a sudden
# unexplained slam to one side. So:
#
#   STEER_EPS      below this, the duty has not meaningfully changed and is
#                  NOT rewritten at all. Kills the constant idle churn.
#   STEER_SLEW     fastest the steering may move, in duty-% per second.
#                  A full-lock request sweeps there instead of slamming in
#                  one jump. Slamming is what strips gears when the servo
#                  meets the linkage's stop, so this limits the damage of
#                  any command -- or glitch -- that would otherwise arrive
#                  as an instant full-throw.
#   STEER_HZ       how often the servo is moved toward its target. This used
#                  to happen once per /cmd_vel (10Hz), so motion came in
#                  visible 0.6%-duty STEPS that read as stutter. At 50Hz the
#                  same speed is split into steps too small to see -- the
#                  smooth ramp confirmed on the bench on 2026-09-21.
#
# Neither hides an ELECTRICAL fault (a motor-current brownout dragging the
# servo rail down will still misbehave); they stop the software adding to it.
STEER_EPS      = 0.02
STEER_SLEW     = 6.0         # duty-%/s  (0 -> full +/-75 lock in ~0.7s)
STEER_HZ       = 50.0

# ----------- Hardware PWM for the steering servos (Pi 5) -----------
# Steering is driven by the Pi's HARDWARE PWM rather than RPi.GPIO's
# software PWM. On the Pi 5, RPi.GPIO is the rpi-lgpio shim, which times
# each pulse in a userspace process competing with the camera, AI and SLAM;
# every scheduling delay shifts a pulse edge, and the servo chases it. With
# hardware PWM the pulse width lives in a register and cannot drift.
#
# Needs the pwm-2chan overlay (in /boot/firmware/config.txt):
#     dtoverlay=pwm-2chan,pin=18,func=2,pin2=19,func2=2
# which routes GPIO18/19 to the RP1 chip's PWM0 block.
#
# CHANNEL NUMBERS ARE PI 5 SPECIFIC. On a Pi 4 GPIO18/19 are PWM channels
# 0/1; on the Pi 5's RP1 they are channels 2/3 (`pinctrl get 18,19` shows
# PWM0_CHAN2 / PWM0_CHAN3). Driving channel 0 moves nothing -- that exact
# mistake cost a round of bench testing.
#
# If the overlay is missing (fresh OS, config.txt edited back), the node
# falls back to software PWM and logs a warning, so the robot still steers
# -- just with the old jitter.
HW_PWM_DEVICE  = "1f00098000.pwm"    # RP1 PWM0; its pwmchipN number varies
HW_PWM_CHANNEL = {18: 2, 19: 3}      # BCM pin -> RP1 PWM0 channel
HW_PWM_PERIOD  = 20_000_000          # ns = 50Hz, standard servo frame
# Pin FUNCTION for PWM on the RP1 (what `pinctrl get 18` prints as "a3").
# The overlay sets this at boot, but anything that later drives these pins
# through RPi.GPIO takes the mux away -- and GPIO.cleanup() leaves the pin an
# INPUT. The PWM channel then keeps pulsing into a disconnected pin: the log
# says HARDWARE PWM, sysfs looks perfect, and the servo gets nothing. Seen
# for real on 2026-09-23 after a software-PWM comparison test. So re-assert
# the function on every start rather than trusting whatever left it behind.
HW_PWM_PIN_FUNC = "a3"

# ----------- Steering jitter control -----------
# A servo holding position on a SOFTWARE-generated PWM twitches: the Pi is a
# multitasking computer, so each pulse lands a few microseconds early or late
# and the servo chases the difference. Two things reduce it here.
#
# STEER_DEADBAND: a touch joystick never returns exactly 0.0 -- it settles
# around +/-0.01-0.03 and wanders. Without a deadband those wobbles become
# real duty changes, and the servo hunts around centre following noise that
# means nothing. Inside this band the steering snaps to exact centre.
STEER_DEADBAND = 0.05        # |angular.z| below this = dead straight
# Same idea on the throttle axis. drive() already ignored anything this small
# for the MOTORS, but the raw value was still being remembered as "the robot
# is driving", which held the steering release off forever: a touch joystick
# reports linear.x values like -0.0008 rather than a clean 0.0.
THROTTLE_DEADBAND = 0.02     # |linear.x| below this = stopped
#
# STEER_IDLE_RELEASE: after this long with the robot NOT driving and the
# steering not changing, stop sending pulses entirely. An unpowered servo
# holds its angle on gear friction alone and goes completely silent -- this
# is the only way to remove software-PWM jitter outright rather than reduce
# it. Pulses resume the instant a new steering command arrives.
#
# Deliberately gated on ZERO THROTTLE: released steering can be pushed out of
# position by the ground, which is fine while parked and is not fine while
# driving. Set to 0 to disable and always hold.
STEER_IDLE_RELEASE = 1.5     # s of stationary + unchanged steering

# ------------------------- SAFETY / TUNING -------------------------
CMD_TIMEOUT = 0.7                    # s; stop if no /cmd_vel within this window
MAX_DUTY    = 100.0                  # absolute cap on motor PWM %

# Master cap on the REAR DRIVE motors. Top speed = SPEED_SCALE * 100% duty,
# and it multiplies every mode: MANUAL (bridge_node's MANUAL_SPEED), AUTO
# (control_node's FWD_SPEED) and NAV (nav_node's CRUISE_SPEED).
#
# Raised 0.60 -> 1.00 on request (2026-09-24): the motors were being held to
# 60% of what the driver can deliver, so manual and AUTO ran at 36% duty and
# NAV at 48%. At 1.00 those become 60% / 60% / 80%; the per-mode numbers are
# unchanged, the ceiling above them is simply gone.
#
# It was 0.60 as a deliberate "slower, safer pace". Faster driving means less
# time to react at the UNCHANGED 0.60m obstacle thresholds
# (control_node's MIN_DIST_FRONT / nav_node's FRONT_STOP_DIST), so if the
# robot starts clipping obstacles, raise those thresholds rather than
# rediscovering this line. Going back down is one number here.
# Don't set it much below ~0.40: plain DC motors may stall under the
# robot's weight instead of turning.
SPEED_SCALE = 1.00

# Full stop bago mag-reverse (gear protection).
# Kapag nag-flip ang direction habang gumagalaw, i-hohold muna sa ZERO ang
# motors na xxng ganito katagal bago i-apply ang kabilang direction.
REVERSE_PAUSE = 0.4                  # s; taasan (0.6) kung gusto ng mas luwag


class _HwPwm:
    """RP1 hardware PWM channel via sysfs, with the same two methods motor_node
    uses on RPi.GPIO's PWM objects -- ChangeDutyCycle(duty %) and stop() -- so
    the rest of the node does not care which one it has.

    Writable without sudo: Raspberry Pi OS's udev rules give the pwm sysfs
    files to group `gpio`, and the service user is in it."""

    def __init__(self, pin):
        chip = None
        for c in os.listdir("/sys/class/pwm"):
            dev = os.path.realpath(f"/sys/class/pwm/{c}/device")
            if os.path.basename(dev) == HW_PWM_DEVICE:
                chip = f"/sys/class/pwm/{c}"
                break
        if chip is None:
            raise OSError("RP1 PWM0 not present -- pwm-2chan overlay not loaded")
        # Claim the pin for the PWM block (see HW_PWM_PIN_FUNC). Works
        # unprivileged: the service user is in the `gpio` group.
        try:
            subprocess.run(["pinctrl", "set", str(pin), HW_PWM_PIN_FUNC],
                           check=True, capture_output=True, timeout=5)
        except (OSError, subprocess.SubprocessError) as e:
            raise OSError(f"could not set GPIO{pin} to PWM function: {e}")
        ch = HW_PWM_CHANNEL[pin]
        self.path = f"{chip}/pwm{ch}"
        if not os.path.isdir(self.path):
            self._w(f"{chip}/export", ch)
        # udev applies the group permissions a moment AFTER export creates the
        # files; writing before then fails with EACCES, so wait for it.
        deadline = time.time() + 2.0
        while True:
            try:
                self._w(f"{self.path}/period", HW_PWM_PERIOD)
                break
            except OSError:
                if time.time() > deadline:
                    raise
                time.sleep(0.05)

    @staticmethod
    def _w(path, value):
        with open(path, "w") as f:
            f.write(str(int(value)))

    def start(self, duty):
        self.ChangeDutyCycle(duty)
        self._w(f"{self.path}/enable", 1)

    def ChangeDutyCycle(self, duty):
        # duty % of a 20ms frame -> pulse width in ns. 0 = no pulses at all,
        # which is what release() relies on to silence the servo.
        self._w(f"{self.path}/duty_cycle", duty * HW_PWM_PERIOD / 100.0)

    def stop(self):
        self._w(f"{self.path}/enable", 0)


class _Servo:
    """One steering servo: its calibration, where it IS, and where it is
    GOING.

    steer() only sets `target`; the 50Hz steer_tick() walks `last` toward it
    at STEER_SLEW. Splitting the two is what makes motion smooth instead of
    arriving in one visible jump per /cmd_vel."""

    def __init__(self, pwm, center, left, right, hardware):
        self.pwm = pwm
        self.center = center
        self.left = left
        self.right = right
        self.hardware = hardware  # True = RP1 hardware PWM, False = software
        self.last = center        # what the servo was last told
        self.target = center      # where it is heading
        self.released = False     # True = not pulsing (see release())
        self.last_change = time.time()

    def step(self, max_step):
        """Move one tick toward target. Called at STEER_HZ."""
        delta = self.target - self.last
        if abs(delta) < STEER_EPS:
            # Already there: leave the servo exactly as it is, holding OR
            # released. (An earlier version re-engaged a released servo on
            # every unchanged command; at 10Hz that made it release and wake
            # endlessly, which was itself a visible twitch.)
            return
        self._write(self.last + max(-max_step, min(max_step, delta)))

    def snap(self, duty):
        """Go straight to `duty`, no slew. Fail-safe path only."""
        self.target = duty
        if abs(duty - self.last) >= STEER_EPS:
            self._write(duty)

    def _write(self, duty):
        self.pwm.ChangeDutyCycle(duty)
        self.last = duty
        self.released = False
        self.last_change = time.time()

    def release(self):
        """Stop pulsing: the servo stops holding, and stops twitching. It
        keeps its angle on gear friction. The next real target change
        re-engages it."""
        if self.released:
            return
        self.pwm.ChangeDutyCycle(0)                 # no pulses at all
        self.released = True


class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')

        # --- GPIO setup ---
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        # Steering servos: each keeps its OWN calibration, so two servos on
        # one linkage hold the same wheel angle instead of fighting.
        self.last_throttle = 0.0              # gates the steering idle release
        self.servos = []                      # [_Servo]
        for pin, cal in ((SERVO_PIN, (STEER_CENTER, STEER_LEFT, STEER_RIGHT)),
                         (SERVO2_PIN, (STEER2_CENTER, STEER2_LEFT,
                                       STEER2_RIGHT))):
            if pin is None:
                continue
            # Hardware PWM first. Crucially this must NOT call GPIO.setup()
            # on the pin: that re-muxes it from PWM back to a plain GPIO
            # output and silently disconnects the hardware channel.
            try:
                pwm = _HwPwm(pin)
                pwm.start(cal[0])
                hardware = True
            except (OSError, KeyError) as e:
                self.get_logger().warn(
                    f"GPIO{pin}: hardware PWM unavailable ({e}) -- falling "
                    f"back to software PWM; steering WILL jitter. Add "
                    f"dtoverlay=pwm-2chan,pin=18,func=2,pin2=19,func2=2 to "
                    f"/boot/firmware/config.txt and reboot.")
                GPIO.setup(pin, GPIO.OUT)
                pwm = GPIO.PWM(pin, 50)
                pwm.start(cal[0])
                hardware = False
            self.servos.append(_Servo(pwm, *cal, hardware=hardware))
        for pin in (LEFT_FWD, RIGHT_FWD, LEFT_BWD, RIGHT_BWD):
            GPIO.setup(pin, GPIO.OUT)
        self.l_fwd = GPIO.PWM(LEFT_FWD, 1000);  self.l_fwd.start(0)
        self.r_fwd = GPIO.PWM(RIGHT_FWD, 1000); self.r_fwd.start(0)
        self.l_bwd = GPIO.PWM(LEFT_BWD, 1000);  self.l_bwd.start(0)
        self.r_bwd = GPIO.PWM(RIGHT_BWD, 1000); self.r_bwd.start(0)

        # --- state ---
        self.mode = "STOP"                 # start safe
        self.last_cmd_time = 0.0
        self.last_drive = (0.0, 0.0)
        self.stopped = True
        self.last_motion_dir = 0           # -1 reverse / 0 none / +1 forward
        self.last_motion_time = 0.0        # when we last drove in that dir
        self._flip_logged = False

        # --- ROS interfaces ---
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_subscription(String, '/mode', self.mode_cb, 10)
        self.create_timer(0.1, self.watchdog)   # 10 Hz safety check
        self.create_timer(0.1, self.steering_idle_check)
        self.create_timer(1.0 / STEER_HZ, self.steer_tick)

        self.get_logger().info(
            f"Motor node ready (GPIO owner). Mode: STOP | "
            f"speed cap {int(SPEED_SCALE * 100)}% | reverse pause {REVERSE_PAUSE}s"
            f" | steering: " + ", ".join(
                f"GPIO{p}={'HARDWARE' if sv.hardware else 'software'} PWM"
                for p, sv in zip([q for q in (SERVO_PIN, SERVO2_PIN)
                                  if q is not None], self.servos)))

    # ----------------------- helpers -----------------------
    def steer(self, angular_z):
        """Map angular.z (-1..+1, + = left) onto EVERY steering servo's
        TARGET, each through its own calibration. The motion itself happens
        in steer_tick(), at STEER_SLEW."""
        az = max(-1.0, min(1.0, angular_z))
        if abs(az) < STEER_DEADBAND:
            az = 0.0                  # joystick noise near centre -> straight
        for sv in self.servos:
            if az >= 0:                               # left half
                duty = sv.center + az * (sv.left - sv.center)
            else:                                     # right half
                duty = sv.center + az * (sv.center - sv.right)
            # Clamp with min/max rather than assuming left > right: on a
            # servo mounted mirrored, LEFT is the LOWER duty and a fixed
            # ordering here would clamp every command back to centre.
            lo, hi = ((sv.left, sv.right) if sv.left < sv.right
                      else (sv.right, sv.left))
            sv.target = max(lo, min(hi, duty))

    def steer_tick(self):
        """50Hz: walk every servo one small step toward its target."""
        max_step = STEER_SLEW / STEER_HZ
        for sv in self.servos:
            sv.step(max_step)

    def center_steering(self):
        """Park every servo at ITS OWN straight-ahead value. Not rate-limited:
        this is the fail-safe path (STOP, watchdog, shutdown) and must take
        effect immediately."""
        for sv in self.servos:
            sv.snap(sv.center)

    def _apply_duty(self, fwd, bwd):
        self.l_fwd.ChangeDutyCycle(fwd); self.r_fwd.ChangeDutyCycle(fwd)
        self.l_bwd.ChangeDutyCycle(bwd); self.r_bwd.ChangeDutyCycle(bwd)
        self.last_drive = (fwd, bwd)

    def steering_idle_check(self):
        """Stop pulsing the servos once the robot has been sitting still with
        unchanged steering -- see STEER_IDLE_RELEASE. This is what silences
        the twitch when parked."""
        if STEER_IDLE_RELEASE <= 0:
            return
        if self.last_throttle != 0.0:
            return                          # driving: keep holding the angle
        now = time.time()
        for sv in self.servos:
            if not sv.released and (now - sv.last_change) > STEER_IDLE_RELEASE:
                sv.release()
                self.get_logger().info(
                    "steering idle -> pulses released (servo holds on "
                    "friction, no twitch); resumes on next command")

    def drive(self, linear_x):
        """Map linear.x (-1..+1) to rear-motor PWM.

        Speed is capped by SPEED_SCALE, and a direction flip goes through a
        mandatory REVERSE_PAUSE at zero duty (full stop) to protect the gears.
        """
        lx = max(-1.0, min(1.0, linear_x))
        if abs(lx) < THROTTLE_DEADBAND:
            lx = 0.0                  # joystick noise -> genuinely stopped
        self.last_throttle = lx
        desired = 1 if lx > 0.0 else (-1 if lx < 0.0 else 0)
        now = time.time()

        if desired == 0:
            # stick released / zero command -> motors to zero
            self._apply_duty(0.0, 0.0)
            return

        # ---- FULL-STOP-BEFORE-REVERSE GUARD (gear protection) ----
        # If the new direction opposes the last motion and that motion was
        # recent, hold ZERO duty until REVERSE_PAUSE has elapsed. Only then
        # is the opposite direction applied.
        if (self.last_motion_dir != 0 and desired != self.last_motion_dir
                and (now - self.last_motion_time) < REVERSE_PAUSE):
            self._apply_duty(0.0, 0.0)
            if not self._flip_logged:
                self.get_logger().info(
                    "Direction flip: full stop before reverse (gear guard)")
                self._flip_logged = True
            return
        self._flip_logged = False

        duty = min(MAX_DUTY, abs(lx) * 100.0 * SPEED_SCALE)

        # brief 100% kick to overcome stall when starting from standstill
        starting = (self.last_drive == (0.0, 0.0))
        if starting:
            if desired > 0:
                self.l_fwd.ChangeDutyCycle(100); self.r_fwd.ChangeDutyCycle(100)
                self.l_bwd.ChangeDutyCycle(0);   self.r_bwd.ChangeDutyCycle(0)
            else:
                self.l_bwd.ChangeDutyCycle(100); self.r_bwd.ChangeDutyCycle(100)
                self.l_fwd.ChangeDutyCycle(0);   self.r_fwd.ChangeDutyCycle(0)
            time.sleep(0.05)

        if desired > 0:
            self._apply_duty(duty, 0.0)
        else:
            self._apply_duty(0.0, duty)

        self.last_motion_dir = desired
        self.last_motion_time = now

    def hard_stop(self):
        self._apply_duty(0.0, 0.0)
        self.stopped = True
        # The robot is no longer driving, so the steering idle release may
        # engage. Without this the last throttle value lingers and the servos
        # keep pulsing (and twitching) after a STOP.
        self.last_throttle = 0.0

    # ----------------------- callbacks -----------------------
    def mode_cb(self, msg: String):
        m = msg.data.strip().upper()
        if m not in ("AUTO", "MANUAL", "STOP", "NAV"):
            return
        if m != self.mode:
            self.get_logger().info(f"Mode -> {m}")
        self.mode = m
        if m == "STOP":
            self.hard_stop()
            self.center_steering()

    def cmd_cb(self, msg: Twist):
        self.last_cmd_time = time.time()
        if self.mode == "STOP":
            self.hard_stop()
            return
        self.steer(msg.angular.z)
        self.drive(msg.linear.x)
        self.stopped = False

    def watchdog(self):
        """Stop the motors if commands stop arriving while we're moving."""
        if self.mode != "STOP" and not self.stopped:
            if time.time() - self.last_cmd_time > CMD_TIMEOUT:
                self.get_logger().warn("cmd_vel timeout -> stopping")
                self.hard_stop()
                self.center_steering()

    # ----------------------- cleanup -----------------------
    def shutdown(self):
        try:
            self.hard_stop()
            self.center_steering()
            time.sleep(0.2)
            # Hardware channels keep pulsing after the process exits unless
            # told otherwise -- the PWM block runs on its own. Stop them so a
            # stopped service really does leave the servos limp.
            for sv in self.servos:
                if sv.hardware:
                    sv.pwm.stop()
            GPIO.cleanup()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
