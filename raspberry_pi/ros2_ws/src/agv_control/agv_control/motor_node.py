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
  * Command watchdog: if no /cmd_vel arrives within CMD_TIMEOUT while moving
    (a publisher died / app disconnected), the motors are stopped automatically.
  * SPEED_SCALE caps the top speed (client request: slower, safer pace).
  * REVERSE GUARD: on a direction flip (forward <-> reverse) the motors are
    held at ZERO for REVERSE_PAUSE seconds first -- a true full stop before
    reversing, so the rear gears are never slammed (client request).
"""

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import RPi.GPIO as GPIO   # provided by rpi-lgpio on the Pi 5

# ------------------------- GPIO PINS -------------------------
SERVO_PIN = 18                       # front steering servo
LEFT_FWD = 12;  RIGHT_FWD = 13       # rear drive (forward)
LEFT_BWD = 22;  RIGHT_BWD = 23       # rear drive (backward)

# ------------- STEERING CALIBRATION (measured, no gear skip) -------------
# Widened from the original 4.0/12.0 (client request, testing more range on
# the swapped-in MG996R). CENTER unchanged -- that's the measured true-
# straight position, not something to guess at. This still sits close to
# the ~0.5-2.5ms standard hobby-servo pulse range (duty% * 20ms @ 50Hz), but
# if the servo ever strains/grinds harder at the new extremes (vs. just not
# moving) than before, back these off immediately -- that's the mechanical
# end-stop, not a power issue.
STEER_CENTER = 8.2                   # true straight
STEER_LEFT   = 13.0                  # angular.z = +1.0  (max safe left)
STEER_RIGHT  = 3.0                   # angular.z = -1.0  (max safe right)

# ------------------------- SAFETY / TUNING -------------------------
CMD_TIMEOUT = 0.7                    # s; stop if no /cmd_vel within this window
MAX_DUTY    = 100.0                  # absolute cap on motor PWM %

# Client request #2: slower, safer pace.
# Top speed = SPEED_SCALE * 100% duty. 0.60 = 60% max.
# Tune: 0.50 kung gusto pang mas mabagal; iwas masyadong baba (< ~0.40)
# baka hindi gumalaw ang plain DC motors sa bigat (stall).
SPEED_SCALE = 0.60

# Client request #3: full stop bago mag-reverse (gear protection).
# Kapag nag-flip ang direction habang gumagalaw, i-hohold muna sa ZERO ang
# motors na xxng ganito katagal bago i-apply ang kabilang direction.
REVERSE_PAUSE = 0.4                  # s; taasan (0.6) kung gusto ng mas luwag


class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')

        # --- GPIO setup ---
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        self.steering = GPIO.PWM(SERVO_PIN, 50)
        self.steering.start(STEER_CENTER)
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

        self.get_logger().info(
            f"Motor node ready (GPIO owner). Mode: STOP | "
            f"speed cap {int(SPEED_SCALE * 100)}% | reverse pause {REVERSE_PAUSE}s")

    # ----------------------- helpers -----------------------
    def steer(self, angular_z):
        """Map angular.z (-1..+1, + = left) to a calibrated servo duty."""
        az = max(-1.0, min(1.0, angular_z))
        if az >= 0:                                   # left half
            duty = STEER_CENTER + az * (STEER_LEFT - STEER_CENTER)
        else:                                         # right half
            duty = STEER_CENTER + az * (STEER_CENTER - STEER_RIGHT)
        duty = max(STEER_RIGHT, min(STEER_LEFT, duty))
        self.steering.ChangeDutyCycle(duty)

    def _apply_duty(self, fwd, bwd):
        self.l_fwd.ChangeDutyCycle(fwd); self.r_fwd.ChangeDutyCycle(fwd)
        self.l_bwd.ChangeDutyCycle(bwd); self.r_bwd.ChangeDutyCycle(bwd)
        self.last_drive = (fwd, bwd)

    def drive(self, linear_x):
        """Map linear.x (-1..+1) to rear-motor PWM.

        Speed is capped by SPEED_SCALE, and a direction flip goes through a
        mandatory REVERSE_PAUSE at zero duty (full stop) to protect the gears.
        """
        lx = max(-1.0, min(1.0, linear_x))
        desired = 1 if lx > 0.02 else (-1 if lx < -0.02 else 0)
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
            self.steering.ChangeDutyCycle(STEER_CENTER)

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
                self.steering.ChangeDutyCycle(STEER_CENTER)

    # ----------------------- cleanup -----------------------
    def shutdown(self):
        try:
            self.hard_stop()
            self.steering.ChangeDutyCycle(STEER_CENTER)
            time.sleep(0.2)
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
