#!/usr/bin/env python3
"""
DIREKTANG FRONT-STEERING SERVO TEST  (NO ROS, NO sim)
=====================================================
Front steering servo sa GPIO 18 (parehong setup ng motor_node).
Iikutin: CENTER -> FULL LEFT -> CENTER -> FULL RIGHT -> CENTER,
hawak bawat posisyon para MAKITA mo, tapos SLOW SWEEP para malaman
kung SAAN sa range ito nag-bbind.

Calibration = pareho ng motor_node:
    CENTER = 8.2 , LEFT = 13.0 , RIGHT = 3.0
    (widened from 12.0/4.0 -- testing more range on the MG996R. Kung
    mas MALAKAS/mas mahirap ang buzz/grind sa bagong dulo kumpara dati
    -- hindi lang yung parehong faint knock -- ihinto agad, nadikit na
    sa physical limit ng servo iyon, hindi power issue.)

PARAAN NG DIAGNOSIS (importante):
  1) Subukan MUNA na NAKATAAS ang harap (libre ang gulong)
     -> test ng servo MISMO, walang bigat.
  2) Ulitin SA LUPA
     -> para makita kung ang bigat/friction sa daan ang dahilan ng hirap.

⚠️  Patayin muna LAHAT ng ROS terminal (Ctrl+C) -- iisa lang ang
    pwedeng humawak ng GPIO.

Run:
    mamba activate ros2_humble
    python3 ~/steering_test.py
"""

import time
import RPi.GPIO as GPIO

SERVO_PIN = 18
CENTER = 8.2
LEFT = 13.0
RIGHT = 3.0

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(SERVO_PIN, GPIO.OUT)
servo = GPIO.PWM(SERVO_PIN, 50)   # 50 Hz = standard servo
servo.start(CENTER)


def go(label, duty, hold=2.5):
    print(">>> " + label + "   (duty=" + str(duty) + ")")
    servo.ChangeDutyCycle(duty)   # hawak ang signal -> aktibong nag-drive
    time.sleep(hold)


try:
    go("CENTER (start)", CENTER)
    go("FULL LEFT  -- dapat lumiko PAKALIWA ang harap", LEFT)
    go("BALIK CENTER  -- malinis bang bumalik? o nahihirapan?", CENTER)
    go("FULL RIGHT -- dapat lumiko PAKANAN", RIGHT)
    go("BALIK CENTER  -- malinis bang bumalik?", CENTER)

    print(">>> SLOW SWEEP: LEFT -> RIGHT (panoorin kung saan nag-bbind)")
    steps = 40
    for i in range(steps + 1):
        d = LEFT + (RIGHT - LEFT) * (i / steps)
        servo.ChangeDutyCycle(d)
        time.sleep(0.10)

    print(">>> SLOW SWEEP balik: RIGHT -> LEFT")
    for i in range(steps + 1):
        d = RIGHT + (LEFT - RIGHT) * (i / steps)
        servo.ChangeDutyCycle(d)
        time.sleep(0.10)

    go("CENTER (final)", CENTER)
    print(">>> TAPOS. Ano'ng napansin mo?")
finally:
    servo.ChangeDutyCycle(CENTER)
    time.sleep(0.6)
    GPIO.cleanup()
