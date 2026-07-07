"""Standalone ultrasonic obstacle-avoiding robot car for Raspberry Pi.

Self-contained merge of:
  - the wheel-driving pin logic from robotcar1.py (functions a/b/c/d/e)
  - the obstacle-avoidance logic from robotcar-ultrasonic-OA.py

Wiring (BCM numbering):
    Back-right wheel  -> GPIO 17, 27
    Back-left wheel   -> GPIO 5, 6
    Front-right wheel -> GPIO 13, 19
    Front-left wheel  -> GPIO 20, 21

    HC-SR04 TRIG  -> GPIO 23
    HC-SR04 ECHO  -> GPIO 24  (voltage-divide or level-shift! Echo is 5V,
                               the Pi's GPIO inputs are only 3.3V-tolerant)
    Pan servo     -> GPIO 18
    Buzzer        -> GPIO 22  (active buzzer; HIGH = beep)

Note: robotcar1.py's original comments disagree with its own w/a/s/d/x key
bindings about which of c()/d() is "left" vs "right" (trailing comments say
c()=right, d()=left; the key bindings use a()=w, b()=s, c()='a' key,
d()='d' key). This file follows the key bindings (turn_left=c(), turn_right
=d()) since that's the scheme that was actually driven/tested. If the car
pivots the wrong way when avoiding an obstacle, swap the bodies of
turn_left() and turn_right() below.
"""

import signal
import sys
import time

import RPi.GPIO as GPIO

# --- Wheel pins (BCM), from robotcar1.py's a()/b()/c()/d()/e() -------------
BACK_RIGHT_A, BACK_RIGHT_B = 17, 27
BACK_LEFT_A, BACK_LEFT_B = 5, 6
FRONT_RIGHT_A, FRONT_RIGHT_B = 13, 19
FRONT_LEFT_A, FRONT_LEFT_B = 20, 21

WHEEL_PINS = (
    BACK_RIGHT_A, BACK_RIGHT_B,
    BACK_LEFT_A, BACK_LEFT_B,
    FRONT_RIGHT_A, FRONT_RIGHT_B,
    FRONT_LEFT_A, FRONT_LEFT_B,
)

# --- Sensor/actuator pins (BCM), from robotcar-ultrasonic-OA.py ------------
TRIG_PIN = 23
ECHO_PIN = 24
SERVO_PIN = 18
BUZZER_PIN = 22

# --- Speed limiting (temporary, while verifying obstacle avoidance) --------
# The wheel pins were originally driven full on/off. Driving the "active" pin
# with PWM at this duty instead lets us run the whole car slow while we confirm
# the avoidance logic is behaving. Bump back toward 100 once it's trusted.
MOTOR_DUTY_PERCENT = 100
MOTOR_PWM_HZ = 100  # software-PWM frequency for the H-bridge inputs

# The motors won't turn from rest at the 60% cruise duty (measured floor is
# ~70-80%), but they cruise fine there once moving. So each time a wheel motion
# *starts* or changes direction, briefly pulse the active pins at full power to
# break static friction, then settle to the cruise duty. Steady-state repeat
# calls for the same motion don't re-kick. Lengthen KICKSTART_SECONDS if it
# still doesn't reliably start; shorten it if the kick makes it lurch.
KICKSTART_DUTY_PERCENT = 100
KICKSTART_SECONDS = 0.15

# --motortest ramps forward through these duties to find the lowest one that
# actually turns the wheels from rest (the practical floor for MOTOR_DUTY_PERCENT).
MOTOR_TEST_DUTIES = (40, 50, 60, 70, 80, 90, 100)
MOTOR_TEST_STEP_SECONDS = 0.6

# A positional servo has no speed setting, so we can't PWM it slower. Instead we
# slew it toward the target a few degrees at a time -- gentler on a loose/off-
# center horn and less likely to overshoot than snapping straight to the angle.
SERVO_SLEW_DEG_PER_STEP = 5
SERVO_SLEW_STEP_SECONDS = 0.02

# --- Behavior tunables (from robotcar-ultrasonic-OA.py) --------------------
SAFE_DISTANCE_CM = 30  # stop and avoid when closer than this
TURN_ANGLE_LEFT_DEG = 150
TURN_ANGLE_CENTER_DEG = 90
TURN_ANGLE_RIGHT_DEG = 30

# While driving, sweep the sensor across this forward cone instead of only
# looking dead-ahead, so obstacles that are ahead-and-to-the-side get seen.
# Narrow these toward 90 if it reacts to side walls too eagerly.
DRIVE_SCAN_ANGLES = (55, 90, 125)
DRIVE_SCAN_SAMPLES = 3  # pings per angle during the driving sweep (lighter than the full median)

REVERSE_SECONDS = 0.5  # initial back-up when an obstacle is hit
REVERSE_STEP_SECONDS = 0.2  # extra backing-up done before each pivot step, not just once up front
PIVOT_STEP_SECONDS = 0.35  # how long each incremental pivot lasts before rechecking distance
MAX_PIVOT_SECONDS = 3.5  # give up turning (and re-pick a direction) after this much total pivot time
MIN_PIVOT_STEPS = 3  # always turn at least this many steps, even if a reading looks clear early
STOP_BEEP_SECONDS = 0.5
SERVO_SETTLE_SECONDS = 0.15
LOOP_DELAY_SECONDS = 0.02

ULTRASONIC_TIMEOUT_SECONDS = 0.02  # ~20000us, matches the ESP32 pulseIn timeout
ULTRASONIC_SAMPLE_COUNT = 5
ULTRASONIC_SAMPLE_DELAY_SECONDS = 0.008

MAX_RUN_SECONDS = 60  # failsafe: stop the car automatically after this long

BUZZER_TONE_HZ = 1000  # this is a passive buzzer -- needs a PWM tone, not steady HIGH

servo_pwm = None  # set up in setup_gpio()
buzzer_pwm = None  # set up in setup_gpio()
wheel_pwms = {}  # pin -> PWM object, set up in setup_gpio()
_last_servo_angle = TURN_ANGLE_CENTER_DEG  # tracked so aim_servo() can slew gently
_current_active = frozenset()  # wheel pins currently driven (empty = stopped); gates the kickstart


def setup_gpio():
    global servo_pwm, buzzer_pwm
    GPIO.setmode(GPIO.BCM)

    for pin in WHEEL_PINS:
        GPIO.setup(pin, GPIO.OUT)
        pwm = GPIO.PWM(pin, MOTOR_PWM_HZ)
        pwm.start(0)
        wheel_pwms[pin] = pwm

    GPIO.setup(TRIG_PIN, GPIO.OUT)
    GPIO.setup(ECHO_PIN, GPIO.IN)
    GPIO.setup(BUZZER_PIN, GPIO.OUT)
    GPIO.output(TRIG_PIN, GPIO.LOW)
    GPIO.output(BUZZER_PIN, GPIO.LOW)
    buzzer_pwm = GPIO.PWM(BUZZER_PIN, BUZZER_TONE_HZ)
    buzzer_pwm.start(0)

    GPIO.setup(SERVO_PIN, GPIO.OUT)
    servo_pwm = GPIO.PWM(SERVO_PIN, 50)  # 50Hz, standard hobby servo rate
    servo_pwm.start(0)


def _shutdown():
    """Stop every PWM channel and release the GPIO. Drops all references to the
    PWM objects *before* GPIO.cleanup() so their finalizers run while the lgpio
    backend is still alive -- otherwise RPi.GPIO spews a stack of 'Exception
    ignored in: PWM.__del__ ... NoneType & int' tracebacks when the objects are
    garbage-collected at interpreter exit, after the chip handle is already gone."""
    global servo_pwm, buzzer_pwm
    stop_all()
    pwms = list(wheel_pwms.values()) + [p for p in (servo_pwm, buzzer_pwm) if p is not None]
    for pwm in pwms:
        pwm.stop()
    wheel_pwms.clear()
    servo_pwm = buzzer_pwm = None
    pwms.clear()
    pwm = None  # drop the loop variable's lingering reference too
    GPIO.cleanup()


# --- Wheel control (same pin patterns as robotcar1.py's a/b/c/d/e, but the
# "HIGH" pin of each motor is now PWM'd at MOTOR_DUTY_PERCENT so we can run the
# car at reduced speed; the opposite pin sits at 0 to set direction) ---------

FORWARD_PINS = (BACK_RIGHT_B, BACK_LEFT_B, FRONT_RIGHT_A, FRONT_LEFT_B)


def _set_wheels(active, duty):
    for pin in WHEEL_PINS:
        wheel_pwms[pin].ChangeDutyCycle(duty if pin in active else 0)


def _drive(*active_pins, duty=MOTOR_DUTY_PERCENT, kick=True):
    """Drive the given pins at `duty` percent, holding every other wheel pin at
    0 (one PWM'd input + one grounded input per motor sets its direction and
    speed on the H-bridge). When this *starts* a new motion -- different active
    pins than are already running -- first pulse them at KICKSTART_DUTY_PERCENT
    for KICKSTART_SECONDS to break static friction, since the motors won't turn
    from rest at the low cruise duty. Repeat calls for the same motion skip the
    kick and just hold the cruise duty. Pass kick=False to drive at exactly
    `duty` from rest (used by the ramp test)."""
    global _current_active
    active = frozenset(active_pins)
    if kick and active and active != _current_active:
        _set_wheels(active, KICKSTART_DUTY_PERCENT)
        time.sleep(KICKSTART_SECONDS)
    _set_wheels(active, duty)
    _current_active = active


def drive_forward():  # robotcar1.py: a()
    _drive(*FORWARD_PINS)


def drive_backward():  # robotcar1.py: b()
    _drive(BACK_RIGHT_A, BACK_LEFT_A, FRONT_RIGHT_B, FRONT_LEFT_A)


def turn_left():  # proper 4-wheel spin: right side backward + left side forward
    # Right side backward (robotcar1.py's original c()) + left side forward.
    _drive(FRONT_RIGHT_B, BACK_RIGHT_A, FRONT_LEFT_B, BACK_LEFT_B)


def turn_right():  # proper 4-wheel spin: left side backward + right side forward
    # Left side backward (robotcar1.py's original d()) + right side forward.
    _drive(FRONT_LEFT_A, BACK_LEFT_A, FRONT_RIGHT_A, BACK_RIGHT_B)


def stop_all():  # robotcar1.py: e()
    global _current_active
    for pin in WHEEL_PINS:
        wheel_pwms[pin].ChangeDutyCycle(0)
    _current_active = frozenset()  # next motion counts as a fresh start -> re-kick


# --- Buzzer / ultrasonic / servo (from robotcar-ultrasonic-OA.py) ----------

def beep(duration_seconds=STOP_BEEP_SECONDS):
    buzzer_pwm.ChangeDutyCycle(50)
    time.sleep(duration_seconds)
    buzzer_pwm.ChangeDutyCycle(0)


def single_ping_cm(timeout_seconds=ULTRASONIC_TIMEOUT_SECONDS):
    """One HC-SR04 ping. Returns distance in cm, or None on timeout (no echo)."""
    GPIO.output(TRIG_PIN, GPIO.LOW)
    time.sleep(0.000002)
    GPIO.output(TRIG_PIN, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, GPIO.LOW)

    wait_start = time.monotonic()
    while GPIO.input(ECHO_PIN) == GPIO.LOW:
        if time.monotonic() - wait_start > timeout_seconds:
            return None
    pulse_start = time.monotonic()

    while GPIO.input(ECHO_PIN) == GPIO.HIGH:
        if time.monotonic() - pulse_start > timeout_seconds:
            return None
    pulse_end = time.monotonic()

    duration_us = (pulse_end - pulse_start) * 1_000_000
    return duration_us / 58.0  # HC-SR04 datasheet: cm = us / 58


def measure_distance_cm(sample_count=ULTRASONIC_SAMPLE_COUNT):
    """Median of several pings to reject outliers, same approach as the ESP32 code."""
    readings = []
    for _ in range(sample_count):
        reading = single_ping_cm()
        readings.append(reading if reading is not None else 9999)
        time.sleep(ULTRASONIC_SAMPLE_DELAY_SECONDS)
    readings.sort()
    return readings[len(readings) // 2]


def _servo_duty(angle_degrees):
    return 2.5 + (angle_degrees / 180.0) * 10.0  # 0.5ms-2.5ms pulse at 50Hz


def aim_servo(angle_degrees):
    """Slew gently to the target angle a few degrees at a time instead of
    snapping to it -- easier on a loose/off-center horn and avoids overshoot."""
    global _last_servo_angle
    span = angle_degrees - _last_servo_angle
    steps = max(1, int(abs(span) / SERVO_SLEW_DEG_PER_STEP))
    for i in range(1, steps + 1):
        intermediate = _last_servo_angle + span * (i / steps)
        servo_pwm.ChangeDutyCycle(_servo_duty(intermediate))
        time.sleep(SERVO_SLEW_STEP_SECONDS)
    _last_servo_angle = angle_degrees
    time.sleep(SERVO_SETTLE_SECONDS)


def scan_forward_cone():
    """Sweep the sensor across DRIVE_SCAN_ANGLES (a forward cone) and return
    the CLOSEST distance seen across all of them. The wheels keep whatever
    drive state they were in, so the car keeps moving during the sweep.
    This is what lets it notice obstacles that are ahead-and-to-the-side,
    not just the one dead in front of the sensor."""
    closest_cm = 9999
    for angle in DRIVE_SCAN_ANGLES:
        aim_servo(angle)
        reading = measure_distance_cm(DRIVE_SCAN_SAMPLES)
        if reading < closest_cm:
            closest_cm = reading
    return closest_cm


def choose_best_turn():
    """Look left and right; return True to turn left, False to turn right."""
    aim_servo(TURN_ANGLE_LEFT_DEG)
    left_distance = measure_distance_cm()
    aim_servo(TURN_ANGLE_RIGHT_DEG)
    right_distance = measure_distance_cm()
    aim_servo(TURN_ANGLE_CENTER_DEG)
    return left_distance >= right_distance


def back_up_and_turn():
    """Recovery move when an obstacle is detected: stop, beep, back up,
    then repeatedly back up a little more and pivot toward whichever side
    has more room -- aiming the ultrasonic at that same side (not straight
    ahead) after each step, so it's actually checking the direction it's
    turning into instead of only ever looking dead ahead. Keeps doing this,
    for at least MIN_PIVOT_STEPS, until that side reads clear or
    MAX_PIVOT_SECONDS of total pivoting is used up."""
    stop_all()
    beep()
    drive_forward()  # swapped: this project's "backward" pin pattern is actually forward
    time.sleep(REVERSE_SECONDS)
    stop_all()

    turning_left = choose_best_turn()
    turn_step = turn_left if turning_left else turn_right
    watch_angle_deg = TURN_ANGLE_LEFT_DEG if turning_left else TURN_ANGLE_RIGHT_DEG

    pivoted_seconds = 0.0
    pivot_steps = 0
    while pivoted_seconds < MAX_PIVOT_SECONDS:
        drive_forward()  # keep backing away while turning, not just once at the start
        time.sleep(REVERSE_STEP_SECONDS)
        stop_all()

        turn_step()
        time.sleep(PIVOT_STEP_SECONDS)
        stop_all()
        pivoted_seconds += PIVOT_STEP_SECONDS
        pivot_steps += 1

        aim_servo(watch_angle_deg)  # look toward the side we're turning into
        side_is_clear = measure_distance_cm() > SAFE_DISTANCE_CM
        if pivot_steps >= MIN_PIVOT_STEPS and side_is_clear:
            break

    aim_servo(TURN_ANGLE_CENTER_DEG)  # re-center before resuming normal forward-facing scans


def run_obstacle_avoidance():
    print("Ultrasonic obstacle avoidance running. Ctrl+C to stop.")
    run_start = time.monotonic()
    while True:
        if time.monotonic() - run_start > MAX_RUN_SECONDS:
            print(f"Failsafe: {MAX_RUN_SECONDS}s runtime limit reached, stopping.")
            return

        drive_backward()  # swapped: this project's "forward" pin pattern is actually forward
        closest_cm = scan_forward_cone()  # keeps driving while sweeping the forward cone
        if closest_cm <= SAFE_DISTANCE_CM:
            print(f"Obstacle at {closest_cm:.0f} cm")
            back_up_and_turn()

        time.sleep(LOOP_DELAY_SECONDS)


def servo_selftest(passes=2):
    """Center + sweep test with the WHEELS NEVER DRIVEN, to check the servo horn
    is centered and the ultrasonic reads sanely before letting the car move.

    Parks at center, then sweeps right(30) -> center(90) -> left(150) and back,
    printing the distance measured at each stop. Use it to confirm:
      - the horn physically points straight ahead when parked at center, and
      - putting an obstacle on one side moves the reading at that side, not the
        opposite one (which would mean the horn is mounted backwards/off).
    """
    labels = (
        ("RIGHT ", TURN_ANGLE_RIGHT_DEG),
        ("CENTER", TURN_ANGLE_CENTER_DEG),
        ("LEFT  ", TURN_ANGLE_LEFT_DEG),
    )
    setup_gpio()
    stop_all()  # belt and suspenders: motors stay at 0 duty for the whole test
    try:
        aim_servo(TURN_ANGLE_CENTER_DEG)
        print("Servo parked at CENTER. Confirm the sensor points straight aheafd.")
        beep(0.1)
        for pass_num in range(1, passes + 1):
            print(f"--- sweep pass {pass_num}/{passes} ---")
            sweep = labels if pass_num % 2 else tuple(reversed(labels))
            for name, angle in sweep:
                aim_servo(angle)
                distance = measure_distance_cm()
                bar = "#" * min(40, int(distance)) if distance < 9999 else "(no echo)"
                print(f"  {name} @ {angle:3d} deg: {distance:6.1f} cm  {bar}")
        aim_servo(TURN_ANGLE_CENTER_DEG)
        print("Self-test done. Motors were never driven.")
    finally:
        _shutdown()


def motor_ramp_test():
    """Pulse forward at each duty in MOTOR_TEST_DUTIES to find the lowest one
    that turns the wheels from rest. Watch the car and note the first level
    where it actually moves. Give it room to roll, or stand it up so the wheels
    spin free. Wheels only ever go forward here, in short pulses."""
    setup_gpio()
    signal.signal(signal.SIGTERM, _request_stop)
    try:
        for duty in MOTOR_TEST_DUTIES:
            print(f"forward @ {duty}% duty", flush=True)
            _drive(*FORWARD_PINS, duty=duty, kick=False)  # no kick: measure the true from-rest floor
            time.sleep(MOTOR_TEST_STEP_SECONDS)
            stop_all()
            time.sleep(0.4)
        print("Motor ramp test done.", flush=True)
    except KeyboardInterrupt:
        print("Stop requested; cutting motors and cleaning up.", flush=True)
    finally:
        _shutdown()


def _request_stop(signum, frame):
    # Turn a `kill` (SIGTERM) into a KeyboardInterrupt so the try/finally in
    # main() runs _shutdown() and actually cuts the motors. Without this,
    # Python's default SIGTERM kills the process outright, skipping cleanup and
    # potentially leaving the wheels energized.
    raise KeyboardInterrupt


def main():
    setup_gpio()
    signal.signal(signal.SIGTERM, _request_stop)
    aim_servo(TURN_ANGLE_CENTER_DEG)
    stop_all()
    beep(0.1)
    try:
        run_obstacle_avoidance()
    except KeyboardInterrupt:
        print("Stop requested; cutting motors and cleaning up.")
    finally:
        _shutdown()


if __name__ == '__main__':
    arg = sys.argv[1] if len(sys.argv) > 1 else ''
    if arg in ('--selftest', '--self-test', 'selftest'):
        servo_selftest()
    elif arg in ('--motortest', '--motor-test', 'motortest'):
        motor_ramp_test()
    else:
        main()
