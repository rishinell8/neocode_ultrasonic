# neocode_ultrasonic
Obstacle-Avoiding Robot Car (Raspberry Pi)
A 4-wheel-drive robot car that drives around and steers itself away from obstacles using an ultrasonic sensor mounted on a pan servo. Built on a Raspberry Pi with two L298N motor drivers.

This README explains how the software thinks.
What it does
The car drives forward continuously. While driving, it constantly checks the road ahead with an ultrasonic sensor that sweeps left-center-right instead of only looking straight ahead — so it notices obstacles coming up on an angle, not just ones dead in front. When something gets closer than 30 cm, the car stops, beeps, backs up, looks both ways to see which side has more open space, and pivots toward the clearer side until the path ahead is safe again. Then it resumes driving forward and scanning.

Hardware overview
Part
Role
Raspberry Pi
Brains — runs the Python control loop
2x L298N motor driver
Each drives one axle (front pair / rear pair) via 2 logic pins per wheel
4x DC gear motors
One per wheel, powered at 7.5V
HC-SR04 ultrasonic sensor
Measures distance to obstacles (TRIG/ECHO)
Micro servo
Pans the ultrasonic sensor left/right/center
Active buzzer
Beeps when an obstacle is detected


Motor power (7.5V) and Pi/logic power (5V) are separate supplies sharing a common ground — see the PDF's power diagram for the full wiring. The HC-SR04's ECHO line outputs 5V but the Pi's GPIO only tolerates 3.3V, so that line needs a voltage divider or level shifter — this is called out directly in the code as something people forget.
GPIO pin map (BCM numbering)
Wheel
Pin A
Pin B
Back-Right
GPIO 17
GPIO 27
Back-Left
GPIO 5
GPIO 6
Front-Right
GPIO 13
GPIO 19
Front-Left
GPIO 20
GPIO 21


Device
Pin
Direction
HC-SR04 TRIG
GPIO 23
Pi → sensor
HC-SR04 ECHO
GPIO 24
sensor → Pi (needs level shift)
Pan servo
GPIO 18
Pi → servo (PWM, 50Hz)
Buzzer
GPIO 22
Pi → buzzer (PWM tone)


Each wheel is one channel of an H-bridge: two GPIO pins per motor, where one pin is PWM'd to set speed/direction and the other sits at 0. Which two pins are "active" and at what pattern determines forward, backward, or a spin-in-place turn.
How the software is organized
The code is one self-contained file, merged from two earlier prototypes: a wheel-driving script and a separate obstacle-avoidance script. Everything now lives together so there's a single source of truth for pin numbers and behavior.
Driving primitives
Four basic motions are built from the wheel pin patterns:

Forward — all four wheels turn the same way.
Backward — the opposite pin pattern on all four wheels.
Turn left / Turn right — a "tank turn": one side's wheels drive forward while the other side's wheels drive backward, spinning the car in place rather than arcing.

Motors won't reliably start moving at low power (they need a stronger nudge to overcome static friction than they need to keep rolling once moving). So whenever the car starts a new kind of motion, it briefly pulses the motors at full power for a fraction of a second, then eases back to the normal cruising duty cycle. Repeating the same motion — like continuing to drive forward — skips that extra kick.
Looking around: the ultrasonic + servo combo
The distance sensor works by sending an ultrasonic pulse and timing how long the echo takes to return; the round-trip time converts directly to distance. A single reading can be noisy (a stray reflection, a timeout), so the code takes several readings in a row and uses the middle (median) value rather than trusting any one ping.

The servo lets that sensor look in different directions without moving the whole car. Instead of snapping straight to a target angle, the servo moves there in small steps — gentler on the hardware and less likely to overshoot.
Scanning while driving
Rather than only checking the distance straight ahead, the car sweeps the sensor across a small forward cone (three angles: left-of-center, straight ahead, right-of-center) while continuing to drive. Whatever the closest reading across that sweep is becomes "the" distance for deciding whether to react. This way, an obstacle that's ahead-and-off-to-one-side gets caught just as reliably as one directly in front.
Reacting to an obstacle
When the closest distance in that sweep drops below the safety threshold (30 cm), the car runs its recovery routine:

Stop and beep, so it's clear (audibly) that something was detected.
Back away briefly to create room to maneuver.
Look left, then look right, and compare which direction is more open.
Start turning toward whichever side had more room, backing away a little more with each turning step.
After each turning step, look again — specifically toward the direction it's turning into (not just straight ahead) — to check whether that path has actually cleared.
Keep repeating (back up a bit, turn a bit, re-check) until that side reads clear, up to a maximum turning time, so it doesn't spin forever if it happens to have picked a bad direction or a sensor reading is off.

Once the pivot is done, the sensor re-centers and the car resumes its normal forward-driving scan loop.
Safety and cleanup
A 60-second failsafe stops the whole car automatically, independent of the obstacle-avoidance logic, as a hard ceiling on unattended runtime.
Ctrl+C and process termination are both handled the same way — even a kill signal is converted into the same clean shutdown path as pressing Ctrl+C, so the motors always get explicitly cut and the GPIO pins released, rather than the car being left running (or worse, one motor stuck energized) if the process dies unexpectedly.
Built-in test modes
Two diagnostic modes are included, meant to be run before trusting the car to drive on its own:

Servo/sensor self-test — parks the sensor at center, then sweeps it right → center → left (and back), printing the distance reading at each stop, with the wheels never driven. This confirms the servo horn is mounted straight and that turning the sensor toward an obstacle actually changes the reading on the correct side, before any risk of the car moving.
Motor ramp test — pulses the wheels forward at increasing power levels (e.g., 40% up to 100%) to find the lowest power that actually gets the wheels turning from a standing stop. This value becomes a useful baseline for tuning the cruising and kickstart power levels.
Running it
python3 robotcar_standalone.py              # normal obstacle-avoidance run

python3 robotcar_standalone.py --selftest   # servo + sensor check, wheels never move

python3 robotcar_standalone.py --motortest  # find minimum power to start the wheels
Tuning notes
A few constants at the top of the file are the main levers if you build your own copy:

SAFE_DISTANCE_CM — how close is "too close" before it reacts.
MOTOR_DUTY_PERCENT / KICKSTART_* — cruising speed vs. the brief full-power kick used to get wheels moving from rest.
DRIVE_SCAN_ANGLES — how wide a cone the sensor sweeps while driving; narrow it if the car reacts too eagerly to side walls.
MAX_PIVOT_SECONDS / MIN_PIVOT_STEPS — bounds on how long/short a recovery turn can be.


