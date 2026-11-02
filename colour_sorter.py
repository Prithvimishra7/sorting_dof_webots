"""
UR5e Colour Sorting Robot - Webots R2025a

Features:
1. Detect red, green and blue blocks.
2. Wait until the block is stationary.
3. Move to the pick position.
4. Close the gripper and lift.
5. Move to the matching crate.
6. Release the block and return.

Requires: numpy
"""

from controller import Robot
import numpy as np
import math

TIME_STEP = 32

# ---------------- POSITION SETTINGS ----------------

PICK = (0.55, 0.02)

BLOCK_Z = 0.30
GRIP_OFFSET = 0.24
HOVER_HEIGHT = 0.15

CRATES = {
    "red": (0.0, 0.6),
    "green": (-0.6, 0.0),
    "blue": (0.0, -0.6),
}

CARRY_Z = 0.58

# ---------------- GRIPPER SETTINGS ----------------

GRIP_CLOSED = 0.9
FINGER_SPEED = 1.5        # rad/s - gentle; full speed slams the block away
MAX_TRIES = 3             # grab attempts per block
RETRY_LOWER = 0.01        # each retry goes 1 cm lower

GRIP_TIME = 1.0
DROP_TIME = 0.6

WRIST_3 = 0.0

# ---------------- CAMERA SETTINGS ----------------

MIN_PIXELS = 40

ROI_X = (14, 46)
ROI_Y = (14, 46)

STILL_TIME = 0.5

# ---------------- MOVEMENT SETTINGS ----------------

SPEED = 2.0               # normal arm speed
SLOW_SPEED = 0.5          # final approach and first part of the lift
APPROACH = 0.05           # slow down for the last 5 cm above the block

# ---------------- UR5e GEOMETRY ----------------

A2 = 0.425
A3 = 0.3922

WRIST_X = 0.0995
SHOULDER_Z = 0.6724
LATERAL = 0.134


def ik(x, y, z):

    r = math.sqrt(max(
        x*x + y*y - LATERAL*LATERAL,
        1e-6
    ))

    pan = (
        math.atan2(y, x)
        - math.atan2(LATERAL, r)
    )

    px = r - WRIST_X
    py = SHOULDER_Z - z

    d = (
        px*px + py*py - A2*A2 - A3*A3
    ) / (2*A2*A3)

    d = max(-1.0, min(1.0, d))

    elbow = math.acos(d)

    lift = (
        math.atan2(py, px)
        - math.atan2(
            A3 * math.sin(elbow),
            A2 + A3 * math.cos(elbow)
        )
    )

    wrist1 = -(lift + elbow) - math.pi/2

    return [
        pan,
        lift,
        elbow,
        wrist1,
        -math.pi/2,
        WRIST_3
    ]


# ---------------- ROBOT INITIALIZATION ----------------

robot = Robot()

joint_names = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
]

motors = []
sensors = []

for name in joint_names:

    motor = robot.getDevice(name)
    sensor = robot.getDevice(name + "_sensor")

    motor.setVelocity(SPEED)
    sensor.enable(TIME_STEP)

    motors.append(motor)
    sensors.append(sensor)


# ---------------- GRIPPER INITIALIZATION ----------------

finger_names = [
    "finger_1_joint_1",
    "finger_2_joint_1",
    "finger_middle_joint_1"
]

fingers = []
finger_sensors = []

for name in finger_names:

    finger = robot.getDevice(name)
    finger.setVelocity(FINGER_SPEED)

    finger_sensor = robot.getDevice(name + "_sensor")
    finger_sensor.enable(TIME_STEP)

    fingers.append(finger)
    finger_sensors.append(finger_sensor)

    print(
        name,
        "Min:", finger.getMinPosition(),
        "Max:", finger.getMaxPosition()
    )


# ---------------- CAMERA INITIALIZATION ----------------

camera = robot.getDevice("camera")
camera.enable(TIME_STEP)

W = camera.getWidth()
H = camera.getHeight()


# ---------------- HELPER FUNCTIONS ----------------

def wait(seconds):

    end = robot.getTime() + seconds

    while robot.step(TIME_STEP) != -1:

        if robot.getTime() >= end:
            break


def move_to(x, y, z, timeout=8.0):

    target = ik(x, y, z)

    for motor, angle in zip(motors, target):
        motor.setPosition(angle)

    end = robot.getTime() + timeout

    while robot.step(TIME_STEP) != -1:

        errors = [
            abs(sensor.getValue() - angle)
            for sensor, angle in zip(sensors, target)
        ]

        if max(errors) < 0.025:
            return True

        if robot.getTime() > end:
            print("Movement timeout:", x, y, z)
            return False

    return False


def set_speed(speed):

    for motor in motors:
        motor.setVelocity(speed)


def finger_angles():

    return [s.getValue() for s in finger_sensors]


def close_and_settle(timeout=2.0):
    """Close the fingers and wait until they stop moving
    (they stop early when they press on the block)."""

    gripper(True)

    end = robot.getTime() + timeout
    last = finger_angles()
    still_steps = 0

    while robot.step(TIME_STEP) != -1:

        now = finger_angles()
        moved = max(abs(a - b) for a, b in zip(now, last))
        last = now

        still_steps = still_steps + 1 if moved < 0.001 else 0

        if still_steps >= 5 or robot.getTime() > end:
            break

    wait(0.2)   # let the grip force build up


def holding_block():
    """Empty hand: every finger reaches the closed target.
    Holding: at least one finger is stopped early by the block.
    A finger bent backwards (below 0) means it hit something else."""

    angles = finger_angles()
    print("  finger angles:", [round(a, 3) for a in angles])

    if min(angles) < 0.0:
        return False

    return min(angles) < GRIP_CLOSED - 0.03


def gripper(close):

    for finger in fingers:

        if close:
            target = GRIP_CLOSED
        else:
            target = finger.getMinPosition()

        finger.setPosition(target)


def look():

    image = camera.getImage()

    if image is None:
        return None, 0, None

    img = np.frombuffer(
        image, dtype=np.uint8
    ).reshape((H, W, 4))

    # Webots camera image channels: BGRA
    roi = img[
        ROI_Y[0]:ROI_Y[1],
        ROI_X[0]:ROI_X[1]
    ]

    b = roi[:, :, 0].astype(int)
    g = roi[:, :, 1].astype(int)
    r = roi[:, :, 2].astype(int)

    masks = {

        "red": (
            (r > 100) &
            (r > g + 50) &
            (r > b + 50)
        ),

        "green": (
            (g > 100) &
            (g > r + 50) &
            (g > b + 50)
        ),

        "blue": (
            (b > 100) &
            (b > r + 50) &
            (b > g + 50)
        )
    }

    best = max(masks, key=lambda c: masks[c].sum())

    count = int(masks[best].sum())

    if count < MIN_PIXELS:
        return None, 0, None

    ys, xs = np.nonzero(masks[best])

    centre = (
        float(xs.mean()),
        float(ys.mean())
    )

    return best, count, centre


# ---------------- BLOCK DETECTION ----------------

def block_is_still():

    colour, _, centre = look()

    if colour is None:
        return None

    end = robot.getTime() + STILL_TIME

    while robot.step(TIME_STEP) != -1:

        if robot.getTime() >= end:
            return colour

        colour2, _, centre2 = look()

        if colour2 is None:
            return None

        if colour2 != colour:
            return None

        if (
            abs(centre2[0] - centre[0]) > 0.5
            or
            abs(centre2[1] - centre[1]) > 0.5
        ):
            return None

    return None


# ---------------- PICKING SEQUENCE ----------------

hover_z = BLOCK_Z + GRIP_OFFSET + HOVER_HEIGHT
grip_z = BLOCK_Z + GRIP_OFFSET

sorted_count = {
    "red": 0,
    "green": 0,
    "blue": 0
}


def pick_block(colour):

    for attempt in range(1, MAX_TRIES + 1):

        z = grip_z - RETRY_LOWER * (attempt - 1)

        print(f"Attempt {attempt}: moving above block")
        set_speed(SPEED)
        gripper(False)

        if not move_to(PICK[0], PICK[1], hover_z):
            return False

        # Make sure the block is still there before going down
        seen, _, _ = look()
        if seen != colour:
            print("  Block is no longer in the pick spot")
            return False

        # Fast to just above the block, then slow for the last few cm
        if not move_to(PICK[0], PICK[1], z + APPROACH):
            return False

        set_speed(SLOW_SPEED)

        if not move_to(PICK[0], PICK[1], z):
            set_speed(SPEED)
            return False

        wait(0.3)                       # let the arm stop shaking

        print("  Closing gripper")
        close_and_settle()

        if holding_block():

            print("  Block gripped, lifting")
            move_to(PICK[0], PICK[1], z + APPROACH)   # slow first lift
            set_speed(SPEED)

            if not move_to(PICK[0], PICK[1], grip_z + 0.15):
                gripper(False)
                return False

            if not holding_block():
                print("  Block slipped while lifting")
                gripper(False)
                continue

            return True

        print("  Missed - opening and trying again")
        gripper(False)
        wait(0.5)
        set_speed(SPEED)

    return False


# ---------------- DROPPING SEQUENCE ----------------

def drop_block(colour):

    cx, cy = CRATES[colour]

    print("Moving to", colour, "crate")

    if not move_to(cx, cy, CARRY_Z):
        return False

    wait(0.3)

    print("Opening gripper")

    gripper(False)

    wait(DROP_TIME)

    return True


# ---------------- MAIN PROGRAM ----------------

print("UR5e Colour Sorting System Started")

gripper(False)

wait(0.5)

move_to(
    PICK[0],
    PICK[1],
    hover_z
)

print("Waiting for blocks...")


while robot.step(TIME_STEP) != -1:

    colour = block_is_still()

    if colour is None:
        continue

    _, count, _ = look()

    print(
        "Detected:",
        colour.upper(),
        "Pixels:",
        count
    )

    success = pick_block(colour)

    if not success:
        print("Picking failed. Returning to waiting position.")
        gripper(False)
        move_to(PICK[0], PICK[1], hover_z)
        continue

    success = drop_block(colour)

    if not success:
        print("Drop movement failed.")
        continue

    sorted_count[colour] += 1

    print(
        "Sorted count:",
        sorted_count
    )

    print("Returning to pick position")

    move_to(
        PICK[0],
        PICK[1],
        hover_z
    )

    wait(0.5)

