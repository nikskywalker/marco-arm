"""Serial interface and calibration of the Marco arm.

The Arduino runs firmware/pca_serial: it receives lines "channel angle" (servo command in degrees),
moves that joint with its speed ramp and answers "Joint: <channel>  Angle: <angle>" when the move
is over, or "ERROR!" if the angle is outside the joint's range.

Opening the serial port resets the Arduino: the arm goes to its neutral pose and the firmware
prints "Ready: all joints at neutral." Only one program can use the port at a time.
"""
import serial

BAUD = 9600
DEFAULT_PORT = "COM3"

# PCA9685 channel of each joint.
CHANNELS = {"base": 0, "shoulder": 1, "elbow": 2, "wrist": 3, "gripper": 4}

# Servo command range accepted by the firmware (ANGLE_MIN / ANGLE_MAX in pca_serial), degrees.
LIMITS = {"base": (45, 135), "shoulder": (10, 170), "elbow": (10, 170), "wrist": (40, 110), "gripper": (30, 150)}

# Neutral pose set by the firmware at power-up (NEUTRAL in pca_serial), servo commands.
NEUTRAL = {"base": 90, "shoulder": 130, "elbow": 90, "wrist": 90, "gripper": 90}

# Calibration measured on the arm: real joint angle = slope * command + offset (degrees).
#   base      yaw, counterclockwise seen from above, 0 at command 90
#   shoulder  link 1 from the table
#   elbow     forearm from link 1 (0 = in line), valid with the forearm in front of vertical
#   wrist     gripper from forearm (0 = in line), about +-2 deg
CALIBRATION = {
    "base": (0.754, -67.86),
    "shoulder": (0.865, -30.1),
    "elbow": (0.948, -89.3),
    "wrist": (-1.00, 83.6),
}


def command_to_angle(joint, command):
    """Real angle (deg) of a joint for a servo command (deg), from the calibration."""
    slope, offset = CALIBRATION[joint]
    return slope * command + offset


def angle_to_command(joint, angle):
    """Servo command (whole degrees) that puts a joint at a real angle (deg).

    Raises ValueError if the command falls outside the joint's range.
    """
    slope, offset = CALIBRATION[joint]
    command = round((angle - offset) / slope)
    low, high = LIMITS[joint]
    if not low <= command <= high:
        raise ValueError(f"{joint}: {angle:.1f} deg needs command {command}, outside {low}-{high}")
    return command


def open_arm(port=DEFAULT_PORT, timeout=5):
    """Open the serial port, wait for the firmware's "Ready" line and return the port."""
    arm = serial.Serial(port=port, baudrate=BAUD, timeout=timeout)
    ready = arm.readline().decode().strip()
    if not ready.startswith("Ready"):
        arm.close()
        raise RuntimeError(f"no 'Ready' line from the Arduino on {port} (got {ready!r})")
    return arm


def move_joint(arm, channel, command):
    """Send one servo command and return the firmware's reply (it arrives when the move is over).

    arm: the port returned by open_arm. channel: 0-4. command: servo angle in whole degrees.
    """
    arm.write(f"{channel} {command}\n".encode())
    return arm.readline().decode().strip()
