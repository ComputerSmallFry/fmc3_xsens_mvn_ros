"""All tunable parameters for the Xsens-to-GR2 bridge."""

# ---------------------------------------------------------------------------
# Network
# ---------------------------------------------------------------------------
XSENS_UDP_PORT = 9763  # Must be > 2000 (XSensUDPClient enforces this)
XSENS_UDP_HOST = "0.0.0.0"
GR2_DOMAIN_ID = 123

# ---------------------------------------------------------------------------
# Control loop
# ---------------------------------------------------------------------------
CONTROL_RATE_HZ = 50
RAMP_DURATION_SEC = 3.0
SHUTDOWN_RAMP_SEC = 2.0
DATA_TIMEOUT_SEC = 0.2
SMOOTHING_ALPHA = 0.3  # Exponential filter, ~7 Hz cutoff at 50 Hz

# ---------------------------------------------------------------------------
# PD gains (from Fourier demo_walk.py / demo_joint_command.py)
# ---------------------------------------------------------------------------
KP = {
    "waist": [200],
    "head": [100, 100],
    "left_manipulator": [300, 300, 100, 100, 50, 50, 50],
    "right_manipulator": [300, 300, 100, 100, 50, 50, 50],
}

KD = {
    "waist": [10],
    "head": [10, 10],
    "left_manipulator": [10, 10, 5, 5, 5, 5, 5],
    "right_manipulator": [10, 10, 5, 5, 5, 5, 5],
}

# ---------------------------------------------------------------------------
# Per-joint sign multipliers for empirical tuning.
# Order: [shoulder_pitch, shoulder_roll, shoulder_yaw,
#          elbow_pitch, wrist_yaw, wrist_pitch, wrist_roll]
# ---------------------------------------------------------------------------
SIGN_MAP = {
    "waist": [1.0],
    "head": [1.0, 1.0],  # [pan, tilt]
    "left_manipulator": [1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
    "right_manipulator": [1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
}

# ---------------------------------------------------------------------------
# Raw Xsens joint/axis selection for each commanded GR-2 joint.
# Axis index uses the remapped Xsens tuple returned by HumanDataHandler:
#   0 -> x, 1 -> y, 2 -> z
# Order matches SIGN_MAP / JOINT_OFFSETS / JOINT_LIMITS.
# ---------------------------------------------------------------------------
JOINT_SOURCE_MAP = {
    "head": [("c1_head", 2), ("c1_head", 0)],
    "left_manipulator": [
        ("left_shoulder", 0),
        ("left_shoulder", 2),
        ("left_shoulder", 1),
        ("left_elbow", 2),
        ("left_wrist", 2),
        ("left_wrist", 0),
        ("left_wrist", 1),
    ],
    "right_manipulator": [
        ("right_shoulder", 0),
        ("right_shoulder", 2),
        ("right_shoulder", 1),
        ("right_elbow", 2),
        ("right_wrist", 2),
        ("right_wrist", 0),
        ("right_wrist", 1),
    ],
}

# ---------------------------------------------------------------------------
# Manual zero offsets in GR-2 joint space (radians).
# These are subtracted from the mapped command, so if a relaxed neutral pose
# prints `+0.4` for some joint, set the corresponding offset to `+0.4`.
# Order matches SIGN_MAP.
# ---------------------------------------------------------------------------
JOINT_OFFSETS = {
    "waist": [0.0],
    "head": [0.0, 0.0],
    "left_manipulator": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "right_manipulator": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
}

# ---------------------------------------------------------------------------
# Joint limits (radians) from local GR-2 alignment docs.
# Order matches SIGN_MAP.
# ---------------------------------------------------------------------------
JOINT_LIMITS = {
    "waist": [(-2.6180, 2.6180)],
    "head": [(-1.3963, 1.3963), (-0.5236, 0.5236)],
    "left_manipulator": [
        (-2.9671, 2.9671),   # shoulder_pitch
        (-0.5236, 2.7925),   # shoulder_roll
        (-1.8326, 1.8326),   # shoulder_yaw
        (-1.5272, 0.4800),   # elbow_pitch
        (-1.8326, 1.8326),   # wrist_yaw
        (-0.6109, 0.6109),   # wrist_pitch
        (-0.9599, 0.9599),   # wrist_roll
    ],
    "right_manipulator": [
        (-2.9671, 2.9671),   # shoulder_pitch
        (-2.7925, 0.5236),   # shoulder_roll (mirrored)
        (-1.8326, 1.8326),   # shoulder_yaw
        (-1.5272, 0.4800),   # elbow_pitch
        (-1.8326, 1.8326),   # wrist_yaw
        (-0.6109, 0.6109),   # wrist_pitch
        (-0.9599, 0.9599),   # wrist_roll
    ],
}

# ---------------------------------------------------------------------------
# Max joint velocity (rad/s) — rate limiter per control step.
# ---------------------------------------------------------------------------
MAX_JOINT_VEL = {
    "waist": [1.5],
    "head": [2.0, 2.0],
    "left_manipulator": [2.0, 2.0, 2.0, 2.0, 3.0, 3.0, 3.0],
    "right_manipulator": [2.0, 2.0, 2.0, 2.0, 3.0, 3.0, 3.0],
}

# ---------------------------------------------------------------------------
# Safe "arms-down" zero pose for ramp start / shutdown target.
# ---------------------------------------------------------------------------
ZERO_POSE = {
    "waist": [0.0],
    "head": [0.0, 0.0],
    "left_manipulator": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "right_manipulator": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
}

# All controlled groups.
UPPER_BODY_GROUPS = ["waist", "head", "left_manipulator", "right_manipulator"]
