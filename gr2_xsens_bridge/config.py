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
#          elbow_pitch, wrist_pitch, wrist_roll, wrist_yaw]
# ---------------------------------------------------------------------------
SIGN_MAP = {
    "waist": [1.0],
    "head": [1.0, 1.0],  # [pan, tilt]
    "left_manipulator": [1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
    "right_manipulator": [1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
}

# ---------------------------------------------------------------------------
# Joint limits (radians).  Placeholder values — refine from GR-2 URDF/docs.
# Order matches SIGN_MAP.
# ---------------------------------------------------------------------------
JOINT_LIMITS = {
    "waist": [(-1.57, 1.57)],
    "head": [(-1.2, 1.2), (-0.5, 0.5)],
    "left_manipulator": [
        (-3.1, 1.5),   # shoulder_pitch
        (-0.3, 2.8),   # shoulder_roll
        (-1.5, 2.6),   # shoulder_yaw
        (-2.6, 0.0),   # elbow_pitch
        (-1.0, 1.0),   # wrist_pitch
        (-0.5, 0.5),   # wrist_roll
        (-0.3, 0.3),   # wrist_yaw
    ],
    "right_manipulator": [
        (-3.1, 1.5),   # shoulder_pitch
        (-2.8, 0.3),   # shoulder_roll (mirrored)
        (-2.6, 1.5),   # shoulder_yaw (mirrored)
        (-2.6, 0.0),   # elbow_pitch
        (-1.0, 1.0),   # wrist_pitch
        (-0.5, 0.5),   # wrist_roll
        (-0.3, 0.3),   # wrist_yaw
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
