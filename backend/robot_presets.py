#!/usr/bin/env python3
"""
backend/robot_presets.py — Preloaded robot model library (Part 2 extension).

Each preset returns a KinematicChain with DH parameters for a well-known
robot configuration. All presets work with KinematicChain.inverse_kinematics()
(damped least-squares numerical IK) — no solver changes needed.

Available presets
-----------------
PRESETS dict maps display name → factory function.

Usage::

    from backend.robot_presets import PRESETS, get_preset

    chain = get_preset("UR5 (6-DOF)")
    # chain is a KinematicChain ready to use with forward/inverse kinematics

DH Convention
-------------
Standard Denavit-Hartenberg (Craig convention):
    [theta, d, a, alpha]  (theta in degrees, d/a in metres, alpha in degrees)
"""

from backend.kinematics import KinematicChain, DHJoint


# ── Helper ────────────────────────────────────────────────────────────────────

def _joint(jtype, theta, d, a, alpha, name, q_min=None, q_max=None) -> DHJoint:
    return DHJoint(jtype, theta=theta, d=d, a=a, alpha=alpha,
                   name=name, q_min=q_min, q_max=q_max)

def _rev(theta, d, a, alpha, name, lo=-180.0, hi=180.0) -> DHJoint:
    return _joint('revolute', theta, d, a, alpha, name, lo, hi)

def _fix(d, a, alpha, name) -> DHJoint:
    return _joint('fixed', 0.0, d, a, alpha, name)


# ═══════════════════════════════════════════════════════════════════════════════
#  3-DOF Presets
# ═══════════════════════════════════════════════════════════════════════════════

def preset_planar_3r() -> KinematicChain:
    """Planar 3R — all revolute, all joints in XY plane.

    Classic textbook robot. All alpha=0 so motion is confined to a plane.
    Good for teaching IK concepts.

    Workspace: ~0.55 m radius circle in the XY plane.
    """
    joints = [
        _rev(0, 0.0,  0.20, 0.0, "J1 — Base",     -180, 180),
        _rev(0, 0.0,  0.20, 0.0, "J2 — Shoulder",  -150, 150),
        _rev(0, 0.0,  0.15, 0.0, "J3 — Elbow",     -150, 150),
    ]
    return KinematicChain(joints, base_height=0.0)


def preset_scara() -> KinematicChain:
    """SCARA (Selective Compliance Assembly Robot Arm) — 4-DOF.

    Used in pick-and-place, assembly tasks. 3 revolute joints in horizontal
    plane + 1 prismatic joint for vertical motion.

    DH based on generic SCARA geometry.
    Workspace: ~0.55 m horizontal radius, 0.15 m vertical stroke.
    """
    joints = [
        _rev(0,   0.10, 0.25, 0.0,   "J1 — Base Yaw",    -170, 170),
        _rev(0,   0.0,  0.20, 180.0, "J2 — Elbow Yaw",   -150, 150),
        _rev(0,   0.0,  0.0,  0.0,   "J3 — Wrist Yaw",   -360, 360),
        _joint('prismatic', 0, 0.0, 0.0, 0.0,
               "J4 — Vertical (Z)", q_min=0.0, q_max=0.15),
    ]
    return KinematicChain(joints, base_height=0.0)


def preset_cylindrical() -> KinematicChain:
    """Cylindrical arm — 1 revolute (base yaw) + 2 prismatic (R, Z).

    Workspace is a hollow cylinder. Simple, used in material handling.
    """
    joints = [
        _rev(0,   0.10, 0.0, 90.0, "J1 — Base Yaw",    -180, 180),
        _joint('prismatic', 0, 0.0,  0.0, 90.0,
               "J2 — Radial (R)", q_min=0.05, q_max=0.40),
        _joint('prismatic', 0, 0.0,  0.0,  0.0,
               "J3 — Vertical (Z)", q_min=0.0, q_max=0.30),
    ]
    return KinematicChain(joints, base_height=0.0)


def preset_spherical() -> KinematicChain:
    """Spherical (Polar) arm — 2 revolute + 1 prismatic.

    Like early Unimate robots. Workspace is a partial sphere.
    """
    joints = [
        _rev(0,   0.10, 0.0,  90.0, "J1 — Base Yaw",    -180, 180),
        _rev(0,   0.0,  0.0, -90.0, "J2 — Shoulder Pitch", -90, 90),
        _joint('prismatic', 0, 0.05, 0.0,  0.0,
               "J3 — Extend (R)", q_min=0.05, q_max=0.40),
    ]
    return KinematicChain(joints, base_height=0.0)


def preset_elbow_manipulator() -> KinematicChain:
    """Elbow manipulator — 3R articulated arm (textbook classic).

    Identical topology to the Standard 3-DOF mode but defined via DH params
    so it can use the generic numerical IK. Matches the default ArmConfig.
    """
    joints = [
        _rev(0, 0.10, 0.0,   90.0, "J1 — Base Yaw",       -180, 180),
        _rev(0, 0.0,  0.30,   0.0, "J2 — Shoulder Pitch",  -90,  90),
        _rev(0, 0.0,  0.25,   0.0, "J3 — Elbow Pitch",    -135, 135),
    ]
    return KinematicChain(joints, base_height=0.0)


# ═══════════════════════════════════════════════════════════════════════════════
#  6-DOF Presets
# ═══════════════════════════════════════════════════════════════════════════════

def preset_ur5() -> KinematicChain:
    """Universal Robots UR5 — 6-DOF collaborative arm.

    DH parameters sourced from UR5 datasheet (modified DH → standard DH
    approximation for visualisation; not production-accurate).

    Link lengths in metres, angles in degrees.
    Payload: 5 kg | Reach: ~850 mm.
    """
    joints = [
        _rev(0,  0.0892,  0.0,     90.0,  "J1 — Base",         -360, 360),
        _rev(0,  0.0,    -0.4250,   0.0,  "J2 — Shoulder",     -360, 360),
        _rev(0,  0.0,    -0.3922,   0.0,  "J3 — Elbow",        -360, 360),
        _rev(0,  0.1093,  0.0,      90.0, "J4 — Wrist 1",      -360, 360),
        _rev(0,  0.0948,  0.0,     -90.0, "J5 — Wrist 2",      -360, 360),
        _rev(0,  0.0825,  0.0,       0.0, "J6 — Wrist 3",      -360, 360),
    ]
    return KinematicChain(joints, base_height=0.0, name="UR5")


def preset_puma560() -> KinematicChain:
    """PUMA 560 — Classic 6-DOF industrial arm (Unimation, 1978).

    One of the most studied robots in robotics academia.
    DH parameters from Corke's Robotics Toolbox reference.
    All dimensions in metres.
    """
    joints = [
        _rev(0,  0.0,     0.0,      90.0,  "J1 — Waist",       -160, 160),
        _rev(0,  0.4318,  0.4318,   0.0,   "J2 — Shoulder",    -45,   225),
        _rev(0, -0.0203,  0.0,     -90.0,  "J3 — Elbow",       -225,  45),
        _rev(0,  0.4318,  0.0,      90.0,  "J4 — Wrist Roll",  -110, 170),
        _rev(0,  0.0,     0.0,     -90.0,  "J5 — Wrist Pitch", -100, 100),
        _rev(0,  0.0,     0.0,       0.0,  "J6 — Wrist Yaw",   -266, 266),
    ]
    return KinematicChain(joints, base_height=0.0, name="PUMA 560")


def preset_stanford_arm() -> KinematicChain:
    """Stanford Arm — 6-DOF with 1 prismatic joint (RRP-RRR).

    One of the earliest computer-controlled robot arms (1969, Stanford).
    Wrist-partitioned design: first 3 joints position EE, last 3 orient it.
    """
    joints = [
        _rev(0,     0.412,  0.0,    -90.0, "J1 — Base Yaw",    -180, 180),
        _rev(0,     0.154,  0.0,     90.0, "J2 — Shoulder",    -90,  90),
        _joint('prismatic', 0, 0.0, 0.0,  -90.0,
               "J3 — Extend (R)", q_min=0.05, q_max=0.60),
        _rev(0,     0.0,    0.0,    -90.0, "J4 — Wrist Roll",  -180, 180),
        _rev(0,     0.0,    0.0,     90.0, "J5 — Wrist Pitch", -90,  90),
        _rev(0,     0.263,  0.0,      0.0, "J6 — Wrist Yaw",   -180, 180),
    ]
    return KinematicChain(joints, base_height=0.0, name="Stanford Arm")


def preset_abb_irb120() -> KinematicChain:
    """ABB IRB 120 — Compact 6-DOF industrial robot.

    Payload: 3 kg | Reach: 580 mm. Popular in education and small-part assembly.
    DH approximation for visualisation.
    """
    joints = [
        _rev(0,   0.290,  0.0,    -90.0, "J1 — Base",         -165, 165),
        _rev(-90, 0.0,    0.270,   0.0,  "J2 — Shoulder",     -110, 110),
        _rev(90,  0.0,    0.070,  -90.0, "J3 — Elbow",        -110, 70),
        _rev(0,   0.302,  0.0,     90.0, "J4 — Wrist 1",      -160, 160),
        _rev(0,   0.0,    0.0,    -90.0, "J5 — Wrist 2",      -120, 120),
        _rev(0,   0.072,  0.0,      0.0, "J6 — Wrist 3",      -400, 400),
    ]
    return KinematicChain(joints, base_height=0.0, name="ABB IRB 120")


# ═══════════════════════════════════════════════════════════════════════════════
#  2-DOF Preset (educational)
# ═══════════════════════════════════════════════════════════════════════════════

def preset_2dof_planar() -> KinematicChain:
    """2-DOF Planar arm — simplest possible manipulator.

    Shoulder + elbow only. IK has closed-form solution (law of cosines),
    but numerical solver works fine. Good for IK teaching/debugging.
    """
    joints = [
        _rev(0, 0.0, 0.30, 0.0, "J1 — Shoulder", -180, 180),
        _rev(0, 0.0, 0.25, 0.0, "J2 — Elbow",    -150, 150),
    ]
    return KinematicChain(joints, base_height=0.10)


# ═══════════════════════════════════════════════════════════════════════════════
#  Registry
# ═══════════════════════════════════════════════════════════════════════════════

#: Ordered dict: display_name → (factory_fn, category, dof, description)
PRESETS: dict[str, dict] = {
    # ── 2-DOF ────────────────────────────────────────────────────────────────
    "2-DOF Planar": {
        "factory":     preset_2dof_planar,
        "category":    "Educational",
        "dof":         2,
        "description": "Simplest 2-link planar arm. Perfect for learning IK.",
        "icon":        "📐",
    },
    # ── 3-DOF ────────────────────────────────────────────────────────────────
    "Planar 3R": {
        "factory":     preset_planar_3r,
        "category":    "Educational",
        "dof":         3,
        "description": "Three revolute joints in a plane. Classic textbook robot.",
        "icon":        "📏",
    },
    "Elbow Manipulator": {
        "factory":     preset_elbow_manipulator,
        "category":    "Educational",
        "dof":         3,
        "description": "Standard 3-DOF articulated arm matching the built-in model.",
        "icon":        "🦾",
    },
    "Spherical Arm": {
        "factory":     preset_spherical,
        "category":    "Classic",
        "dof":         3,
        "description": "2 revolute + 1 prismatic. Polar workspace. Like early Unimate.",
        "icon":        "🌐",
    },
    "Cylindrical Arm": {
        "factory":     preset_cylindrical,
        "category":    "Classic",
        "dof":         3,
        "description": "Base yaw + radial + vertical prismatic. Hollow-cylinder workspace.",
        "icon":        "🔧",
    },
    # ── 4-DOF ────────────────────────────────────────────────────────────────
    "SCARA": {
        "factory":     preset_scara,
        "category":    "Industrial",
        "dof":         4,
        "description": "Selective compliance assembly robot. Ideal for pick-and-place.",
        "icon":        "🏭",
    },
    # ── 6-DOF ────────────────────────────────────────────────────────────────
    "UR5 (6-DOF)": {
        "factory":     preset_ur5,
        "category":    "Industrial",
        "dof":         6,
        "description": "Universal Robots UR5. Collaborative 6-DOF arm, 850 mm reach.",
        "icon":        "⚙️",
    },
    "PUMA 560": {
        "factory":     preset_puma560,
        "category":    "Classic",
        "dof":         6,
        "description": "Legendary 6-DOF industrial arm. Most studied robot in academia.",
        "icon":        "🎓",
    },
    "Stanford Arm": {
        "factory":     preset_stanford_arm,
        "category":    "Classic",
        "dof":         6,
        "description": "6-DOF with 1 prismatic joint (RRP-RRR). Wrist-partitioned.",
        "icon":        "🔬",
    },
    "ABB IRB 120": {
        "factory":     preset_abb_irb120,
        "category":    "Industrial",
        "dof":         6,
        "description": "Compact ABB 6-DOF arm. 580 mm reach, 3 kg payload.",
        "icon":        "🤖",
    },
}

CATEGORIES = ["Educational", "Classic", "Industrial"]


def get_preset(name: str) -> KinematicChain:
    """Return a fresh KinematicChain for the named preset.

    Args:
        name: Key from PRESETS dict (e.g. "UR5 (6-DOF)").

    Returns:
        New KinematicChain instance.

    Raises:
        KeyError: if name is not in PRESETS.
    """
    if name not in PRESETS:
        raise KeyError(f"Unknown preset '{name}'. Available: {list(PRESETS.keys())}")
    return PRESETS[name]["factory"]()


def list_presets_by_category() -> dict[str, list[str]]:
    """Return preset names grouped by category."""
    result: dict[str, list[str]] = {cat: [] for cat in CATEGORIES}
    for name, meta in PRESETS.items():
        result[meta["category"]].append(name)
    return result
