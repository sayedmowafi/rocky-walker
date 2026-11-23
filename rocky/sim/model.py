"""MuJoCo model of Rocky, generated from the robot config.

Servos are position actuators with a torque limit equal to the MG996R stall
torque, so the sim refuses anything the real servos couldn't do.
"""

from __future__ import annotations

import mujoco
import numpy as np

from rocky.config import RobotConfig
from rocky.kinematics.body import body_to_leg, leg_mounts, neutral_feet
from rocky.kinematics.leg import leg_ik

FOOT_RADIUS = 0.010
FOOT_MASS = 0.005
TIMESTEP = 0.002
# Builder-tunable actuator/joint constants (not part of the robot config).
ACTUATOR_KP = 20.0
JOINT_DAMPING = 0.1
JOINT_ARMATURE = 0.002
FRICTION = "1 0.005 0.0001"

_JOINTS = ("coxa", "femur", "tibia")


def joint_names(n_legs: int = 5) -> list[str]:
    """Joint/actuator names, row-major: matches ``angles.reshape(-1)``."""
    return [f"leg{i}_{j}" for i in range(n_legs) for j in _JOINTS]


def _f(*values: float) -> str:
    return " ".join(f"{v:.10g}" for v in values)


def stand_angles(cfg: RobotConfig) -> np.ndarray:
    """Joint angles of the neutral stance, shape ``(n_legs, 3)``."""
    feet = neutral_feet(cfg)
    return np.array([leg_ik(body_to_leg(feet[m.index], m), cfg.leg) for m in leg_mounts(cfg.body)])


def build_mjcf(cfg: RobotConfig) -> str:
    leg, lim, mass = cfg.leg, cfg.limits.as_array(), cfg.mass
    n = cfg.body.n_legs
    link_total = mass.coxa_link + mass.femur_link + mass.tibia_link
    torso_mass = mass.total - n * link_total
    z0 = cfg.gait.body_height + FOOT_RADIUS
    tau = cfg.servo.stall_torque

    legs = []
    for m in leg_mounts(cfg.body):
        i = m.index
        legs.append(f"""
      <body name="leg{i}" pos="{_f(*m.position)}" euler="0 0 {m.yaw:.9g}">
        <joint name="leg{i}_coxa" axis="0 0 1" range="{_f(*lim[0])}"/>
        <geom type="capsule" fromto="0 0 0 {_f(leg.coxa)} 0 0" size="0.012"
              mass="{_f(mass.coxa_link)}" rgba="0.15 0.15 0.15 1"/>
        <body name="leg{i}_femur" pos="{_f(leg.coxa)} 0 0">
          <joint name="leg{i}_femur" axis="0 -1 0" range="{_f(*lim[1])}"/>
          <geom type="capsule" fromto="0 0 0 {_f(leg.femur)} 0 0" size="0.011"
                mass="{_f(mass.femur_link)}" rgba="0.9 0.9 0.9 1"/>
          <body name="leg{i}_tibia" pos="{_f(leg.femur)} 0 0">
            <joint name="leg{i}_tibia" axis="0 -1 0" range="{_f(*lim[2])}"/>
            <geom type="capsule" fromto="0 0 0 {_f(leg.tibia)} 0 0" size="0.008"
                  mass="{_f(mass.tibia_link - FOOT_MASS)}" rgba="0.2 0.45 0.85 1"/>
            <geom name="leg{i}_foot" type="sphere" pos="{_f(leg.tibia)} 0 0"
                  size="{_f(FOOT_RADIUS)}" mass="{_f(FOOT_MASS)}" contype="2" conaffinity="0"
                  friction="{FRICTION}" rgba="0.1 0.1 0.1 1"/>
          </body>
        </body>
      </body>""")

    actuators = "\n".join(
        f'    <position name="{name}" joint="{name}" kp="{_f(ACTUATOR_KP)}" '
        f'forcerange="{_f(-tau, tau)}" ctrlrange="{_f(*lim[k % 3])}"/>'
        for k, name in enumerate(joint_names(n))
    )
    legs_xml = "".join(legs)
    stand_qpos = _f(0, 0, z0, 1, 0, 0, 0, *stand_angles(cfg).reshape(-1))

    return f"""<mujoco model="rocky">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{TIMESTEP}"/>
  <visual><global offwidth="1280" offheight="720"/></visual>
  <default>
    <joint damping="{_f(JOINT_DAMPING)}" armature="{_f(JOINT_ARMATURE)}"/>
    <geom contype="0" conaffinity="0"/>
  </default>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.82 0.82 0.8" rgb2="0.72 0.72 0.7"
             width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="8 8"/>
  </asset>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="10 10 0.1" material="grid" contype="1"
          conaffinity="2" friction="{FRICTION}"/>
    <body name="torso" pos="0 0 {_f(z0)}">
      <freejoint name="root"/>
      <camera name="track" mode="trackcom" pos="0.42 -0.42 0.32"
              xyaxes="0.707 0.707 0 -0.33 0.33 0.88"/>
      <geom name="torso" type="cylinder" size="{_f(cfg.body.mount_radius, cfg.body.thickness / 2)}"
            mass="{_f(torso_mass)}" contype="2" conaffinity="0" rgba="0.55 0.5 0.45 1"/>{legs_xml}
    </body>
  </worldbody>
  <actuator>
{actuators}
  </actuator>
  <keyframe>
    <key name="stand" qpos="{stand_qpos}" ctrl="{_f(*stand_angles(cfg).reshape(-1))}"/>
  </keyframe>
</mujoco>
"""


def load_model(cfg: RobotConfig) -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_string(build_mjcf(cfg))
