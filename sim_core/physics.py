"""
physics.py

Contains the basic vehicle physics calculations.

This module is responsible for converting tyre behaviour into
forces and moments acting on the vehicle chassis.

The main flow is:

    Wheel slip
        ↓
    Tyre force coefficient
        ↓
    Tyre force
        ↓
    Chassis force
        ↓
    Vehicle acceleration

And separately:

    Tyre force
        ↓
    Yaw moment
        ↓
    Yaw acceleration


Coordinate system:

    Vehicle-local X = forward
    Vehicle-local Y = left

    Positive yaw moment = turning counter-clockwise
"""

import math
from dataclasses import dataclass

from sim_core.tyre_wheel import Wheel




@dataclass
class PhysicsState:
    """
    Stores the current physical state of the vehicle.

    Position and velocity use the vehicle's world coordinates.

    Heading describes which direction the vehicle is pointing.
    Yaw rate describes how quickly the vehicle is rotating.
    """

    position_x: float = 0.0
    position_y: float = 0.0

    velocity_x: float = 0.0
    velocity_y: float = 0.0

    acceleration_x: float = 0.0
    acceleration_y: float = 0.0

    heading: float = 0.0
    yaw_rate: float = 0.0
    yaw_acceleration: float = 0.0




# ------------------------------------------------------------
# Tyre force calculations
# ------------------------------------------------------------

def tyre_force_coefficient(
    slip: float,
    B: float,
    C: float,
    D: float,
    E: float,
) -> float:
    """
    Calculate a tyre force coefficient using the simplified
    Pacejka Magic Formula.

    Formula:

        y = D * sin(
                C * atan(
                    Bx - E * (Bx - atan(Bx))
                )
            )

    Args:
        slip:
            Slip ratio or slip angle.

        B:
            Stiffness factor.

        C:
            Shape factor.

        D:
            Peak force coefficient.

        E:
            Curvature factor.

    Returns:
        A dimensionless tyre force coefficient.
    """

    bx = B * slip

    return D * math.sin(
        C * math.atan(
            bx - E * (bx - math.atan(bx))
        )
    )


# These are deliberately simple starting values.
#
# D represents the approximate peak friction coefficient.
#
# A value of 1.0 means the tyre can produce approximately
# one times its normal load as force.
SPORTS_TYRE = {
    "B": 10.0,
    "C": 1.9,
    "D": 1.0,
    "E": 0.97,
}


def calculate_longitudinal_tyre_force(
    wheel: Wheel,
    tyre_settings: dict[str, float],
) -> float:
    """
    Calculate longitudinal tyre force from wheel slip.

    Positive slip means the wheel is rotating faster than
    the vehicle is moving, which produces forward traction.

    The force is limited by the tyre's available grip.

    Args:
        wheel:
            Wheel whose slip and normal force are being used.

        tyre_settings:
            Magic Formula parameters.

    Returns:
        Longitudinal tyre force in newtons.
    """

    force_coefficient = tyre_force_coefficient(
        slip=wheel.slip_ratio,
        B=tyre_settings["B"],
        C=tyre_settings["C"],
        D=tyre_settings["D"],
        E=tyre_settings["E"],
    )

    """
        force_coefficient = tyre_force_coefficient(
        slip=wheel.slip_ratio,
        **tyre_settings,
    )
    """

    return force_coefficient * wheel.normal_force


def calculate_lateral_tyre_force(
    wheel: Wheel,
    tyre_settings: dict[str, float],
) -> float:
    """
    Calculate the sideways force produced by a tyre.

    This uses the same simplified Magic Formula as the
    longitudinal force calculation.

    In a more advanced model, lateral and longitudinal grip
    would interact through a combined-slip model.

    Args:
        wheel:
            Wheel whose slip angle and normal force are used.

        tyre_settings:
            Magic Formula parameters.

    Returns:
        Lateral tyre force in newtons.
    """

    force_coefficient = tyre_force_coefficient(
        slip=wheel.slip_angle,
        **tyre_settings,
    )

    # The negative sign makes the tyre force oppose the slip angle.
    #
    # If the tyre is moving sideways to the left, the tyre should
    # generally produce a force pushing it back toward the right.
    return -force_coefficient * wheel.normal_force


# ------------------------------------------------------------
# Wheel force conversion
# ------------------------------------------------------------

def tyre_force_to_chassis(
    longitudinal_force: float,
    lateral_force: float,
    steering_angle: float,
) -> tuple[float, float]:
    """
    Convert a tyre's local forces into vehicle-local coordinates.

    A tyre's force directions depend on its steering angle.

    For a wheel pointing straight ahead:

        longitudinal force → vehicle forward direction
        lateral force       → vehicle sideways direction

    For a steered wheel, the force vector must be rotated.

    Args:
        longitudinal_force:
            Force along the wheel's rolling direction.

        lateral_force:
            Force across the wheel.

        steering_angle:
            Wheel steering angle in radians.

    Returns:
        Tuple containing:

            chassis_force_x
            chassis_force_y
    """

    cos_angle = math.cos(steering_angle)
    sin_angle = math.sin(steering_angle)

    chassis_force_x = (
        longitudinal_force * cos_angle
        - lateral_force * sin_angle
    )

    chassis_force_y = (
        longitudinal_force * sin_angle
        + lateral_force * cos_angle
    )

    return chassis_force_x, chassis_force_y


# ------------------------------------------------------------
# Total chassis force
# ------------------------------------------------------------

def calculate_total_force(
    wheels: list[Wheel],
) -> tuple[float, float]:
    """
    Sum the forces produced by all wheels.

    Each wheel's force is first converted into vehicle-local
    coordinates, then added to the total.

    Args:
        wheels:
            List containing the vehicle's wheels.

    Returns:
        Tuple containing:

            total_force_x
            total_force_y

        Both values are in newtons.
    """

    total_force_x = 0.0
    total_force_y = 0.0

    for wheel in wheels:
        force_x, force_y = tyre_force_to_chassis(
            longitudinal_force=wheel.longitudinal_force,
            lateral_force=wheel.lateral_force,
            steering_angle=wheel.steering_angle,
        )

        total_force_x += force_x
        total_force_y += force_y

    return total_force_x, total_force_y


def calculate_acceleration(
    total_force_x: float,
    total_force_y: float,
    mass: float,
) -> tuple[float, float]:
    """
    Calculate chassis acceleration using Newton's second law.

    Formula:

        acceleration = force / mass

    Args:
        total_force_x:
            Total forward/backward force in newtons.

        total_force_y:
            Total sideways force in newtons.

        mass:
            Vehicle mass in kilograms.

    Returns:
        Tuple containing:

            acceleration_x
            acceleration_y

        Both values are in metres per second squared.
    """

    acceleration_x = total_force_x / mass
    acceleration_y = total_force_y / mass

    return acceleration_x, acceleration_y


# ------------------------------------------------------------
# Yaw moment calculations
# ------------------------------------------------------------

def calculate_yaw_moment(
    force_x: float,
    force_y: float,
    position_x: float,
    position_y: float,
) -> float:
    """
    Calculate the yaw moment created by one wheel.

    Formula:

        Mz = rx * Fy - ry * Fx

    The wheel position is measured from the vehicle's
    centre of mass.

    Args:
        force_x:
            Wheel force along the vehicle's forward axis.

        force_y:
            Wheel force along the vehicle's sideways axis.

        position_x:
            Wheel position forward/backward from the centre of mass.

        position_y:
            Wheel position left/right from the centre of mass.

    Returns:
        Yaw moment in Nm.
    """

    return (
        position_x * force_y
        - position_y * force_x
    )


def calculate_total_yaw_moment(
    wheels: list[Wheel],
) -> float:
    """
    Calculate the total yaw moment produced by all wheels.

    Each wheel contributes a moment based on:

        force × distance from the centre of mass

    Args:
        wheels:
            List containing the vehicle's wheels.

    Returns:
        Total yaw moment in Nm.
    """

    total_moment = 0.0

    for wheel in wheels:
        force_x, force_y = tyre_force_to_chassis(
            longitudinal_force=wheel.longitudinal_force,
            lateral_force=wheel.lateral_force,
            steering_angle=wheel.steering_angle,
        )

        wheel_moment = calculate_yaw_moment(
            force_x=force_x,
            force_y=force_y,
            position_x=wheel.position_x,
            position_y=wheel.position_y,
        )

        total_moment += wheel_moment

    return total_moment


def calculate_yaw_acceleration(
    yaw_moment: float,
    yaw_inertia: float,
) -> float:
    """
    Calculate angular acceleration around the vertical axis.

    Formula:

        angular acceleration = yaw moment / yaw inertia

    Args:
        yaw_moment:
            Total yaw moment in Nm.

        yaw_inertia:
            Vehicle rotational inertia in kg·m².

    Returns:
        Yaw acceleration in radians per second squared.
    """

    return yaw_moment / yaw_inertia


# ------------------------------------------------------------
# Complete chassis calculation
# ------------------------------------------------------------

def calculate_chassis_dynamics(
    wheels: list[Wheel],
    mass: float,
    yaw_inertia: float,
) -> tuple[float, float, float]:
    """
    Calculate the chassis acceleration and yaw acceleration.

    This combines the force and moment calculations.

    Args:
        wheels:
            List containing all four wheels.

        mass:
            Vehicle mass in kilograms.

        yaw_inertia:
            Vehicle rotational inertia in kg·m².

    Returns:
        Tuple containing:

            acceleration_x
            acceleration_y
            yaw_acceleration
    """

    total_force_x, total_force_y = calculate_total_force(
        wheels=wheels,
    )

    total_yaw_moment = calculate_total_yaw_moment(
        wheels=wheels,
    )

    acceleration_x, acceleration_y = calculate_acceleration(
        total_force_x=total_force_x,
        total_force_y=total_force_y,
        mass=mass,
    )

    yaw_acceleration = calculate_yaw_acceleration(
        yaw_moment=total_yaw_moment,
        yaw_inertia=yaw_inertia,
    )

    return (
        acceleration_x,
        acceleration_y,
        yaw_acceleration,
    )
