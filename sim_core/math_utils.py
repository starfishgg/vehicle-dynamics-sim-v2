"""
math_utils.py

Contains mathematical helper functions used by 
the vehicle simulation
"""

import math




def clamp(value: float, minimum: float, maximum: float) -> float:
    """
    Restricts a value between minimum and maximum.
    """

    return max(minimum, min(value, maximum))


def rotate_local_vector(
        local_x: float,
        local_y: float,
        heading: float
) -> tuple[float, float]:
    """
    Converts a vector from vehicle-local coordinates
    into world coordinates.

    Args:
        local_x:
            Vector component in the vehicle's forward direction.

        local_y:
            Vector component in the vehicle's sideways direction.

        heading:
            Vehicle heading in degrees.

    Returns:
        A tuple containing the  world X and Y components.
    """

    heading_radians = math.radians(heading)

    world_x = (
        local_x * math.cos(heading_radians)
        - local_y * math.sin(heading_radians)
    )

    world_y = (
        local_x * math.sin(heading_radians)
        + local_y * math.cos(heading_radians)
    )

    return world_x, world_y



def torque_curve(
        rpm: float,
        peak_torque: float,
        base_torque: float,
        peak_rpm: float,
        rise_width: float,
        fall_width: float,
        ) -> float:
    """
    Calculate engine torque at a given RPM.

    The curve uses two Gaussian widths:

    - rise_width controls the shape before peak torque
    - fall_width controls the shape after peak torque

    This allows the engine to have a different torque curve on
    either side of its peak.

    Args:
        rpm:
            Current engine speed in revolutions per minute.

        peak_torque:
            Maximum torque produced by the engine in Nm.

        base_torque:
            Minimum torque level used by the curve in Nm.

        peak_rpm:
            RPM where peak torque occurs.

        rise_width:
            Width of the curve before peak RPM.

        fall_width:
            Width of the curve after peak RPM.

    Returns:
        Estimated engine torque in Nm.
    """

    # Use a different curve width depending on whether the engine
    # is below or above its peak torque RPM.
    if rpm < peak_rpm:
        width = rise_width
    else:
        width = fall_width

    torque = (
        (peak_torque - base_torque)
        * math.exp(
            -((rpm - peak_rpm) ** 2)
            / (2 * width ** 2)
        )
        + base_torque
    )

    return torque


def calculate_rpm_change(
        rpm,
        engine_torque,
        load_torque,
        engine_inertia,
        friction_torque,
        dt,
) -> float:

    angular_velocity = rpm * 2 * math.pi / 60

    net_torque = (
        engine_torque
        - load_torque
        - friction_torque
    )

    angular_acceleration = net_torque / engine_inertia

    angular_velocity += angular_acceleration * dt

    return max(0.0, angular_velocity * 60 / (2 * math.pi))


def calculate_wheel_angular_velocity(
        angular_velocity,
        drive_torque,
        brake_torque,
        resistance_torque,
        wheel_inertia,
        dt,
) -> float:
    
    net_torque = (
        drive_torque
        - brake_torque
        - resistance_torque
    )

    angular_acceleration = net_torque / wheel_inertia

    angular_velocity += angular_acceleration * dt

    return max(0.0, angular_velocity)


def tyre_force_coefficient(slip_ratio, B, C, D, E):
    bx = B * slip_ratio

    return D * math.sin(
        C * math.atan(
            bx - E * (bx - math.atan(bx))
        )
    )




# don't forget to sum all 4 wheels!
def tyre_force_to_chassis(
    longitudinal_force,
    lateral_force,
    steering_angle,
):
    cos_angle = math.cos(steering_angle)
    sin_angle = math.sin(steering_angle)

    chassis_x = (
        longitudinal_force * cos_angle
        - lateral_force * sin_angle
    )

    chassis_y = (
        longitudinal_force * sin_angle
        + lateral_force * cos_angle
    )

    return chassis_x, chassis_y


def calculate_total_force(wheels):
    total_x = 0.0
    total_y = 0.0

    for wheel in wheels:
        force_x, force_y = tyre_force_to_chassis(
            wheel.longitudinal_force,
            wheel.lateral_force,
            wheel.steering_angle,
        )

        total_x += force_x
        total_y += force_y

    return total_x, total_y


def calculate_acceleration(total_force_x, total_force_y, mass):
    acceleration_x = total_force_x / mass
    acceleration_y = total_force_y / mass

    return acceleration_x, acceleration_y


def calculate_yaw_moment(
    force_x,
    force_y,
    position_x,
    position_y,
):
    return (
        position_x * force_y
        - position_y * force_x
    )


def calculate_total_yaw_moment(wheels):
    total_moment = 0.0

    for wheel in wheels:
        force_x, force_y = tyre_force_to_chassis(
            wheel.longitudinal_force,
            wheel.lateral_force,
            wheel.steering_angle,
        )

        total_moment += calculate_yaw_moment(
            force_x,
            force_y,
            wheel.position_x,
            wheel.position_y,
        )

    return total_moment


def calculate_yaw_acceleration(yaw_moment, yaw_inertia):
    return yaw_moment / yaw_inertia


def calculate_chassis_dynamics(car):
    total_force_x = 0.0
    total_force_y = 0.0
    total_yaw_moment = 0.0

    for wheel in car.wheels:

        force_x, force_y = tyre_force_to_chassis(
            wheel.longitudinal_force,
            wheel.lateral_force,
            wheel.steering_angle,
        )

        total_force_x += force_x
        total_force_y += force_y

        total_yaw_moment += calculate_yaw_moment(
            force_x,
            force_y,
            wheel.position_x,
            wheel.position_y,
        )

    acceleration_x = total_force_x / car.mass
    acceleration_y = total_force_y / car.mass
    yaw_acceleration = total_yaw_moment / car.yaw_inertia

    return (
        acceleration_x,
        acceleration_y,
        yaw_acceleration,
    )
