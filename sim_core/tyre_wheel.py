"""
tyre_wheel.py

Contains the Wheel class.

A Wheel stores the physical state of one wheel and the forces
currently acting through its tyre.

At this stage, the class does not calculate the complete vehicle
movement. That will be handled later by physics.py.

Coordinate system:

    Vehicle-local X = forward
    Vehicle-local Y = left

Wheel positions are measured from the vehicle's centre of mass.

Example:

                    FRONT

             FL                 FR
             ●------------------●
             |                  |
             |       COM        |
             |                  |
             ●------------------●
             RL                 RR

                    REAR
"""

import math


class Wheel:
    """
    Represents one physical wheel.

    The wheel stores:

    - Its position on the vehicle
    - Its steering angle
    - Its rotational state
    - Its tyre forces
    - Whether it receives engine torque
    """

    def __init__(
            self,
            name:str,
            position_x: float,
            position_y: float,
            radius: float = 0.32,
            inertia: float = 2.5,
            driven: bool = False,
            steering: bool = False,
    ) -> None:
        """
        Initialise a wheel.

        Args:
            name:
                Human-readable wheel name.

            position_x:
                Wheel position along the vehicle's forward/backward axis.

                Positive values are in front of the centre of mass.
                Negative values are behind the centre of mass.

            position_y:
                Wheel position along the vehicle's left/right axis.

                Positive values are to the left.
                Negative values are to the right.

            radius:
                Wheel radius in metres.

            inertia:
                Wheel rotational inertia in kg·m².

            driven:
                True if engine torque is delivered to this wheel.

            steering:
                True if this wheel can steer.
        """

        # --------------------------------------------------------
        # Basic wheel information
        # --------------------------------------------------------
        
        self.name: str = name

        # Position relative to the vehicle's centre of mass.
        self.position_x: float = position_x
        self.position_y: float = position_y

        self.radius: float = radius
        self.inertia: float = inertia

        self.driven: bool = driven
        self.steering:bool = steering

        # --------------------------------------------------------
        # Steering and rotation state
        # --------------------------------------------------------

        # Steering angle is stored in radians.
        #
        # 0.0 means the wheel points straight ahead.
        self.steering_angle: float = 0.0

        # Wheel angular velocity in radians per second.
        self.angular_velocity: float = 0.0

        # Individual wheel torque (total torque divided by driven wheels)
        self.drive_torque: float = 0.0

        # --------------------------------------------------------
        # Tyre state
        # --------------------------------------------------------

        # Longitudinal slip:
        #
        # Positive values normally represent a driven wheel
        # rotating faster than the road speed.
        self.slip_ratio: float = 0.0

        # Lateral slip angle in radians.
        self.slip_angle: float = 0.0

        # --------------------------------------------------------
        # Forces acting through the tyre
        # --------------------------------------------------------

        # Force along the wheel's rolling direction.
        self.longitudinal_force: float = 0.0

        # Force sideways across the tyre.
        self.lateral_force: float = 0.0

        # Normal loal pressing the tyre into the road.
        self.normal_force: float = 0.0


    # TODO: Check tyre/wheel width values, do we need to add on a tyre R size?
    def get_surface_speed(self) -> float:
        """
        Return the speed of the tyre contact patch.
        
        The outside edge of the wheel moves at:
            
            speed = angular velocity * radius

        Returns:
            Tyre surface speed in metres per second.
        """

        return self.angular_velocity * self.radius


    def update_rotation(
            self,
            brake_torque: float,
            dt: float,
    ) -> None:
        """
        Update wheel angular velocity from the torques acting on the wheel.

        Drive torque rotates the wheel forward.

        Brake torque and tyre reaction torque oppose wheel rotation.

        Tyre reaction torque is calculated from the longitudinal tyre force.
        """

        if dt <= 0.0:
            raise ValueError("dt must be greater than zero.")

        tyre_reaction_torque = (
            self.longitudinal_force
            * self.radius
        )

        net_torque = (
            self.drive_torque
            - brake_torque
            - tyre_reaction_torque
        )

        angular_acceleration = (
            net_torque / self.inertia
        )

        self.angular_velocity += (
            angular_acceleration * dt
        )

        # Prevent the wheel from rotating backwards
        # because of braking when it has reached zero speed.
        if self.angular_velocity < 0.0:
            self.angular_velocity = 0.0


    def get_wheel_torque(
            self,
            engine_torque: float,
            total_gear_ratio: float,
            drivetrain_efficiency: float = 0.90,
    ) -> float:
        """
        Calculate the torque delivered to this wheel.
        
        This is a simple open-drivetrain model.
        
        For now, engine torque is divided equally between all
        driven wheels.

        Args:
            engine_torque:
                Torque produced by the engine in Nm.

            total_gear_ratio:
                Gearbox ratio multiplied by final drive ratio.

            drivetrain_efficiency:
                Fraction of torque that reaches the wheels.

        Returns:
            Torque delivered to this wheel in Nm.
        """

        # Non-driven wheels receive no engine torque.
        if not self.driven:
            return 0.0

        # Calculate torque at the drivetrain output.
        drivetrain_torque = (
            engine_torque
            * total_gear_ratio
            * drivetrain_efficiency
        )

        return drivetrain_torque


    def calculate_slip_ratio(
            self,
            vehicle_speed: float,
    ) -> float:
        """
        Calculate longitudinal tyre slip.

        Wheel surface speed is:

            angular velocity * radius

        The difference betweeen wheel surface speed and vehicle
        speed gives us the amount of longitudinal slip.
        
        A small minimuim speed avoids unstable division when the vehicle is almost stationary.

        Args:
            vehicle_speed:
                Vehicle speed in the wheel's forward direction,
                measured in metres per second.

        Returns:
            Longitudinal slip ratio.
        """

        wheel_speed = self.get_surface_speed()

        # Avoid dividing by zero when the vehicle is stationary
        minimum_speed = 0.1

        reference_speed = max(
            abs(vehicle_speed),
            minimum_speed
        )

        self.slip_ratio = (
            wheel_speed - vehicle_speed
        ) / reference_speed

        return self.slip_ratio


    def calculate_slip_angle(
            self,
            forward_velocity: float,
            sideways_velocity: float,
    ) -> float:
        """
        Calculate the tyre slip angle.
        
        The slip angle describes the difference between:
        
            - The direction the wheel is pointing
            - The direction the tyre is actually moving

        Args:
            forward_velocity:
                Wheel velocity along the wheel's forward direction.

            sideways_velocity:
                Wheel velocity across the wheel's sideways direction.

        Returns:
            Slip angle in radians.
        """

        # Avoid an unstable angle calculation at extremely low speed.
        minimum_forward_speed = 0.1

        safe_forward_velocity = max(
            abs(forward_velocity),
            minimum_forward_speed
        )

        self.slip_angle = math.atan2(
            sideways_velocity,
            safe_forward_velocity
        )

        return self.slip_angle


    def reset_forces(self) -> None:
        """
        Clear the forces calculated during the previous physics step.
        
        This is useful because each simulation update should
        calculate a fresh set of tyre forces.
        """

        self.longitudinal_force = 0.0
        self.lateral_force = 0.0
