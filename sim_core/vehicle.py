"""
vehicle.py

Contains the Vehicle class.

Vehicle brings together the major parts of the simulator:

    DriverInput
        ↓
    Engine
        ↓
    Gearbox
        ↓
    Wheels
        ↓
    Tyre forces
        ↓
    Chassis physics

The Vehicle class coordinates these systems but does not contain
the detailed tyre or engine mathematics itself.
"""



from sim_core.driver_input import DriverInput
from sim_core.engine import Engine, Gearbox
from sim_core.physics import PhysicsState
from sim_core.tyre_wheel import Wheel
from sim_core.settings import PETROL_WEIGHT, SPORTS_TYRE
from sim_core.math_utils import clamp


import math




class Vehicle:
    """
    Represents a complete vehicle.

    This first version models a simple front-wheel-drive car.

    Front wheels:
        - Driven
        - Steered

    Rear wheels:
        - Not driven
        - Not steered
    """

    def __init__(
        self,
        driver_input: DriverInput,
        engine: Engine,
        gearbox: Gearbox,            
    ) -> None:

        # ----------------------------------------------------------
        # Driver controls
        # ----------------------------------------------------------
        
        self.driver_input = driver_input

        # ----------------------------------------------------------
        # Powertrain
        # ----------------------------------------------------------

        self.engine = engine
        self.gearbox = gearbox

        # ----------------------------------------------------------
        # Vehicle dimensions
        # ----------------------------------------------------------

        # Distance between the front and rear axles.
        self.wheelbase: float = 2.60

        # Distance between the left and right wheels.
        self.front_track: float = 1.50
        self.rear_track: float = 1.50

        # ----------------------------------------------------------
        # Vehicle mass
        # ----------------------------------------------------------

        # Mass of the vehicle without driver or fuel.
        self.body_mass: float = 1150.0 # kg

        # Minimum driver mass used by the simulator (as per F1 regs)
        self.driver_mass: float = 82.0 # kg

        # Fuel tank capacity in litres
        self.fuel_tank_size: float = 45.0

        # Start with a full tank.
        self.fuel_level: float = self.fuel_tank_size

        # Petrol density is currently stored as kg/litre.
        self.fuel_mass: float = self.fuel_level * PETROL_WEIGHT

        # Total mass used by the physics calculations.
        self.total_mass = (
            self.body_mass
            + self.fuel_mass
            + self.driver_mass
        )

        # ----------------------------------------------------------
        # Chassis rotational inertia
        # ----------------------------------------------------------

        # Simplified yaw moment of inertia.
        self.yaw_inertia: float = 1800.0

        # --------------------------------------------------------
        # Clutch
        # --------------------------------------------------------
        # The clutch allows the engine and wheels to rotate at
        # different speeds during launch.
        #
        # A higher capacity allows more engine torque to be
        # transferred before the clutch slips.
        self.clutch_capacity: float = 250.0

        # Controls how strongly the clutch reacts to speed
        # difference between the engine and drivetrain.
        self.clutch_stiffness: float = 20.0

        # ----------------------------------------------------------
        # Physical state
        # ----------------------------------------------------------
        self.physics = PhysicsState()

        # ----------------------------------------------------------
        # Wheels
        # ----------------------------------------------------------
        self.wheels = self._create_wheels()


    def _create_wheels(self) -> list[Wheel]:
        """
        Create the vehicle's four wheels.

        Vehicle-local coordinates:

            X = forward
            Y = left

        Therefore:

            Front axle   = positive X
            Rear axle    = negative X
            Left wheels  = positive Y
            Right wheels = negative Y
        """

        half_wheelbase = self.wheelbase / 2.0
        half_front_track = self.front_track / 2.0
        half_rear_track = self.rear_track / 2.0

        wheels = [
            Wheel(
                name="Front Left",
                position_x=half_wheelbase,
                position_y=half_front_track,
                driven=True,
                steering=True,
            ),

            Wheel(
                name="Front Right",
                position_x=half_wheelbase,
                position_y=-half_front_track,
                driven=True,
                steering=True,
            ),

            Wheel(
                name="Rear Left",
                position_x=-half_wheelbase,
                position_y=half_rear_track,
                driven=False,
                steering=False,
            ),

            Wheel(
                name="Rear Right",
                position_x=-half_wheelbase,
                position_y=-half_rear_track,
                driven=False,
                steering=False,
            ),
        ]

        return wheels


    def get_driven_wheels(self) -> list[Wheel]:
        """
        Return all wheels receiving engine torque.
        """

        return [
            wheel
            for wheel in self.wheels
            if wheel.driven
        ]


    def get_steering_wheels(self) -> list[Wheel]:
        """
        Return all wheels controlled by steering input
        """

        return [
            wheel
            for wheel in self.wheels
            if wheel.steering
        ]


    def get_drivetrain_angular_velocity(self) -> float:
        """
        Return the average angular velocity of the driven wheels.

        The gearbox ratio is used to convert wheel speed back into
        the equivalent engine speed.

        Returns:
            Equivalent engine angular velocity in rad/s.
        """

        driven_wheels = self.get_driven_wheels()

        if not driven_wheels:
            return 0.0

        average_wheel_speed = sum(
            wheel.angular_velocity
            for wheel in driven_wheels
        ) / len(driven_wheels)

        total_ratio = self.gearbox.get_total_ratio()

        if total_ratio == 0.0:
            return 0.0

        return average_wheel_speed * total_ratio


    def calculate_clutch_torque(self) -> float:
        """
        Calculate the torque transmitted through the clutch.

        The clutch transfers positive engine torque to the drivetrain.
        When the engine and drivetrain speeds are different, the clutch
        can slip and its transmitted torque is limited by clutch capacity.

        Once the engine and drivetrain are nearly synchronised, the clutch
        is considered engaged and transmits the available engine torque.
        """

        engine_angular_velocity = (
            self.engine.rpm * 2.0 * math.pi / 60.0
        )

        drivetrain_angular_velocity = (
            self.get_drivetrain_angular_velocity()
        )

        speed_difference = (
            engine_angular_velocity
            - drivetrain_angular_velocity
        )

        # When the engine and drivetrain are rotating at almost the
        # same speed, treat the clutch as fully engaged.
        if abs(speed_difference) < 1.0:
            return self.engine.get_available_torque()

        # While slipping, the clutch torque opposes the speed
        # difference between the engine and drivetrain.
        clutch_torque = (
            speed_difference * self.clutch_stiffness
        )

        # The clutch should not transmit more torque than its capacity.
        # Do not allow it to become negative and act like a brake.
        return clamp(
            clutch_torque,
            0.0,
            self.clutch_capacity
        )


    def get_drivetrain_torque(self) -> float:
        """
        Calculate the torque delivered to the drivetrain.

        When a gear is engaged, the clutch is treated as fully locked.
        Engine torque therefore passes directly through the gearbox and
        final drive, with a small allowance for drivetrain losses.
        """

        engine_torque = self.engine.get_available_torque()
        total_gear_ratio = self.gearbox.get_total_ratio()
        drivetrain_efficiency = 0.90

        return (
            engine_torque
            * total_gear_ratio
            * drivetrain_efficiency
        )


    def update_engine(self, dt: float) -> None:
        """
        Update engine RPM.

        When a gear is engaged, the clutch is treated as locked and
        engine speed follows the drivetrain speed through the gearbox.

        In neutral, the engine is free to accelerate and decelerate
        independently.
        """

        total_ratio = self.gearbox.get_total_ratio()

        if total_ratio == 0.0:
            self.engine.update_rpm(
                load_torque=0.0,
                dt=dt,
            )
            return

        drivetrain_angular_velocity = (
            self.get_drivetrain_angular_velocity()
        )

        drivetrain_rpm = (
            abs(drivetrain_angular_velocity)
            * 60.0
            / (2.0 * math.pi)
        )

        self.engine.rpm = clamp(
            drivetrain_rpm,
            self.engine.idle_rpm,
            self.engine.redline,
        )


    def distribute_drive_torque(self) -> None:
        """
        Divide drivetrain torque equally between driven wheels.

        This prevents each driven wheel from incorrectly receiving
        the entire engine output.

        A real differential is more complicated, but equal torque
        distribution is a useful starting point.
        """

        driven_wheels = self.get_driven_wheels()

        if not driven_wheels:
            return

        total_torque = self.get_drivetrain_torque()

        torque_per_wheel = (
            total_torque / len(driven_wheels)
        )

        for wheel in driven_wheels:
            wheel.drive_torque = torque_per_wheel


    def apply_driver_inputs(self) -> None:
        """
        Transfer the current driver commands into the vehicle systems.
        """

        # Keep all driver controls inside their valid ranges.
        self.driver_input.validate()

        # Transfer accelerator input to the engine.
        self.engine.throttle = self.driver_input.throttle

        # Apply the requested gear.

        # This is deliberately simple for now.
        # We are not simulating clutch movement or gear-shift timing yet
        self.gearbox.current_gear = self.driver_input.gear_request

        # Apply steering to the steered wheels.
        #
        # The steering input is normalised between -1.0 and 1.0.
        # Maximum steering angile is currently 30 degrees.
        maximum_steering_angle = 0.523599 # Radians

        steering_angle = (
            self.driver_input.steering
            * maximum_steering_angle
        )

        for wheel in self.get_steering_wheels():
            wheel.steering_angle = steering_angle


    def calculate_wheel_forces(self) -> None:
        """
        Calculate longitudinal and lateral force for every wheel.

        This is the first simplified force model.

        Longitudinal force comes from drive torque.

        Lateral force currently comes from the wheel slip angle.
        """

        from sim_core.physics import (
            calculate_lateral_tyre_force,
            calculate_longitudinal_tyre_force,
        )

        tyre_settings = SPORTS_TYRE
        for wheel in self.wheels:
            # Reset forces before calculating new ones.
            wheel.reset_forces()

            # Calculate wheel slip relative to vehicle forward speed.
            wheel.calculate_slip_ratio(
                self.physics.velocity_x
            )

            # Calculate slip angle.
            wheel.calculate_slip_angle(
                forward_velocity=self.physics.velocity_x,
                sideways_velocity=self.physics.velocity_y
            )

            # Calculate longitudinal tyre force.
            wheel.longitudinal_force = (
                calculate_longitudinal_tyre_force(
                    wheel,
                    tyre_settings
                )
            )

            # Calculate lateral tyre force.
            wheel.lateral_force = (
                calculate_lateral_tyre_force(
                    wheel,
                    tyre_settings,
                )
            )


    def calculate_normal_forces(self) -> None:
        """
        Calculate the static normal force on each wheel.

        This first version assumes:

        - The vehicle is standing on level ground.
        - Weight is distributed equally.
        - There is no weight transfer during acceleration.
        - There is no aerodynamic downforce.

        More advanced weight transfer will be added later.
        """

        gravity = 9.81

        total_weight = (self.total_mass * gravity)

        normal_force_per_wheel = (total_weight / len(self.wheels))

        for wheel in self.wheels:
            wheel.normal_force = normal_force_per_wheel


    def update(self, dt: float) -> None:
        """
        Advance the vehicle simulation by one time step.

        dt is the elapsed time in seconds.
        """

        if dt <= 0.0:
            raise ValueError("dt must be greater than zero.")

        # ----------------------------------------------------------
        # 1. Apply driver controls
        # ----------------------------------------------------------

        self.apply_driver_inputs()

        # ----------------------------------------------------------
        # 2. Calculate the normal load on each wheel
        # ----------------------------------------------------------
        
        self.calculate_normal_forces()

        # ----------------------------------------------------------
        # 3. Apply engine torque to driven wheels
        # ----------------------------------------------------------

        self.distribute_drive_torque()

        # ----------------------------------------------------------
        # 4. Calculate tyre forces
        # ----------------------------------------------------------

        self.calculate_wheel_forces()

        for wheel in self.wheels:
            wheel.update_rotation(
                brake_torque=0.0,
                dt=dt
            )

        # The wheels have now reacted to the tyre forces.
        # Feedf the resulting drivetrain speed back into the engine.
        self.update_engine(dt)

        # ----------------------------------------------------------
        # 5. Calculate chassis acceleration
        # ----------------------------------------------------------

        from sim_core.physics import calculate_chassis_dynamics

        acceleration_x, acceleration_y, yaw_acceleration = (
            calculate_chassis_dynamics(
                wheels=self.wheels,
                mass=self.total_mass,
                yaw_inertia=self.yaw_inertia
            )
        )

        self.physics.acceleration_x = acceleration_x
        self.physics.acceleration_y = acceleration_y
        self.physics.yaw_acceleration = yaw_acceleration

        # ----------------------------------------------------------
        # 6. Integrate acceleration into velocity
        # ----------------------------------------------------------

        self.physics.velocity_x += (
            self.physics.acceleration_x * dt
        )

        self.physics.velocity_y += (
            self.physics.acceleration_y * dt
        )

        # ----------------------------------------------------------
        # 7. Integrate velocity into position
        # ----------------------------------------------------------

        self.physics.position_x += (
            self.physics.velocity_x * dt
        )

        self.physics.position_y += (
            self.physics.velocity_y * dt
        )

        # ----------------------------------------------------------
        # 8. Integrate yaw motion
        # ----------------------------------------------------------

        self.physics.yaw_rate += (
            self.physics.yaw_acceleration * dt
        )

        self.physics.heading += (
            self.physics.yaw_rate * dt
        )

        # ----------------------------------------------------------
        # 9. Apply engine RPM limiter
        # ----------------------------------------------------------

        self.engine.apply_rev_limiter()
