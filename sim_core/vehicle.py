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
from sim_core.drivetrain import Drivetrain
from sim_core.physics import PhysicsState
from sim_core.tyre_wheel import Wheel
from sim_core.settings import (
    PETROL_WEIGHT,
    SPORTS_TYRE,
    GRAVITY,
    FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT,
)
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
        name: str="Unknown",
    ) -> None:

        self.name=name

        # ----------------------------------------------------------
        # Driver controls
        # ----------------------------------------------------------
        
        self.driver_input = driver_input

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

        # Centre of gravity height
        self.cg_height: float = 0.55 # metres

        # Centre of mass
        self.centre_of_mass = 0.5 # exact halfway point

        # ----------------------------------------------------------
        # Chassis rotational inertia
        # ----------------------------------------------------------

        # Simplified yaw moment of inertia.
        self.yaw_inertia: float = 1800.0

        # ----------------------------------------------------------
        # Wheels
        # ----------------------------------------------------------
        self.wheels = self._create_wheels()

        # ----------------------------------------------------------
        # Powertrain
        # ----------------------------------------------------------

        driven_wheel_ids_list = self.get_driven_wheel_ids()

        self.drivetrain = Drivetrain(
            engine=engine,
            gearbox=gearbox,
            driven_wheel_ids=driven_wheel_ids_list,
        )

        # ----------------------------------------------------------
        # Physical state
        # ----------------------------------------------------------
        self.physics = PhysicsState()


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


    def get_driven_wheel_ids(self) -> list[int]:
        """
        Return all the ids of wheels receiving engine torque.
        """

        return [
            wheel_index
            for wheel_index, wheel in enumerate(self.wheels)
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


    def apply_driver_inputs(self) -> None:
        """
        Transfer the current driver commands into the vehicle systems.
        """

        # Keep all driver controls inside their valid ranges.
        self.driver_input.validate()

        # Apply accelerator input to the drivetrain.
        self.drivetrain.apply_driver_input(
            self.driver_input.throttle
            # self.drive_input.gear_request
        )

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
        Calculate the normal force acting on each wheel.

        This assumes:

        - The vehicle is on level ground.
        - The vehicle has four wheels.
        - The centre-of-mass position is expressed as a fraction
        of the wheelbase measured from the front axle.
        - Longitudinal acceleration causes weight transfer between
        the front and rear axles.
        - There is no aerodynamic downforce.

        Positive acceleration transfers load from the front axle
        to the rear axle.

        Negative acceleration, such as braking, transfers load from
        the rear axle to the front axle.
        """

        gravity = GRAVITY

        total_weight = (self.total_mass * gravity)


        # Static weight distribution before acceleration or braking.
        #
        # centre_of_mass = 0.0 means the centre of mass is directly
        # above the front axle.
        #
        # centre_of_mass = 1.0 means the centre of mass is directly
        # above the rear axle.
        #
        # For example, 0.5 means a 50/50 static weight distribution.
        static_front_load = total_weight * (1.0 - self.centre_of_mass)
        static_rear_load = total_weight * self.centre_of_mass


        # Longitudinal weight transfer caused by acceleration.
        #
        # Positive acceleration transfers load rearwards.
        # Negative acceleration transfers load forwards.
        load_transfer = (
            self.total_mass
            * self.physics.acceleration_x
            * self.cg_height
            / self.wheelbase
        )

        front_axle_load = static_front_load - load_transfer
        read_axle_load = static_rear_load + load_transfer


        # Assuming a four-wheeled vehicle with equel left/right
        # weight distribution.
        front_wheel_load = front_axle_load / 2.0
        rear_wheel_load = read_axle_load / 2.0


        self.wheels[FRONT_LEFT].normal_force = front_wheel_load
        self.wheels[FRONT_RIGHT].normal_force = front_wheel_load
        self.wheels[REAR_LEFT].normal_force = rear_wheel_load
        self.wheels[REAR_RIGHT].normal_force = rear_wheel_load



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
        # 3. Calculate clutch torque once for this timestep
        # ----------------------------------------------------------
        self.drivetrain.clutch_torque = self.drivetrain.calculate_clutch_torque(
            self.wheels
        )

        # ----------------------------------------------------------
        # 4. Apply engine torque to driven wheels
        # ----------------------------------------------------------
        self.drivetrain.distribute_drive_torque(
            self.wheels
        )

        # ----------------------------------------------------------
        # 5. Calculate tyre forces
        # ----------------------------------------------------------
        self.calculate_wheel_forces()

        # ----------------------------------------------------------
        # 6. Update wheel rotations
        # ----------------------------------------------------------
        for wheel in self.wheels:
            wheel.update_rotation(
                brake_torque=0.0,
                dt=dt
            )

        # The wheels have now reacted to the tyre forces.
        # Feed the resulting wheel speed back into the engine.
        self.drivetrain.update_engine(dt)

        # Check whether an automatic gear change is required
        self.drivetrain.update_automatic_shift()

        # Update the automatic clutch state
        self.drivetrain.update_clutch(dt)


        # ----------------------------------------------------------
        # 7. Calculate chassis acceleration
        # ----------------------------------------------------------
        from sim_core.physics import calculate_chassis_dynamics

        acceleration_x, acceleration_y, yaw_acceleration = (
            calculate_chassis_dynamics(
                wheels=self.wheels,
                mass=self.total_mass,
                yaw_inertia=self.yaw_inertia,
                vehicle_velocity_x=self.physics.velocity_x,
            )
        )

        self.physics.acceleration_x = acceleration_x
        self.physics.acceleration_y = acceleration_y
        self.physics.yaw_acceleration = yaw_acceleration

        # ----------------------------------------------------------
        # 8. Integrate acceleration into velocity
        # ----------------------------------------------------------

        self.physics.velocity_x += (
            self.physics.acceleration_x * dt
        )

        self.physics.velocity_y += (
            self.physics.acceleration_y * dt
        )

        # ----------------------------------------------------------
        # 9. Integrate velocity into position
        # ----------------------------------------------------------

        self.physics.position_x += (
            self.physics.velocity_x * dt
        )

        self.physics.position_y += (
            self.physics.velocity_y * dt
        )

        # ----------------------------------------------------------
        # 10. Integrate yaw motion
        # ----------------------------------------------------------

        self.physics.yaw_rate += (
            self.physics.yaw_acceleration * dt
        )

        self.physics.heading += (
            self.physics.yaw_rate * dt
        )

        # ----------------------------------------------------------
        # 11. Apply engine RPM limiter
        # ----------------------------------------------------------

        self.drivetrain.engine.apply_rev_limiter()

