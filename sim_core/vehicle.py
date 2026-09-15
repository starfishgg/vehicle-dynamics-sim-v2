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

        # Centre of gravity height
        self.cg_height: float = 0.55 # metres

        # Centre of mass
        self.centre_of_mass = 0.5 # exact halfway point

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
        self.clutch_capacity: float = 500.0

        # Controls how strongly the clutch reacts to speed
        # difference between the engine and drivetrain.
        self.clutch_stiffness: float = 5.0

        # The automatic clutch starts fully engaged.
        self.clutch_engaged: bool = True

        # During a gear change the clutch is initially released,
        # then gradually re-engaged
        self.clutch_engagement: float = 1.0

        # Duration of the automatic clutch release during a shift.
        self.clutch_shift_duration: float = 0.15

        self.shift_throttle_cut: float = 0.0

        self.clutch_torque: float = 0.0

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

        When the clutch is engaged, torque is transmitted in whichever
        direction is required to reduce the speed difference between
        the engine and drivetrain.

        Positive torque means the engine is driving the drivetrain.

        Negative torque means the drivetrain is driving the engine.

        Positive torque is limited by the engine's available torque.
        Negative torque is limited by the clutch capacity.
        """
        if not self.clutch_engaged:
            return 0.0

        drivetrain_angular_velocity = (
            self.get_drivetrain_angular_velocity()
        )

        engine_angular_velocity = (
            self.engine.rpm * 2.0 * math.pi / 60.0
        )

        relative_angular_velocity = (
            engine_angular_velocity
            - drivetrain_angular_velocity
        )

        coupling_torque = (
            relative_angular_velocity
            * self.clutch_stiffness
            * self.clutch_engagement
        )

        if coupling_torque >= 0.0:
            maximum_torque = min(
                coupling_torque,
                self.engine.get_available_torque(),
                self.clutch_capacity * self.clutch_engagement,
            )

            return maximum_torque

        return max(
            coupling_torque,
            -self.clutch_capacity * self.clutch_engagement,
        )



    def update_clutch(self, dt: float) -> None:
        """
        Update the automatic clutch state.
        
        During an automatic gear change the cluth remains released
        for a short peroid. Once that peroid has elapsed, the clutch
        engages again.
        """
        if self.clutch_engaged:
            return

        engagement_rate = 1.0 / self.clutch_shift_duration

        self.clutch_engagement += engagement_rate * dt
        if self.clutch_engagement >= 1.0:
            self.clutch_engagement = 1.0
            self.clutch_engaged = True


    def get_drivetrain_torque(self) -> float:
        """
        Calculate the torque delivered to the drivetrain.

        The clutch determines how much of the engine torque is
        transmitted to the gearbox. The gearbox and final drive
        then multiply the transmitted torque.

        Returns:
            Torque delivered to the driven wheels in Nm.
        """

        total_gear_ratio = self.gearbox.get_total_ratio()
        drivetrain_efficiency = 0.90
        

        return (
            self.clutch_torque
            * total_gear_ratio
            * drivetrain_efficiency
        )


    def update_engine(self, dt: float) -> None:
        """
        Update engine RPM from engine torque and clutch load.

        The clutch torque was already calculated earlier in this
        simulation timestep and is reused here.

        When the clutch is released during a gear change, the stored
        clutch torque will be zero.
        """
        self.engine.update_rpm(
            load_torque=self.clutch_torque,
            dt=dt
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

        if self.clutch_engaged:
            self.shift_throttle_cut = 0.0
        else:
            self.shift_throttle_cut = 1.00

        # Transfer accelerator input to the engine.
        self.engine.throttle = (
            self.driver_input.throttle
            * (1.0 - self.shift_throttle_cut)
        )

        # Apply the requested gear.

        # This is deliberately simple for now.
        # We are not simulating clutch movement or gear-shift timing yet
        # self.gearbox.current_gear = self.driver_input.gear_request

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
        # 3. Calculate cluth torque once for this timestep
        # ----------------------------------------------------------
        self.clutch_torque = self.calculate_clutch_torque()


        # ----------------------------------------------------------
        # 4. Apply engine torque to driven wheels
        # ----------------------------------------------------------

        self.distribute_drive_torque()

        # ----------------------------------------------------------
        # 5. Calculate tyre forces
        # ----------------------------------------------------------

        self.calculate_wheel_forces()

        for wheel in self.wheels:
            wheel.update_rotation(
                brake_torque=0.0,
                dt=dt
            )

        # The wheels have now reacted to the tyre forces.
        # Feed the resulting drivetrain speed back into the engine.
        self.update_engine(dt)

        # Check whether an automatic gear change is required.
        self.update_automatic_shift()

        # Update teh autommatic clutch state.
        self.update_clutch(dt)

        # ----------------------------------------------------------
        # 6. Calculate chassis acceleration
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
        # 7. Integrate acceleration into velocity
        # ----------------------------------------------------------

        self.physics.velocity_x += (
            self.physics.acceleration_x * dt
        )

        self.physics.velocity_y += (
            self.physics.acceleration_y * dt
        )

        # ----------------------------------------------------------
        # 8. Integrate velocity into position
        # ----------------------------------------------------------

        self.physics.position_x += (
            self.physics.velocity_x * dt
        )

        self.physics.position_y += (
            self.physics.velocity_y * dt
        )

        # ----------------------------------------------------------
        # 9. Integrate yaw motion
        # ----------------------------------------------------------

        self.physics.yaw_rate += (
            self.physics.yaw_acceleration * dt
        )

        self.physics.heading += (
            self.physics.yaw_rate * dt
        )

        # ----------------------------------------------------------
        # 10. Apply engine RPM limiter
        # ----------------------------------------------------------

        self.engine.apply_rev_limiter()


    def update_automatic_shift(self) -> None:
        """
        Shift up automatically when the engine approaches redline.

        The clutch is temporarily released during the gear change so
        the engine RPM can fall to match the new gear ratio before
        the drivetrain is coupled again.
        """
        if not self.clutch_engaged:
            return
    
        shift_rpm = self.engine.redline * 0.90

        if self.engine.rpm < shift_rpm:
            return
        
        current_gear = self.gearbox.current_gear
        next_gear = current_gear + 1

        if next_gear not in self.gearbox.ratios:
            return

        if not self.is_drivetrain_transmitting_torque():
            return
        
        print(
            f"{self.name}: "
            f"GEAR SHIFT {current_gear} -> {next_gear}"
        )

        if next_gear in self.gearbox.ratios:
            self.gearbox.shift_up()
            self.clutch_engaged = False
            self.clutch_engagement = 0.0


    def is_drivetrain_transmitting_torque(self) -> bool:
        if not self.clutch_engaged:
            return False

        if self.gearbox.get_total_ratio() == 0.0:
            return False

        if self.clutch_torque <= 0.0:
            return False

        return True

    