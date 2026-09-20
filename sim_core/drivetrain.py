"""
drivetrain.py

Contains the Drivetrain class.

The Drivetrain coordinates the engine, clutch, gearbox and the
transfer of engine torque to the driven wheels.

Vehicle remains responsible for the overall vehicle simulation
while Drivetrain handles the powertrain.
"""


import math
from typing import TYPE_CHECKING

from sim_core.engine import Engine, Gearbox

# allows us to type-hint wheels without creating an import loop
if TYPE_CHECKING:
    from sim_core.tyre_wheel import Wheel




class Drivetrain:
    """
    Manage the vehicles, engine, clutch, gearbox and drive torque.
    """

    def __init__(
            self,
            engine: Engine,
            gearbox: Gearbox,
            driven_wheel_ids: list[int],
    ) -> None:
        self.engine = engine
        self.gearbox = gearbox
        self.driven_wheel_ids = driven_wheel_ids

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


    def apply_driver_input(self, throttle: float) -> None:
        """
        Apply accelerator input to the engine.
        
        The drivetrain temporarily cuts throttle during an automatic
        gear change to allow the clutch to disengage.
        """

        if self.clutch_engaged:
            self.shift_throttle_cut = 0.0
        else:
            self.shift_throttle_cut = 1.0

        self.engine.throttle = (
            throttle
            * (1.0 - self.shift_throttle_cut)
        )


    def calculate_clutch_torque(
        self,
        wheels: list["Wheel"],
    ) -> float:
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
            self.get_drivetrain_angular_velocity(wheels)
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


    def get_drivetrain_angular_velocity(
        self,
        wheels: list["Wheel"],
    ) -> float:
        """
        Calculate the average angular velocity of the driven wheels
        and convert it through the current gearbox ratio.

        Returns:
            Equivalent engine angular velocity in rad/s.
        """

        driven_wheels = [
            wheels[wheel_id]
            for wheel_id in self.driven_wheel_ids
        ]

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
        
        # *** TEMP TEST DIAGNOSTIC ***
        print(
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


    def distribute_drive_torque(
        self,
        wheels: list["Wheel"]
    ) -> None:
        """
        Divide drivetrain torque equally between driven wheels.

        This prevents each driven wheel from incorrectly receiving
        the entire engine output.

        A real differential is more complicated, but equal torque
        distribution is a useful starting point.
        """

        driven_wheel_count = len(self.driven_wheel_ids)

        if driven_wheel_count == 0:
            return

        total_drive_torque = self.get_drivetrain_torque()

        torque_per_wheel = (
            total_drive_torque / driven_wheel_count
        )

        for wheel_index in self.driven_wheel_ids:
            wheels[wheel_index].drive_torque = torque_per_wheel

