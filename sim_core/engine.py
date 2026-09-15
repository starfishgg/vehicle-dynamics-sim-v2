"""
engine.py

Contains the Engine and Gearbox classes.

Engine is responsible for the engine's current operating state,
such as RPM and throttle-related torque.

Gearbox is responsible for gear ratios and the currently selected gear.

The engine does not know which gear the vehicle is using.
That is the gearbox's responsibility.
"""

from typing import Mapping

from sim_core.settings import PETROL, DIESEL, SPORTY_PETROL_ENGINE
from sim_core.math_utils import (
    torque_curve,
    calculate_rpm_change,
    clamp,

)



class Engine:
    """
    Represents a vehicle engine.

    The engine configuration is supplied as a dictionary from settings.py.

    Example:

        engine = Engine(SPORTY_PETROL_ENGINE)
    """

    def __init__(
            self,
            settings: Mapping[str, float]
    ) -> None:

        # --------------------------------------------------------
        # Engine configuration
        # --------------------------------------------------------

        self.idle_rpm:      float = settings["idle_rpm"]
        self.peak_torque:   float = settings["peak_torque"]
        self.base_torque:   float = settings["base_torque"]
        self.peak_rpm:      float = settings["peak_rpm"]

        # These control the shape of the torque curve.
        self.rise_width: float = settings["rise_width"]
        self.fall_width: float = settings["fall_width"]

        self.redline: float = settings["redline"]

        # Torque and resistance characteristics.
        self.idle_torque:     float = settings["idle_torque"]
        self.engine_braking:  float = settings["engine_braking"]
        self.friction_torque: float = settings["friction_torque"]

        # Rotational inertia in kg-m².    
        #
        # A lower value means the engine RPM changes more quickly.
        # A higher value means the engine RPM changes more slowly.
        self.inertia: float = settings["inertia"]

        # --------------------------------------------------------
        # Current engine state
        # --------------------------------------------------------
        
        # Start the engine at idle rather than an arbitrary RPM.
        self.rpm:float = self.idle_rpm

        # Throttle is expected to be between 0.0 and 1.0.
        #
        # 0.0 = no throttle
        # 1.0 = full throttle
        self.throttle: float = 0.0


    def get_torque(self) -> float:
        """
        Return the engine's available torque at its current RPM.

        This is the torque produced by the engine before drivetrain
        losses and gearbox multiplication are applied.

        Returns:
            Engine torque in Nm.
        """

        return torque_curve(
            rpm=self.rpm,
            peak_torque=self.peak_torque,
            base_torque=self.base_torque,
            peak_rpm=self.peak_rpm,
            rise_width=self.rise_width,
            fall_width=self.fall_width,
        )


    def get_friction_torque(self) -> float:
        """
        Return the engine's internal friction torque.
        
        At this stage we use a constant value from the engine settings.
        
        Returns:
            Internal resistance torque in Nm.
        """

        return self.friction_torque


    def get_available_torque(self) -> float:
        """
        Return torque after applying throttle.

        Example:

            0.0 throttle = 0% of available torque
            0.5 throttle = 50% of available torque
            1.0 throttle = 100% of available torque

        Engine torque is cut when the redline is reached.
        This acts as a simple rev limiter.

        Returns:
            Throttle-adjusted engine torque in Nm.
        """

        # Cut engine torque when the redline is reached.
        # This prevents the engine from continuing to accelerate
        # the drivetrain beyond its maximum operating speed.
        if self.rpm >= self.redline:
            return 0.0
        
        return self.get_torque() * self.throttle


    def update_rpm(
            self,
            load_torque: float,
            dt: float,
    ) -> None:
        """
        Update engine RPM based on the current torque balance.

            The engine accelerates when its produced torque is greater
            than the torque resisting it.

            Engine rotational inertia determines how quickly the RPM
            changes.

            Args:
                load_torque:
                    Torque being demanded from the engine by the
                    drivetrain, measured at the engine.

                dt:
                    Simulation time step in seconds.
        """

        engine_torque = self.get_available_torque()

        self.rpm = calculate_rpm_change(
            rpm=self.rpm,
            engine_torque=engine_torque,
            load_torque=load_torque,
            engine_inertia=self.inertia,
            friction_torque=self.get_friction_torque(),
            dt=dt,
        )

        self.apply_rev_limiter()

        
    
    def apply_rev_limiter(self):
        """
        Keep the engine RPM inside its allowed operating range.

        This is a simple RPM clamp for now.

        Later, we can make the limiter more realistic by temporarily
        cutting engine torque when the redline is reached.
        """

        self.rpm = clamp(self.rpm, self.idle_rpm, self.redline)




class Gearbox:
    """
    Represents the vehicle gearbox.

    The gearbox stores gear ratios and the currently selected gear.

    Gear identifiers:

        -1 = reverse
         0 = neutral
         1+ = forward gears 
    """

    def __init__(
            self, 
            ratios: Mapping[int, float],
            final_drive: float = 5.0
    ) -> None:
        """
        Initialise the gearbox.

        Args:
            ratios:
                Dictionary mapping gear numbers to gear ratios.

            final_drive:
                Final drive ratio multiplying gearbox output torque.
        """
        self.ratios: Mapping[int, float] = ratios
        self.final_drive: float = final_drive
        self.current_gear:int = 1


    def get_ratio(self, gear: int | None = None) -> float:
        """
        Return the ratio for the requested gear.

        If no gear is supplied, use the currently selected gear.

        Unknown gears return 0.0, which behaves like neutral.
        """

        if gear is None:
            gear = self.current_gear

        return self.ratios.get(gear, 0.0)


    def get_total_ratio(self) -> float:
        """
        Return the gearbox ratio multiplied by the final drive.
        
        This is the total torque multiplication between the engine and the driven wheels.
        """

        return self.get_ratio() * self.final_drive


    def shift_up(self) -> None:
        """
        Shift up to the next available forward gear.

        If the gearbox is already in the highest forward gear,
        no change is made.
        """

        next_gear = self.current_gear + 1

        if next_gear in self.ratios:
            self.current_gear = next_gear




def create_5_speed_gearbox() -> Gearbox:
    return Gearbox(
        {
            -1: -2.92, # Reverse
            0:  0.00, # Neutral
            1:  2.50,
            2:  1.61,
            3:  1.10,
            4:  0.81,
            5:  0.68,
        },
        final_drive=5.0,
    )
