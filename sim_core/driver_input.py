"""
driver_inputs.py

Contains DriverInput, which represents commands given
to a vehicles by a human or AI driver.
"""


from dataclasses import dataclass

from sim_core.math_utils import clamp




@dataclass
class DriverInput:
    """
    Represents driver controls.

    Values are normalised between -1.0 and 1.0
    unless otherwise stated
    """

    # Accelerator pedal
    # 0 = no throttle
    # 1 = fully pressed
    throttle: float = 0.0

    # Brake pedal
    # 0 = no brake
    # 1 = maximum braking
    brake: float = 0.0

    # Steering wheel
    # -1 = full left
    # 0 = centre
    # 1 - full right
    steering: float = 0.0

    # Manual trasmission controls
    clutch: float = 0.0

    # Requested gear
    # 0 = neutral
    # -1 = reverse
    gear_request: int = 1


    
    def validate(self) -> None:
        """
        Ensure driver inputs remain within valid ranges.
        """

        self.throttle = clamp(self.throttle, 0.0, 1.0)
        self.brake = clamp(self.brake, 0.0, 1.0)
        self.clutch = clamp(self.clutch, 0.0, 1.0)
        self.steering = clamp(self.steering, -1.0, 1.0)
        self.gear_request = int(clamp(self.gear_request, -1, 8))

