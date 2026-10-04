"""
main.py
"""

from sim_core.driver_input import DriverInput
from sim_core.engine import Engine, create_5_speed_gearbox
from sim_core.tyre_wheel import Wheel
from sim_core.settings import (
    SPORTY_PETROL_ENGINE, NORMAL_PETROL_ENGINE,
    FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT,
)
from sim_core.vehicle import Vehicle
from sim_core.drag_race import DragRace
from sim_core.math_utils import metres_per_second_to_kph
from sim_core.visualizer import Visualizer




def main() -> None:

    throttle_setting: float = 1.0

    driver_input_1 = DriverInput(
        throttle=throttle_setting,
        brake=0.0,
        steering=0.0,
        gear_request=1,
    )

    driver_input_2 = DriverInput(
        throttle=throttle_setting,
        brake=0.0,
        steering=0.0,
        gear_request=1,
    )

    normal_engine = Engine(NORMAL_PETROL_ENGINE)
    normal_engine_2 = Engine(NORMAL_PETROL_ENGINE)
    sporty_engine = Engine(SPORTY_PETROL_ENGINE)

    car_1 = Vehicle(
        driver_input_1,
        normal_engine,
        gearbox=create_5_speed_gearbox(),
        rwd=False,
        name="FWD Car",
    )

    car_2 = Vehicle(
        driver_input_2,
        normal_engine_2,
        gearbox=create_5_speed_gearbox(),
        rwd=True,
        name="RWD Car",
    )

    car_1.physics.position_y =  2.0
    car_2.physics.position_y = -2.0

    race = DragRace(
        cars=[car_1, car_2],
        finish_distance=402.336, # a quarter of a mile
    )

    # Visualizer
    visualizer = Visualizer()

    physics_time_step = 0.0001
    accumulator = 0.0
    simulation_time = 0.0

    while visualizer.running and not race.finished:
        # Limit rendering to 60 frames per second.
        frame_time = visualizer.clock.tick(60) / 1000.0

        # Add the real time that has passed since the previous frame.
        accumulator += frame_time

        visualizer.handle_events()

        # Run the physics using the fixed timestep.
        while accumulator >= physics_time_step:
            race.update(physics_time_step)
            accumulator -= physics_time_step
            simulation_time += physics_time_step

        visualizer.draw(
            race.cars,
            simulation_time,
            race,
        )

    print()
    print(f"Throttle Setting: {driver_input_1.throttle}")
    if race.finished:
        for car, finish_time in race.get_results():
            print(f"{car.name}: {finish_time:.2f} seconds")
    print()
    print(f"WINNER: {race.winner.name}")

    visualizer.close()




if __name__ == "__main__":
    main()
