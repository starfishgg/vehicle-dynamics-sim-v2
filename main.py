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



def vehicle_test():
    print("VEHICLE UPDATE TEST")
    print("=" * 40)

    driver_input = DriverInput(
        throttle=1.0,
        brake=0.0,
        steering=0.0,
        gear_request=1,
    )

    engine = Engine(SPORTY_PETROL_ENGINE)

    vehicle = Vehicle(
        driver_input=driver_input,
        engine=engine,
        gearbox=create_5_speed_gearbox()
    )

    vehicle.apply_driver_inputs()

    print(
        f"Initial drivetrain torque: "
        f"{vehicle.get_drivetrain_torque():.2f} Nm"
    )

    vehicle.calculate_normal_forces()
    vehicle.distribute_drive_torque()

    print()

    for wheel in vehicle.wheels:
        print(
            f"{wheel.name}: "
            f"normal force={wheel.normal_force:.2f} N, "
            f"torque={wheel.drive_torque:.2f} Nm, "
            f"slip={wheel.slip_ratio:.2f}, "
            f"longitudinal force={wheel.longitudinal_force:.2f} N"
        )

    print()

    time_step = 0.001
    simulation_time = 5.0
    steps = int(simulation_time / time_step)

    for step in range(steps):
        vehicle.update(time_step)

        if step % 100 == 0:
            print(
                f"Time={step * time_step:.2f}s | "
                f"Vehicle speed={vehicle.physics.velocity_x:.2f} m/s | "
            )
            for wheel in vehicle.wheels:
                print(
                    f"{wheel.name}: "
                    f"normal={wheel.normal_force:.2f} N, "
                    f"surface={wheel.get_surface_speed():.2f} m/s, "
                    f"slip={wheel.slip_ratio:.3f}, "
                    f"force={wheel.longitudinal_force:.2f} N"
                    
                )
            print(
                f"Front axle load: "
                f"{vehicle.wheels[FRONT_LEFT].normal_force + vehicle.wheels[FRONT_RIGHT].normal_force:.2f} N"
            )
            print(
                "Rear axle load: "
                f"{vehicle.wheels[REAR_LEFT].normal_force + vehicle.wheels[REAR_RIGHT].normal_force:.2f} N"
            )

    print()
    print("FINAL STATE")
    print("=" * 40)

    print(f"Simulation time: {simulation_time:.2f} s")
    print(f"Position X: {vehicle.physics.position_x:.2f} m")
    print(f"Position Y: {vehicle.physics.position_y:.2f} m")
    print(f"Acceleration X: {vehicle.physics.acceleration_x:.2f} m/s²")
    print(f"Engine RPM: {vehicle.engine.rpm:.0f}")
    print(f"Heading: {vehicle.physics.heading:.3f} rad")
    print(f"Engine torque: {vehicle.engine.get_available_torque():.2f} Nm")
    print(f"Engine torque delivered: {vehicle.calculate_clutch_torque():.2f} Nm")
    print(f"Drivetrain torque: {vehicle.get_drivetrain_torque():.2f} Nm")
    print(f"Gear ratio: {vehicle.gearbox.get_total_ratio():.2f}")



    print()


def engine_rpm_test():
    print("ENGINE RPM TEST")
    print("=" * 40)

    engine = Engine(SPORTY_PETROL_ENGINE)
    engine.throttle = 1.0

    time_step = 0.01
    simulation_time = 1.0
    steps = int(simulation_time / time_step)

    print(f"Starting RPM: {engine.rpm:.0f}")
    print()

    for step in range(steps):
        engine.update_rpm(
            load_torque=0.0,
            dt=time_step,
        )

        if step % 10 == 0:
            print(
                f"Time: {step * time_step:.2f} s | "
                f"RPM: {engine.rpm:.0f} | "
                f"Torque: {engine.get_torque():.1f} Nm"
            )

    print()
    print(f"Final RPM: {engine.rpm:.0f}")


def drag_race() -> None:

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
    sporty_engine = Engine(SPORTY_PETROL_ENGINE)

    car_1 = Vehicle(
        driver_input_1,
        normal_engine,
        gearbox=create_5_speed_gearbox(),
        name="Normal Engine Car",
    )
    car_2 = Vehicle(
        driver_input_2,
        sporty_engine,
        gearbox=create_5_speed_gearbox(),
        name="Sporty Engine Car",
    )

    race = DragRace(
        cars=[car_1, car_2],
        finish_distance=40.0
    )

    output_interval: float = 0.1 #1.0
    next_output_time: float = output_interval
    physics_updates: float = 0.001

    car_id: int = 1

    print("=" * 60)
    print(f"{race.finish_distance}m DRAG RACE | THROTTLE: {driver_input_1.throttle:.1f} | CAR {car_id} | Interval: {output_interval} | PhyUpdates: {physics_updates}")
    print("=" * 60)
    print()
    print(" Time  |   m/s   |   km/h   |   RPM   | Throttle | Raw Torque | Avail Torque | Drivetrain Torque | Gear | Wheel Speed")
    #for car in race.cars:
    #    print(f"{car.name:>30}", end="")
    #print()
    print("-" * 120)



    while not race.finished:
        race.update(physics_updates)

        if race.elapsed_time >= next_output_time:

            raw_engine_torque = race.cars[car_id].drivetrain.engine.get_torque()
            available_engine_torque = race.cars[car_id].drivetrain.engine.get_available_torque()
            drivetrain_torque = race.cars[car_id].drivetrain.get_drivetrain_torque()

            print(f"{race.elapsed_time:4.1f}s  ", end="")
            positions = race.get_positions()
            #print(f"{positions[0]:.1f}m  ", end="")
            print(f"{race.cars[car_id].physics.velocity_x:6.2f}m/s  ", end="")
            print(f"{metres_per_second_to_kph(race.cars[car_id].physics.velocity_x):6.2f}km/h  ", end="")
            print(f"{race.cars[car_id].drivetrain.engine.rpm:7.2f} rpm ", end="")
            print(f"{race.cars[car_id].drivetrain.engine.throttle:5.2f}  ", end="")
            print(f"{raw_engine_torque:10.2f} Nm ", end="")
            print(f"{available_engine_torque:10.2f} Nm ", end="")
            print(f"{drivetrain_torque:10.2f} Nm  ", end="")
            print(f"{race.cars[car_id].drivetrain.gearbox.current_gear:8} ", end="")
            print(f"{race.cars[car_id].wheels[FRONT_LEFT].get_surface_speed():10.2f} m/s")

            print(
                f"slip={race.cars[car_id].wheels[FRONT_LEFT].slip_ratio}"
                f"long-tf={race.cars[car_id].wheels[FRONT_LEFT].longitudinal_force}  "
                f"normal={race.cars[car_id].wheels[FRONT_LEFT].normal_force}  "
                f"drive_torque={race.cars[car_id].wheels[FRONT_LEFT].drive_torque}  ", end=""

            )
            #for position, car in zip(positions, race.cars):
            #    print(f"{position:10.2f} m  ", end="")
            #    print(f"{car.physics.velocity_x:5.2f} m/s ", end="")
            #    print(f"{car.engine.rpm:5.0f} rpm", end="")

            print()
            next_output_time += output_interval
            
            
    print()
    if race.finished:
        for car, finish_time in race.get_results():
            print(f"{car.name}: {finish_time:.2f} seconds")
    print()
    print(f"WINNER: {race.winner.name}")




def main() -> None:
    """
    Just testing for now.
    """

    # vehicle_test()
    drag_race()




if __name__ == "__main__":
    main()
