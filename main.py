"""
main.py
"""

from sim_core.driver_input import DriverInput
from sim_core.engine import Engine, GEARBOX_5_SPEED
from sim_core.settings import SPORTY_PETROL_ENGINE
from sim_core.vehicle import Vehicle


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
        gearbox=GEARBOX_5_SPEED
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
                f"Rear wheel speed={vehicle.wheels[2].get_surface_speed():.2f} m/s | "
                f"Rear slip={vehicle.wheels[2].slip_ratio:.2f} | "
                f"Rear force={vehicle.wheels[2].longitudinal_force:.2f} N"
            )

    print(f"Simulation time: {simulation_time:.2f} s")
    print(f"Position X: {vehicle.physics.position_x:.2f} m")
    print(f"Position Y: {vehicle.physics.position_y:.2f} m")
    print(f"Acceleration X: {vehicle.physics.acceleration_x:.2f} m/s²")
    print(f"Engine RPM: {vehicle.engine.rpm:.0f}")
    print(f"Heading: {vehicle.physics.heading:.3f} rad")
    print(f"Engine torque: {vehicle.engine.get_available_torque():.2f} Nm")
    print(f"Clutch torque: {vehicle.calculate_clutch_torque():.2f} Nm")
    print(f"Drivetrain torque: {vehicle.get_drivetrain_torque():.2f} Nm")
    print(f"Gear ratio: {vehicle.gearbox.get_total_ratio():.2f}")

    print()
    print("FINAL WHEEL STATE")
    print("=" * 40)

    for wheel in vehicle.wheels:
        print(
            f"{wheel.name}: "
            f"angular velocity={wheel.angular_velocity:.2f} rad/s, "
            f"surface speed={wheel.get_surface_speed():.2f} m/s, "
            f"slip={wheel.slip_ratio:.2f}, "
            f"longitudinal force={wheel.longitudinal_force:.2f} N"
        )

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


def main() -> None:
    """
    Just testing for now.
    """

    vehicle_test()



if __name__ == "__main__":
    main()
