"""
visualizer.py
"""


import math
import pygame

from sim_core.vehicle import Vehicle
from sim_core.drag_race import DragRace
from sim_core.math_utils import metres_per_second_to_kph



class Visualizer:
    """
    Display the vehicle simulation from a top-down view.
    """

    def __init__(
            self,
            width: int = 1200,
            height: int = 700,
            pixels_per_meter: float = 10.0,
    ) -> None:

        pygame.init()

        self.width = width
        self.height = height
        self.pixels_per_meter = pixels_per_meter

        self.screen = pygame.display.set_mode(
            (self.width, self.height)
        )

        pygame.display.set_caption(
            "Vehicle Dynamics Simulator"
        )

        self.font = pygame.font.Font(None, 28)

        self.clock = pygame.time.Clock()
        self.running = True


    def handle_events(self) -> None:
        """
        Process window events.
        """

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False


    def world_to_screen(
            self,
            x: float,
            y: float,
    ) -> tuple[int, int]:
        """
        Convert simulation coordinates in metres to screen pixels.

        The simulation uses a Cartesian coordinate system
        where positive Y points upwards. Pygame's screen coordinates have
        positive Y pointing downwards, so the Y coordinate is inverted.
        """

        screen_x = int(
            self.width / 2
            + x * self.pixels_per_meter
        )

        screen_y = int(
            self.height / 2
            - y * self.pixels_per_meter
        )

        return screen_x, screen_y


    def draw_car(
            self,
            car: Vehicle,
    ) -> None:
        """
        Draw a vehicle from a top-down view.
        """

        car_length = int(4.40 * self.pixels_per_meter)
        car_width = int(1.80 * self.pixels_per_meter)

        car_surface = pygame.Surface(
            (car_length, car_width),
            pygame.SRCALPHA,
        )

        if car.name == "Sporty Engine Car" or car.name == "RWD Car":
            car_colour = (220, 40, 40)
        else:
            car_colour = (40, 120, 220)

        pygame.draw.rect(
            car_surface,
            car_colour,
            (
                0,
                0,
                car_length,
                car_width,
            ),
            border_radius=8,
        )

        # Mark the front of the car so that its heading is visible.
        pygame.draw.polygon(
            car_surface,
            (240, 240, 240),
            [
                (car_length, car_width / 2),
                (car_length - 20, car_width / 2 - 10),
                (car_length - 20, car_width / 2 + 10),
            ],
        )

        rotated_surface = pygame.transform.rotate(
            car_surface,
            math.degrees(car.physics.heading),
        )

        screen_position = self.world_to_screen(
            car.physics.position_x,
            car.physics.position_y,
        )

        rotated_rectangle = rotated_surface.get_rect(
            center=screen_position
        )

        self.screen.blit(
            rotated_surface,
            rotated_rectangle,
        )

    def draw_vehicle_info(
            self,
            car: Vehicle,
            race: DragRace,
            simulation_time: float,
            screen_position: tuple[int, int] = (20, 20),
    ) -> None:
        """
        Draw vehicle information on the screen.
        """

        speed = metres_per_second_to_kph(
            math.sqrt(
                car.physics.velocity_x ** 2
                + car.physics.velocity_y ** 2
            )
        )

        if car in race.finish_times:
            display_time = race.finish_times[car]
        else:
            display_time = simulation_time

        text = (
            f"{car.name}\n"
            f"Time: {display_time:.2f} s\n"
            f"Distance: {car.physics.position_x:.2f} m\n"
            f"Speed: {speed:.2f} km/h\n"
            f"Gear: {car.drivetrain.gearbox.current_gear}\n"
            f"RPM: {car.drivetrain.engine.rpm:.0f}\n"
        )

        text_surface = self.font.render(
            text,
            True,
            (240, 240, 240),
        )

        self.screen.blit(
            text_surface,
            screen_position,
        )


    def draw(
            self,
            cars: list[Vehicle],
            race: DragRace,
            simulation_time: float,
    ) -> None:
        """
        Draw the current simulation state.
        """

        self.screen.fill((35, 35, 35))

        for car in cars:
            self.draw_car(car)

        if cars:
            self.draw_vehicle_info(cars[0], simulation_time, race, (20, 20))
            if len(cars) > 1:
                self.draw_vehicle_info(cars[1], simulation_time, race, (500, 20))

        pygame.display.flip()


    def close(self) -> None:
        """
        Shut dow Pygame cleanly
        """

        pygame.quit()

