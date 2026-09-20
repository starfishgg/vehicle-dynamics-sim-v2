"""
drag_race.py

Contains the DragRace class.

DragRace manages a collection of vehicles competing in a straight-line drag race.

Each Vehicle remains responsible for its own engine, gearbox, wheels, tyre forces, and chassis physics. DragRace is responsible only for running the vehicles together and determining the race result.
"""


from sim_core.vehicle import Vehicle




class DragRace:
    """
    Manage a straight-line drag race between multiple vehicles
    """

    def __init__(
        self,
        cars: list[Vehicle],
        finish_distance: float = 400.0,
    ) -> None:
        """
        Initialise the drag race.

        ArgsL
            cars: Vehicles taking part in the race.
            finish_distance: Distance required to finish the race
            measured in metres.
        """

        if len(cars) < 2:
            raise ValueError("A drag race requires at least two cars.")

        if finish_distance <= 0.0:
            raise ValueError("Finish distance must be greater than zero.")

        self.cars = cars
        self.finish_distance = finish_distance

        self.elapsed_time: float = 0.0
        self.finished: bool = False
        self.winner: Vehicle | None = None

        self.finished_cars: set[Vehicle] = set()
        self.finish_times: dict[Vehicle, float] = {}

    def update(self, dt: float) -> None:
        """
        Advamce every car by one simulation step.
        
        All cars are updates using the same time step so that
        they remain part of the same simulation.
        """

        if dt <= 0.0:
            raise ValueError("dt must be greater than zero.")

        if self.finished:
            return

        for car in self.cars:
            if car not in self.finished_cars:
               car.update(dt)

        self.elapsed_time += dt

        self.check_finish()


    def check_finish(self) -> None:
        """
        Check whether any car has reached the finish distance.

        The first car to reach the finish line is declared
        the winner, but the race continues until every car has crossed the finish line.
        """

        for car in self.cars:
            if (
                car not in self.finished_cars
                and car.physics.position_x >= self.finish_distance
            ):
                self.finished_cars.add(car)
                self.finish_times[car] = self.elapsed_time

                if self.winner is None:
                    self.winner = car
                return

        if len(self.finished_cars) == len(self.cars):
            self.finished = True


    def get_positions(self) -> list[float]:
        """
        Return the current position of every car.
        
        The positions are returned in the same order as
        self.cars.
        """

        return [
            car.physics.position_x
            for car in self.cars
        ]


    def get_speeds(self) -> list[float]:
        """
        Return the current sped of every car.

        The speeds are returned in the same order as
        self.cars.
        """

        return [
            car.physics.velocity_x
            for car in self.cars
        ]


    def get_leader(self) -> Vehicle:
        """
        Return the car currently furthest along the track.
        
        """

        return max(
            self.cars,
            key=lambda car: car.physics.position_x
        )


    def get_results(self) -> list[tuple[Vehicle, float]]:
        """
        Return the race results ordered by finish time.
        """

        return sorted(
            self.finish_times.items(),
            key=lambda result: result[1],
        )

