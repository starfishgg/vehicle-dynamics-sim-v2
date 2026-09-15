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
            car.update(dt)

        self.elapsed_time += dt

        self.check_finish()


    def check_finish(self) -> None:
        """
        Check whether any car has reached the finish distance.

        The first car to reach the finish line is declared
        the winner.
        """

        for car in self.cars:
            if car.physics.position_x >= self.finish_distance:
                self.finished = True
                self.winner = car
                return


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

