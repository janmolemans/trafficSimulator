from .vehicle import Vehicle
from numpy.random import randint

class VehicleGenerator:
    def __init__(self, config={}):
        # Set default configurations
        self.set_default_config()

        # Update configurations
        for attr, val in config.items():
            setattr(self, attr, val)

        # Calculate properties
        self.init_properties()

    def set_default_config(self) -> None:
        """Set default configuration for the vehicle generator."""
        self.vehicle_rate = 10
        self.vehicles = [
            (1, {})
        ]
        self.last_added_time = 0

    def init_properties(self) -> None:
        """Initialize generator properties including the upcoming vehicle."""
        self.upcoming_vehicle = self.pick_vehicle()

    def pick_vehicle(self) -> "Vehicle":
        """
        Randomly select a vehicle based on the defined weights and return an instance.
        """
        total = sum(weight for weight, _ in self.vehicles)
        r = randint(1, total + 1)
        for weight, config in self.vehicles:
            r -= weight
            if r <= 0:
                return Vehicle(config)

    def update(self, simulation) -> None:
        """Add vehicles into the simulation if the appropriate time has elapsed."""
        if simulation.t - self.last_added_time >= 60 / self.vehicle_rate:
            print('adding vehicle')
            # If time elasped after last added vehicle is
            # greater than vehicle_period; generate a vehicle
            segment = simulation.segments[self.upcoming_vehicle.path[0]]      
            if len(segment.vehicles) == 0\
               or simulation.vehicles[segment.vehicles[-1]].x > self.upcoming_vehicle.s0 + self.upcoming_vehicle.l:
                # If there is space for the generated vehicle; add it
                simulation.add_vehicle(self.upcoming_vehicle)
                # Reset last_added_time and upcoming_vehicle
                self.last_added_time = simulation.t
            self.upcoming_vehicle = self.pick_vehicle()
