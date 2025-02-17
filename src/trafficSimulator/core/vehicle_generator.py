import numpy as np
from .vehicle import Vehicle
from numpy.random import randint

class VehicleGenerator:
    def __init__(self, config={}):
        # Set default configurations
        self.set_default_config()

        # Update configurations
        self.vehicle_rate = config.get('vehicle_rate', self.vehicle_rate)
        self.vehicles = config.get('vehicles', self.vehicles)

        # Calculate properties
        self.upcoming_vehicle = self.pick_vehicle()

    def set_default_config(self) -> None:
        """Set default configuration for the vehicle generator."""
        self.vehicle_rate = 100
        self.vehicles = [
            (1, {'path': [0],  'lane': randint(0, 3)}),
        ]
        self.last_added_time = 0
        # New: track the next spawn time using exponential
        self.next_spawn_time = np.random.exponential(60.0 / self.vehicle_rate)

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
        if simulation.t >= self.next_spawn_time:
            segment = simulation.segments[self.upcoming_vehicle.path[0]]
            chosen_lane = None
            # Find the first lane with enough space for the vehicle
            for ln in range(segment.num_lanes):
                lane_vehicles = [
                    simulation.vehicles[vid]
                    for vid in segment.vehicles
                    if simulation.vehicles[vid].lane == ln
                ]
                if not lane_vehicles or lane_vehicles[-1].x > self.upcoming_vehicle.s0 + self.upcoming_vehicle.l:
                    chosen_lane = ln
                    break
            if chosen_lane is not None:
                self.upcoming_vehicle.lane = chosen_lane
                simulation.add_vehicle(self.upcoming_vehicle)
            self.upcoming_vehicle = self.pick_vehicle()
            self.last_added_time = simulation.t
            # Resample next spawn time
            self.next_spawn_time = simulation.t + np.random.exponential(60.0 / self.vehicle_rate)
