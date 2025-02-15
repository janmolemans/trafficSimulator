from .vehicle_generator import VehicleGenerator
from .geometry.quadratic_curve import QuadraticCurve
from .geometry.cubic_curve import CubicCurve
from .geometry.segment import Segment
from .vehicle import Vehicle


class Simulation:
    """
    Core engine for managing vehicles, segments, and vehicle generators.
    """
    def __init__(self) -> None:
        self.segments = []
        self.vehicles = {}
        self.vehicle_generator = []

        self.t = 0.0
        self.frame_count = 0
        self.dt = 1/60  
        # New lists to record events:
        self.departures = []
        self.arrivals = []


    def add_vehicle(self, veh) -> None:
        """Add a vehicle to the simulation and place it on its starting segment."""
        self.vehicles[veh.id] = veh
        # Record departure time when vehicle is added.
        veh.departure_time = self.t
        self.departures.append(self.t)
        if len(veh.path) > 0:
            self.segments[veh.path[0]].add_vehicle(veh)

    def add_segment(self, seg) -> None:
        self.segments.append(seg)
    
    def create_vehicle(self, **kwargs) -> None:
        """Factory method to create a new vehicle and add it to the simulation."""
        veh = Vehicle(kwargs)
        self.add_vehicle(veh)

    def create_segment(self, *points, num_lanes=1) -> None:
        """Factory method to create a new segment given a set of points."""
        seg = Segment(points, num_lanes=num_lanes)
        self.add_segment(seg)

    def create_quadratic_bezier_curve(self, start, control, end) -> None:
        """Create and add a quadratic Bezier curve segment."""
        cur = QuadraticCurve(start, control, end)
        self.add_segment(cur)

    def create_cubic_bezier_curve(self, start, control_1, control_2, end) -> None:
        """Create and add a cubic Bezier curve segment."""
        cur = CubicCurve(start, control_1, control_2, end)
        self.add_segment(cur)

    def create_vehicle_generator(self, **kwargs) -> None:
        """Create and add a new vehicle generator."""
        gen = VehicleGenerator(kwargs)
        self.vehicle_generator.append(gen)


    def run(self, steps: int) -> None:
        """Advance the simulation by a given number of steps."""
        for _ in range(steps):
            self.update()

    def update(self) -> None:
        """
        Update simulation state:
        - Advance vehicle dynamics.
        - Handle transitions between segments.
        - Process vehicle generators.
        """
        # Update vehicles for each segment
        for segment in self.segments:
            if segment.vehicles:
                self.vehicles[segment.vehicles[0]].update(None, self.dt)
                for idx in range(1, len(segment.vehicles)):
                    lead_vehicle = self.vehicles[segment.vehicles[idx - 1]]
                    self.vehicles[segment.vehicles[idx]].update(lead_vehicle, self.dt)

        # Handle vehicles that have exited their segment
        for segment in self.segments:
            if not segment.vehicles:
                continue
            vehicle_id = segment.vehicles[0]
            vehicle = self.vehicles[vehicle_id]
            if vehicle.x >= segment.get_length():
                # Record arrival time only once (if not already recorded)
                if vehicle.arrival_time is None:
                    vehicle.arrival_time = self.t
                    self.arrivals.append(self.t)
                if vehicle.current_road_index + 1 < len(vehicle.path):
                    vehicle.current_road_index += 1
                    next_segment = self.segments[vehicle.path[vehicle.current_road_index]]
                    next_segment.vehicles.append(vehicle_id)
                vehicle.x = 0
                segment.vehicles.popleft()

            # Perform a simple lane change where applicable
            for veh_id in segment.vehicles:
                veh = self.vehicles[veh_id]
                if segment.num_lanes > 1 and veh.v > 0 and hash(veh_id) % 20 == 0:
                    veh.change_lane()

        # Update all vehicle generators
        for gen in self.vehicle_generator:
            gen.update(self)

        self.t += self.dt
        self.frame_count += 1
