import uuid
import numpy as np

class Vehicle:
    """
    Represents a vehicle with properties and dynamics.
    Configuration keys may include: lane, v, etc.
    """
    def __init__(self, config: dict = {}):
        """
        Initialize vehicle with default configuration and update with provided settings.
        """
        # Initialize default values
        self._initialize_defaults()
        
        # Overwrite defaults with custom configuration
        for attr, val in config.items():
            setattr(self, attr, val)
            
        self.lane = config.get('lane', 0)
        self._initialize_properties()
        
    def _initialize_defaults(self) -> None:
        """Establish default vehicle parameters with randomized behavior."""
        self.id = uuid.uuid4()
        # Vehicle length: mean 4 meters, std 0.2, lower bound 3.5
        self.l = max(3.5, np.random.normal(4, 0.2))
        # Minimum gap: mean 4 m, std 0.5, lower bound 2 m
        self.s0 = max(2, np.random.normal(4, 0.5))
        # Safe time headway between 0.8 and 1.2 seconds
        self.T = np.random.uniform(0.8, 1.2)
        # Maximum velocity: mean 16.6 m/s, std 1.0, minimum 10 m/s
        self.v_max = max(10, np.random.normal(16.6, 1.0))
        # Maximum acceleration: mean 1.44 m/s², std 0.2, minimum 0.5 m/s²
        self.a_max = max(0.5, np.random.normal(1.44, 0.2))
        # Maximum deceleration: mean 4.61 m/s², std 0.5, minimum 3.5 m/s²
        self.b_max = max(3.5, np.random.normal(4.61, 0.5))
        
        self.path = []
        self.current_road_index = 0
        self.x = 0
        # Optionally, randomize the initial velocity within a reasonable range
        self.v = np.random.uniform(8, 12)
        self.a = 0
        self.stopped = False
        # New attributes for tracking times:
        self.departure_time = None  
        self.arrival_time = None  

    def _initialize_properties(self):
        """Pre-compute useful properties."""
        self.sqrt_ab = 2 * np.sqrt(self.a_max * self.b_max)
        self._v_max = self.v_max

    def update(self, lead, dt: float):
        """
        Update vehicle dynamics.
        :param lead: leading vehicle (or None)
        :param dt: time step
        """
        # Position and velocity update 
        if self.v + self.a * dt < 0:
            self.x -= 0.5 * self.v**2 / self.a
            self.v = 0
        else:
            self.v += self.a * dt
            self.x += self.v * dt + 0.5 * self.a * dt * dt
        
        # Compute acceleration adjustment
        alpha = 0
        if lead:
            delta_x = lead.x - self.x - lead.l
            delta_v = self.v - lead.v
            alpha = (self.s0 + max(0, self.T * self.v + (delta_v * self.v) / self.sqrt_ab)) / delta_x

        self.a = self.a_max * (1 - (self.v / self.v_max)**4 - alpha**2)

        if self.stopped: 
            self.a = -self.b_max * self.v / self.v_max

    def change_lane(self, direction=1):
        """
        Change lane of the vehicle.
        :param direction: +1 for right, -1 for left
        """
        self.lane += direction
        if self.lane < 0: 
            self.lane = 0

    def update_lane_decision(self, max_lanes: int) -> None:
        """
        Determine lane change for overtaking or returning to the right.
        Assumes lane 0 is the right-most (preferred) lane.
        """
        import random
        # If on the right lane and velocity is below 90% of maximum, attempt to overtake to the left.
        if self.lane == 0:
            if self.v < self.v_max * 0.9 and max_lanes > 1:
                if random.random() < 0.5:
                    self.lane = 1  # move left
        else:
            # If already left and vehicle is performing well, try to return right gradually.
            if self.v > self.v_max * 0.95:
                if random.random() < 0.3:
                    self.lane = max(self.lane - 1, 0)
            # Optionally, if still slow (e.g., heavily impeded), try going further left if possible.
            if self.v < self.v_max * 0.85 and self.lane < max_lanes - 1:
                if random.random() < 0.3:
                    self.lane += 1
