import uuid
import numpy as np

class Vehicle:
    """
    Represents a vehicle with dynamic properties and behavior.
    """
    def __init__(self, config: dict = {}) -> None:
        """
        Initialize a new vehicle instance.
        
        :param config: Custom configuration dictionary for the vehicle.
        """
        # Initialize default values
        self._initialize_defaults()
        
        # Overwrite defaults with custom configuration
        for attr, val in config.items():
            setattr(self, attr, val)
            
        self.lane = config.get('lane', 0)
        self._initialize_properties()
        
    def _initialize_defaults(self) -> None:
        """Establish default vehicle parameters."""
        self.id = uuid.uuid4()
        self.l = 4           # vehicle length
        self.s0 = 4          # minimum gap
        self.T = 1           # safe time headway
        self.v_max = 16.6    # max velocity (m/s)
        self.a_max = 1.44    # max acceleration (m/s²)
        self.b_max = 4.61    # max deceleration (m/s²)
        self.path = []
        self.current_road_index = 0
        self.x = 0
        self.v = 0
        self.a = 0
        self.stopped = False
        # New attributes for tracking times:
        self.departure_time = None  
        self.arrival_time = None  

    def _initialize_properties(self) -> None:
        """
        Pre-compute properties such as the combined acceleration term.
        """
        self.sqrt_ab = 2 * np.sqrt(self.a_max * self.b_max)
        self._v_max = self.v_max

    def update(self, lead, dt: float) -> None:
        """
        Update the vehicle's position, velocity, and acceleration.
        
        :param lead: The leading vehicle instance (or None)
        :param dt: Time step for updating vehicle dynamics.
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

    def change_lane(self, direction: int = 1) -> None:
        """
        Change the vehicle's lane.
        
        :param direction: +1 for moving right, -1 for moving left.
        """
        self.lane += direction
        # Example clamp to avoid invalid lanes:
        if self.lane < 0: self.lane = 0
