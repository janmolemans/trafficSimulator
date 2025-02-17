import uuid
import numpy as np
import random  # moved to module level

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
        # Optionally, randomize the initial velocity to be higher for busy traffic.
        self.v = np.random.uniform(12, 18)
        self.a = 0
        self.stopped = False
        # New attributes for tracking times:
        self.departure_time = None  
        self.arrival_time = None  
        # Add lane change timer to prevent rapid oscillations.
        self.lane_change_timer = 0
        self.lane_change_cooldown = 1.0  # seconds

    def update(self, lead, dt: float):
        """
        Update vehicle dynamics.
        :param lead: leading vehicle (or None)
        :param dt: time step
        """
        # Decrease lane change timer
        self.lane_change_timer = max(0, self.lane_change_timer - dt)
        
        new_v = self.v + self.a * dt
        if new_v < 0:
            new_x = self.x - 0.5 * self.v**2 / self.a
            new_v = 0
        else:
            new_x = self.x + self.v * dt + 0.5 * self.a * dt * dt

        # Collision avoidance: if there is a lead vehicle and the new position
        # would intrude its safe gap, clip the position and velocity.
        if lead:
            safe_x = lead.x - lead.l - self.s0
            if new_x > safe_x:
                new_x = safe_x
                new_v = lead.v
        
        self.x = new_x
        self.v = new_v

        # Use IDM for acceleration
        s_star = self.s0 + max(
            0,
            self.T * self.v
            + (self.v - (lead.v if lead else 0)) * self.v / (2.0 * np.sqrt(self.a_max * self.b_max))
        )
        gap = (lead.x - self.x - lead.l) if lead else 9999
        self.a = self.a_max * (
            1 - (self.v / self.v_max)**4 - (s_star / max(gap, 0.1))**2
        )

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

    def update_lane_decision(self, max_lanes: int, lane_occupancy: dict) -> None:
        """
        Revised lane decision logic with cooldown to prevent rapid oscillations.
        """
        # Use a simplified MOBIL approach
        if self.lane_change_timer > 0:
            return

        safety_gap = self.s0

        def lane_free(target_lane):
            for other in lane_occupancy.get(target_lane, []):
                if other.id == self.id:
                    continue
                if abs(other.x - self.x) < safety_gap:
                    return False
            return True

        def accel_if_lane(lane):
            others = lane_occupancy.get(lane, [])
            lead = min([o for o in others if o.x > self.x], key=lambda v: v.x, default=None)
            gap = (lead.x - self.x - lead.l) if lead else 9999
            s_star = self.s0 + max(
                0,
                self.T * self.v + (self.v - (lead.v if lead else 0)) * self.v / (2 * np.sqrt(self.a_max*self.b_max))
            )
            return self.a_max * (1 - (self.v/self.v_max)**4 - (s_star / max(gap, 0.1))**2)

        accel_current = accel_if_lane(self.lane)
        candidate_lanes = []
        if self.lane > 0: candidate_lanes.append(self.lane - 1)
        if self.lane < max_lanes - 1: candidate_lanes.append(self.lane + 1)

        # Additional check: if there's a free right lane, prefer to move there.
        # We'll do this check first, and if it's beneficial, move right.
        if self.lane > 0:
            if lane_free(self.lane - 1):
                # Move right more aggressively
                # e.g., if gain is positive or random chance is high
                gain_right = accel_if_lane(self.lane - 1) - accel_if_lane(self.lane)
                if gain_right > -0.5:  # Accept a slight decrease to ensure right-lane preference
                    self.lane = self.lane - 1
                    self.lane_change_timer = self.lane_change_cooldown
                    return

        best_lane = self.lane
        best_gain = 0
        for ln in candidate_lanes:
            if lane_free(ln):
                a_new = accel_if_lane(ln)
                gain = a_new - accel_current
                if gain > best_gain:
                    best_gain = gain
                    best_lane = ln

        if best_lane != self.lane:
            self.lane = best_lane
            self.lane_change_timer = self.lane_change_cooldown
