"""
Test script for TrafficSimulator.
Creates a single multi-lane segment and a vehicle generator with varied speeds.
"""
import trafficSimulator as ts

sim = ts.Simulation()

# Single segment with 3 lanes
sim.create_segment(
    (-200, 0),
    (200, 0),
    num_lanes=3
)

# Add a vehicle generator that randomly assigns lanes
sim.create_vehicle_generator(
    vehicle_rate=100,
    vehicles=[
        (1, {'path': [0], 'v': 10, 'lane': 0}),
        (1, {'path': [0], 'v': 12, 'lane': 1}),
        (1, {'path': [0], 'v': 14, 'lane': 2}),
    ]
)

win = ts.Window(sim)
win.run()
win.show()
