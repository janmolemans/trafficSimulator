"""
Benchmark test for TrafficSimulator.
Runs the simulation for a fixed number of steps and prints the elapsed time,
frame count, and simulation time.
"""
import time
import trafficSimulator as ts

# Set up simulation
sim = ts.Simulation()
sim.create_segment(
    (-200, 0),
    (200, 0),
    num_lanes=3
)
sim.create_vehicle_generator(
    vehicle_rate=100,
    vehicles=[
        (1, {'path': [0], 'v': 10, 'lane': 0}),
        (1, {'path': [0], 'v': 12, 'lane': 1}),
        (1, {'path': [0], 'v': 14, 'lane': 2}),
    ]
)

# Benchmarking
steps = 60000
start_time = time.time()
sim.run(steps)
elapsed = time.time() - start_time

print(f"Benchmark completed in {elapsed:.4f} seconds.")
print(f"Total frames: {sim.frame_count}")
print(f"Simulation time: {sim.t:.2f} s")
