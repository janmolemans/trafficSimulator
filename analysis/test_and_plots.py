"""
Test and Plot script for TrafficSimulator.
Creates a single multi-lane segment and a vehicle generator,
runs the simulation for a fixed number of steps to ensure termination,
and then displays four plots in one figure using Plotly:
1. Histogram of Vehicle Departures
2. Histogram of Vehicle Arrivals
3. Histogram of Travel Times
4. Scatter Plot: Travel Time vs. Departure Time
"""
import trafficSimulator as ts
from plotly.subplots import make_subplots
import plotly.graph_objects as go

# Initialize simulation
sim = ts.Simulation()

# Create a single segment with 3 lanes
sim.create_segment(
    (-200, 0),
    (200, 0),
    num_lanes=3
)

# Update vehicle generator to create a busy scenario:
sim.create_vehicle_generator(
    vehicle_rate=100000,
)

# Increase simulation steps to load more vehicles on the road
print("Running simulation...")
sim.run(3000)
print("Simulation complete.")

# Create a window to visualize the rest of the simulation
win = ts.Window(sim)
win.run()
win.show()

# Prepare data for plots
departures = sim.departures
arrivals = sim.arrivals
travel_times = [veh.arrival_time - veh.departure_time for veh in sim.vehicles.values() 
                 if veh.departure_time is not None and veh.arrival_time is not None]
data_scatter = [(veh.departure_time, veh.arrival_time - veh.departure_time) 
                for veh in sim.vehicles.values() 
                if veh.departure_time is not None and veh.arrival_time is not None]
if data_scatter:
    dep_scatter, travel_scatter = zip(*sorted(data_scatter))
else:
    dep_scatter, travel_scatter = [], []

# Create a 2x2 subplot grid
fig = make_subplots(
    rows=2, cols=2,
    subplot_titles=(
        'Histogram of Vehicle Departures',
        'Histogram of Vehicle Arrivals',
        'Histogram of Travel Times',
        'Travel Time vs. Departure Time'
    )
)

fig.add_trace(
    go.Histogram(x=departures, nbinsx=20, marker_color='skyblue', name='Departures'),
    row=1, col=1
)
fig.add_trace(
    go.Histogram(x=arrivals, nbinsx=20, marker_color='lightgreen', name='Arrivals'),
    row=1, col=2
)
fig.add_trace(
    go.Histogram(x=travel_times, nbinsx=20, marker_color='salmon', name='Travel Times'),
    row=2, col=1
)
fig.add_trace(
    go.Scatter(x=dep_scatter, y=travel_scatter, mode='markers', marker=dict(color='purple'), name='Scatter'),
    row=2, col=2
)

fig.update_layout(title_text='Traffic Simulation Data', showlegend=False)
fig.show()
