# TrafficVehicle.msg
# A single background vehicle detected in the SUMO simulation.
# Published by sumo_bridge on /perception/traffic_vehicles.
# Consumed by behavior_planner for lane-change and speed decisions.

string vehicle_id           # SUMO internal vehicle identifier
float64 x                   # SUMO x coordinate [m]
float64 y                   # SUMO y coordinate [m]
float64 speed               # Current speed [m/s]
float64 heading             # Heading angle [rad] (0=east, CCW positive)
string lane_id              # SUMO lane identifier (e.g. "E12_0")
float64 distance_to_ego     # Euclidean distance from ego vehicle [m]
string vehicle_type         # car | truck | motorcycle | bus
