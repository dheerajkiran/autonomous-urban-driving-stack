# TrafficVehicleArray.msg
# All vehicles within detection radius of the ego vehicle in the current sim step.
# Published by sumo_bridge at each simulation tick.

std_msgs/Header header
TrafficVehicle[] vehicles       # Vehicles sorted by distance_to_ego (nearest first)
int32 total_simulated           # Total vehicles currently in the simulation
float64 detection_radius_m      # Radius within which vehicles are reported [m]
