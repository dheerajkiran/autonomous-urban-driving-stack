# Route.msg
# A planned route from start to destination on the Tempe road network.
# Published by route_planner on /navigation/route.
# Consumed by waypoint_publisher, behavior_planner, and semantic_reasoner.

std_msgs/Header header

string start_address              # Human-readable origin (as entered by user)
string end_address                # Human-readable destination (as entered by user)
float64 total_distance_m          # Total route length [m]
float64 estimated_duration_s      # Estimated travel time at speed limits [s]

Waypoint[] waypoints              # Ordered sequence of road-network waypoints
int32 current_waypoint_index      # Index of the next waypoint to reach (updated in flight)
