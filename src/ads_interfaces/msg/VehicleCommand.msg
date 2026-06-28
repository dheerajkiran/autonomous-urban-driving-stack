# VehicleCommand.msg
# Motion command issued to the vehicle plant.
# In production, issued by the trajectory planner or controller node.
# For Phase 2, issued by vehicle_commander test node.
# Consumed by: vehicle_state_publisher (plant model).

std_msgs/Header header

float64 target_speed      # Desired longitudinal speed [m/s] (0.0 = stop in place)
float64 steering_angle    # Desired front-wheel steering angle [rad] (positive = left turn)
string gear               # PARK | REVERSE | NEUTRAL | DRIVE
bool emergency_stop       # If true, overrides all fields — maximum braking, zero throttle
