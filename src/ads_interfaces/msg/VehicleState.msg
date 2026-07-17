# VehicleState.msg
# Kinematic and actuator state of the ego vehicle.
# Published by ads_simulation/sumo_bridge from live TraCI data.
# Consumed by: pygame_viewer, car3d_bridge.

std_msgs/Header header

# --- Kinematic State ---
float64 speed           # Longitudinal speed [m/s]
float64 acceleration    # Longitudinal acceleration [m/s^2]
float64 heading         # Yaw angle [rad] (0 = east, CCW positive, range [-π, π])
float64 yaw_rate        # Yaw rate [rad/s] (CCW positive)

# --- 2D Position (world frame, origin = start pose) ---
float64 x               # Position x [m]
float64 y               # Position y [m]

# --- Actuator State ---
float64 throttle        # Normalized throttle command [0.0, 1.0]
float64 brake           # Normalized brake pressure  [0.0, 1.0]
float64 steering_angle  # Front wheel steering angle [rad] (positive = left turn)

# --- Operational Mode ---
string gear             # PARK | REVERSE | NEUTRAL | DRIVE
bool is_autonomous      # True if vehicle is under autonomous control
