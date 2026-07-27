# Next Scope: Perception + Learned Control

Context: current stack (ROS2 + SUMO + A* routing + live 3D viewer) is solid systems
engineering, but doesn't demonstrate the AI/perception/RL specialization the MS
coursework covers. The ego vehicle's driving behavior is entirely SUMO's built-in
car-following model — nothing in the project currently makes a learned or perceptual
decision. Two extensions to close that gap, in priority order.

---

## 1. Perception module

Add a simulated sensor + detector in the loop, instead of trusting SUMO's
ground-truth vehicle/lane state directly.

- Render a simulated camera (and/or LIDAR-style point cloud) view from the ego's
  pose, using the geometry already available to `car3d_bridge`/`sumo_bridge`
  (lane polygons, building footprints, other vehicle poses).
- Run detection/tracking on that rendered view to localize nearby traffic and
  lane boundaries, rather than reading them straight from SUMO/TraCI.
- Publish detections on a new ROS2 topic (e.g. `/perception/detections`) so the
  rest of the stack can consume them the same way it consumes ground truth today
  — sets up a clean before/after comparison (ground truth vs. perceived).
- Open questions: render camera frames from the existing Three.js scene (headless,
  server-side) vs. a lighter synthetic renderer; which detector (YOLOv8, given it's
  already used in the assistive-vision project); how to score accuracy against the
  SUMO ground truth we already publish.

## 2. Learned control / RL for ego driving

Replace (or run alongside, for comparison) SUMO's stock car-following/lane-change
model for the ego with a trained policy.

- Keep SUMO's A*-resolved route as the plan; train a policy that controls
  speed/lane-keeping along that route, using `traci` to override the ego's
  low-level control instead of letting SUMO drive it.
- Reward shaping around what's already observable: progress along route,
  collision/near-miss avoidance vs. background traffic, smoothness.
- Benchmark trained policy against the current SUMO-default baseline on the same
  routes (already reproducible via the click-to-drive UI).
- Open questions: on-policy RL (e.g. PPO) vs. simpler imitation of SUMO's own
  behavior as a first baseline; how many training routes/episodes are feasible
  given SUMO isn't GPU-accelerated; whether this runs on the Mac dev machine or
  needs the Ubuntu target.

---

## Sequencing note

Perception is the more natural first step — it's additive (new topic, doesn't
change existing ego behavior) and directly matches the in-progress "Perception in
Robotics" coursework. RL control is a bigger lift (training loop, reward design,
evaluation harness) and changes core ego behavior, so it should come after
perception is in place and stable.
