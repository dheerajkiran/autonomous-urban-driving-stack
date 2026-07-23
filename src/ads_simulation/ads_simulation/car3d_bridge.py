"""
Car3D Bridge Node

Streams the ego vehicle's live state and its current route geometry over
a plain WebSocket, for a lightweight browser-based 3D viewer
(car3d_viewer.html). This is a standalone visualization aid — no other
node depends on it.

Deliberately scoped to the ego's current route only, not the whole road
network — Tempe's network is ~18,000 drivable edges across ~9km x ~15km,
which is far too much thin-line geometry for a browser to render cleanly
and makes camera framing/depth-precision needlessly hard. A route is
usually a few dozen edges across at most a couple of km, which sidesteps
both problems entirely.

Subscribes
----------
/navigation/route            (ads_interfaces/msg/Route)  — published by sumo_bridge on ego spawn
/navigation/route_lanes      (std_msgs/String)  — JSON per-lane geometry for the same route
/navigation/route_buildings  (std_msgs/String)  — JSON building footprints near the same route
/navigation/traffic_vehicles (std_msgs/String)  — JSON background traffic state, every sim tick
/vehicle/state                (ads_interfaces/msg/VehicleState)

Serves
------
ws://0.0.0.0:<ws_port>  — JSON messages, each also sent to a client on connect if available:
  {"type": "route", "points": [[x,y], ...]}
  {"type": "lanes", "edges": [{"direction": "forward"|"reverse",
                                "lanes": [{"width":, "shape": [[x,y], ...]}, ...]}],
                     "junctions": [{"shape": [[x,y], ...]}, ...]}
  {"type": "buildings", "buildings": [[[x,y], ...], ...]}
  {"type": "ego", "x":, "y":, "heading":, "speed":}   — sent on every /vehicle/state update
  {"type": "traffic", "vehicles": [{"id":, "x":, "y":, "heading":, "speed":}, ...]}
"""

import asyncio
import json
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ads_interfaces.msg import Route, VehicleState


class Car3DBridge(Node):

    def __init__(self) -> None:
        super().__init__("car3d_bridge")

        self.declare_parameter("ws_port", 8765)
        self._ws_port = self.get_parameter("ws_port").value

        self._ws_clients: set = set()
        self._route_json: Optional[str] = None
        self._lanes_json: Optional[str] = None
        self._buildings_json: Optional[str] = None
        self._traffic_json: Optional[str] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        # Recenter on the route's own start point rather than any city-wide
        # reference — keeps scene coordinates small regardless of where in
        # Tempe the route happens to be.
        self._origin = (0.0, 0.0)

        self.create_subscription(Route, "/navigation/route", self._on_route, 10)
        self.create_subscription(VehicleState, "/vehicle/state", self._on_state, 10)
        self.create_subscription(String, "/navigation/route_lanes", self._on_route_lanes, 10)
        self.create_subscription(String, "/navigation/route_buildings", self._on_route_buildings, 10)
        self.create_subscription(String, "/navigation/traffic_vehicles", self._on_traffic_vehicles, 10)

        threading.Thread(target=self._run_ws_server, daemon=True).start()

        self.get_logger().info(f"Car3DBridge initialized — ws://0.0.0.0:{self._ws_port}")

    # ------------------------------------------------------------------
    # ROS2 callbacks -> broadcast to connected WebSocket clients
    # ------------------------------------------------------------------

    def _on_route(self, msg: Route) -> None:
        if not msg.waypoints:
            return
        ox, oy = msg.waypoints[0].x, msg.waypoints[0].y
        self._origin = (ox, oy)
        points = [[wp.x - ox, wp.y - oy] for wp in msg.waypoints]
        self._route_json = json.dumps({"type": "route", "points": points})

        if self._loop is not None:
            asyncio.run_coroutine_threadsafe(self._broadcast(self._route_json), self._loop)

    def _on_route_lanes(self, msg: String) -> None:
        # sumo_bridge always publishes /navigation/route right before
        # /navigation/route_lanes for the same spawn, so self._origin is
        # already current by the time this arrives.
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        ox, oy = self._origin
        edges = [
            {
                "direction": edge["direction"],
                "lanes": [
                    {"width": lane["width"], "shape": [[x - ox, y - oy] for x, y in lane["shape"]]}
                    for lane in edge["lanes"]
                ],
            }
            for edge in data.get("edges", [])
        ]
        junctions = [
            {"shape": [[x - ox, y - oy] for x, y in j["shape"]]}
            for j in data.get("junctions", [])
        ]
        self._lanes_json = json.dumps({"type": "lanes", "edges": edges, "junctions": junctions})

        if self._loop is not None:
            asyncio.run_coroutine_threadsafe(self._broadcast(self._lanes_json), self._loop)

    def _on_route_buildings(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        ox, oy = self._origin
        buildings = [
            [[x - ox, y - oy] for x, y in shape]
            for shape in data.get("buildings", [])
        ]
        self._buildings_json = json.dumps({"type": "buildings", "buildings": buildings})

        if self._loop is not None:
            asyncio.run_coroutine_threadsafe(self._broadcast(self._buildings_json), self._loop)

    def _on_traffic_vehicles(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        ox, oy = self._origin
        vehicles = [
            {
                "id": v["id"],
                "x": v["x"] - ox,
                "y": v["y"] - oy,
                "heading": v["heading"],
                "speed": v["speed"],
            }
            for v in data.get("vehicles", [])
        ]
        self._traffic_json = json.dumps({"type": "traffic", "vehicles": vehicles})

        if self._loop is not None:
            asyncio.run_coroutine_threadsafe(self._broadcast(self._traffic_json), self._loop)

    def _on_state(self, msg: VehicleState) -> None:
        if self._loop is None:
            return
        ox, oy = self._origin
        payload = json.dumps({
            "type": "ego",
            "x": msg.x - ox,
            "y": msg.y - oy,
            "heading": msg.heading,
            "speed": msg.speed,
        })
        asyncio.run_coroutine_threadsafe(self._broadcast(payload), self._loop)

    async def _broadcast(self, payload: str) -> None:
        if not self._ws_clients:
            return
        await asyncio.gather(
            *(client.send(payload) for client in list(self._ws_clients)),
            return_exceptions=True,
        )

    # ------------------------------------------------------------------
    # WebSocket server (own thread, own asyncio event loop)
    # ------------------------------------------------------------------

    def _run_ws_server(self) -> None:
        asyncio.run(self._serve())

    async def _serve(self) -> None:
        import websockets

        self._loop = asyncio.get_running_loop()

        async def handler(websocket):
            self._ws_clients.add(websocket)
            self.get_logger().info(f"3D viewer connected ({len(self._ws_clients)} total).")
            try:
                if self._route_json:
                    await websocket.send(self._route_json)
                if self._lanes_json:
                    await websocket.send(self._lanes_json)
                if self._buildings_json:
                    await websocket.send(self._buildings_json)
                if self._traffic_json:
                    await websocket.send(self._traffic_json)
                async for _ in websocket:
                    pass   # no client -> server messages expected
            finally:
                self._ws_clients.discard(websocket)
                self.get_logger().info(f"3D viewer disconnected ({len(self._ws_clients)} total).")

        async with websockets.serve(handler, "0.0.0.0", self._ws_port):
            await asyncio.Future()   # run forever


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Car3DBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Car3DBridge.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
