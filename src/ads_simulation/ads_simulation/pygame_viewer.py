"""
Pygame Viewer Node

Real-time map visualization of the ADS simulation using OpenStreetMap
tile imagery. Renders the full-color Tempe, AZ street map as background,
with the ego vehicle (cyan), planned route (yellow), and traffic overlaid.

Tiles are downloaded from OSM on first run and cached locally — after
that the viewer works fully offline.

Controls
--------
  Scroll wheel  — zoom in / out
  ESC           — quit

Run alongside the full stack:
  ros2 run ads_simulation pygame_viewer
"""

import io
import json
import math
import sys
import threading
import time
import urllib.request
from pathlib import Path
from typing import Optional

sys.path.append("/usr/share/sumo/tools")

import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ads_interfaces.msg import Route, TrafficVehicleArray, VehicleState

# ── OSM tile config ───────────────────────────────────────────────────────────
TILE_PX     = 256
TILE_URL    = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
USER_AGENT  = "ADS-Portfolio-Viewer/1.0 github.com/dheerajkiran/autonomous-urban-driving-stack"
EARTH_R     = 6_378_137.0   # metres
OSM_ZOOM_MIN = 13
OSM_ZOOM_MAX = 18

# ── Overlay colors ────────────────────────────────────────────────────────────
EGO_OUTER    = (0,   140, 200)
EGO_INNER    = (0,   210, 255)
TRAFFIC_CAR  = (220,  60,  60)
TRAFFIC_HEAVY= (220, 140,  40)
ROUTE_LINE   = (30,  120, 255)
WAYPOINT_DOT = (0,   240, 110)
HUD_FG       = (240, 240, 250)
HUD_BG       = (0,     0,   0, 160)
HUD_DIM      = (160, 160, 180)

# ── Window ────────────────────────────────────────────────────────────────────
W, H         = 1280, 720
FPS          = 30
ZOOM_DEFAULT = 0.25   # pixels per metre
ZOOM_MIN     = 0.05
ZOOM_MAX     = 4.0


# ── Tile math ─────────────────────────────────────────────────────────────────

def _deg2tile(lat: float, lon: float, zoom: int):
    n   = 2 ** zoom
    xt  = int((lon + 180.0) / 360.0 * n)
    lat_r = math.radians(lat)
    yt  = int((1.0 - math.log(math.tan(lat_r) + 1.0 / math.cos(lat_r)) / math.pi) / 2.0 * n)
    return xt, yt


def _tile_origin_latlon(tx: int, ty: int, zoom: int):
    """Return the (lat, lon) of the NW corner of tile (tx, ty)."""
    n   = 2 ** zoom
    lon = tx / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * ty / n))))
    return lat, lon


def _metres_per_tile(lat: float, zoom: int) -> float:
    n = 2 ** zoom
    return 2 * math.pi * EARTH_R * math.cos(math.radians(lat)) / n


def _best_osm_zoom(pygame_zoom: float, lat: float) -> int:
    """Pick the OSM tile zoom where tiles render closest to native resolution."""
    # native_ppm at OSM zoom Z: TILE_PX * 2^Z / (2π * R * cos(lat))
    # We want the Z where native_ppm ≈ pygame_zoom.
    # Solving: Z = log2(pygame_zoom * 2π * R * cos(lat) / TILE_PX)
    denom = TILE_PX / (2 * math.pi * EARTH_R * math.cos(math.radians(lat)))
    z = round(math.log2(pygame_zoom / denom))
    return max(OSM_ZOOM_MIN, min(OSM_ZOOM_MAX, z))


_PNG_MAGIC = b"\x89PNG\r\n\x1a\n"


def _is_valid_png(path: Path) -> bool:
    try:
        with open(path, "rb") as f:
            return f.read(8) == _PNG_MAGIC
    except Exception:
        return False


class TileCache:
    """Downloads and caches OSM PNG tiles.

    Background threads only write bytes to disk.
    pygame.image.load is always called on the main thread to avoid SDL issues.
    Corrupt or partial files are detected via PNG magic bytes and re-downloaded.
    """

    def __init__(self, cache_dir: Path) -> None:
        self._dir = cache_dir / "tiles"
        self._dir.mkdir(parents=True, exist_ok=True)
        self._mem: dict = {}
        self._pending: set = set()
        self._lock = threading.Lock()

    def get(self, tx: int, ty: int, zoom: int) -> Optional[pygame.Surface]:
        """Return cached surface, or None while downloading. Main thread only."""
        key = (zoom, tx, ty)
        with self._lock:
            if key in self._mem:
                return self._mem[key]

        path = self._dir / f"{zoom}_{tx}_{ty}.png"

        if path.exists():
            if not _is_valid_png(path):
                # Corrupted or partial file — delete and re-download.
                path.unlink(missing_ok=True)
            else:
                try:
                    surf = pygame.image.load(str(path))
                    with self._lock:
                        self._mem[key] = surf
                    return surf
                except Exception:
                    path.unlink(missing_ok=True)

        # Queue a background download if not already in flight.
        with self._lock:
            if key not in self._pending:
                self._pending.add(key)
                threading.Thread(
                    target=self._download, args=(tx, ty, zoom, path, key), daemon=True
                ).start()
        return None

    def _download(self, tx: int, ty: int, zoom: int, path: Path, key: tuple) -> None:
        """Fetch tile from OSM and write to disk. Background thread — no pygame calls."""
        url = TILE_URL.format(z=zoom, x=tx, y=ty)
        try:
            req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
            with urllib.request.urlopen(req, timeout=10) as resp:
                data = resp.read()
            if data[:8] == _PNG_MAGIC:
                path.write_bytes(data)
            time.sleep(0.1)   # be polite to OSM tile servers
        except Exception:
            pass
        finally:
            with self._lock:
                self._pending.discard(key)


class PygameViewer(Node):

    def __init__(self) -> None:
        super().__init__("pygame_viewer")

        self.declare_parameter("cache_dir", str(Path.home() / "ads_map_cache"))
        cache_dir = Path(self.get_parameter("cache_dir").value).expanduser()
        self._net_path  = cache_dir / "tempe.net.xml"
        self._tile_cache = TileCache(cache_dir)

        self._lock        = threading.Lock()
        self._ego: Optional[VehicleState] = None
        self._last_state_time: float = 0.0   # wall-clock time of last VehicleState
        self._traffic     = []
        self._route_latlng: list = []
        self._route_xy:     list = []
        self._start_addr  = ""
        self._end_addr    = ""
        self._distance_m  = 0.0
        self._prev_xy     = None
        self._net         = None

        # Click-to-drive pins (lat, lon) or None
        self._pin_start: Optional[tuple] = None
        self._pin_end:   Optional[tuple] = None

        # Tempe, AZ centre as fallback before ego spawns
        self._default_lat = 33.4255
        self._default_lon = -111.9400

        self.create_subscription(VehicleState,       "/vehicle/state",               self._on_state,   1)
        self.create_subscription(TrafficVehicleArray,"/perception/traffic_vehicles",  self._on_traffic, 1)
        self.create_subscription(Route,              "/navigation/route",             self._on_route,   10)

        self._goal_pub    = self.create_publisher(String, "/navigation/latlon_goal",    10)
        self._confirm_pub = self.create_publisher(String, "/navigation/mission_confirm", 10)

        self.get_logger().info("PygameViewer ready — left-click=start, right-click=end, SPACE=go.")

    # ── ROS2 callbacks ────────────────────────────────────────────────────────

    def _on_state(self, msg: VehicleState) -> None:
        with self._lock:
            if self._prev_xy is not None:
                dx = msg.x - self._prev_xy[0]
                dy = msg.y - self._prev_xy[1]
                dist = math.sqrt(dx * dx + dy * dy)
                self._distance_m += dist
                if dist > 0.3:
                    msg.heading = math.atan2(dy, dx)
                elif self._ego is not None:
                    msg.heading = self._ego.heading
            self._prev_xy = (msg.x, msg.y)
            self._ego = msg
            self._last_state_time = time.time()

    def _on_traffic(self, msg: TrafficVehicleArray) -> None:
        with self._lock:
            self._traffic = list(msg.vehicles)

    def _on_route(self, msg: Route) -> None:
        with self._lock:
            self._start_addr  = msg.start_address
            self._end_addr    = msg.end_address
            self._distance_m  = 0.0
            self._prev_xy     = None
            self._route_latlng = [(wp.latitude, wp.longitude) for wp in msg.waypoints]
            if self._net is not None:
                self._route_xy = [
                    self._net.convertLonLat2XY(lon, lat)
                    for lat, lon in self._route_latlng
                ]

    # ── Coordinate helpers ────────────────────────────────────────────────────

    def _sumo_to_latlon(self, x: float, y: float):
        if self._net is None:
            return self._default_lat, self._default_lon
        lon, lat = self._net.convertXY2LonLat(x, y)
        return lat, lon

    def _latlon_to_screen(self, lat: float, lon: float,
                          cam_lat: float, cam_lon: float,
                          mpt: float, zoom: float):
        """Convert (lat, lon) to screen pixel (sx, sy) given camera centre."""
        dlat = lat - cam_lat
        dlon = lon - cam_lon
        dy_m =  dlat * (math.pi * EARTH_R / 180.0)
        dx_m =  dlon * (math.pi * EARTH_R / 180.0) * math.cos(math.radians(cam_lat))
        sx = int(W / 2 + dx_m * zoom)
        sy = int(H / 2 - dy_m * zoom)
        return sx, sy

    def _screen_to_latlon(self, sx: int, sy: int,
                          cam_lat: float, cam_lon: float, zoom: float):
        """Inverse of _latlon_to_screen — convert screen pixel to (lat, lon)."""
        dx_m = (sx - W / 2) / zoom
        dy_m = (H / 2 - sy) / zoom
        R_deg = math.pi * EARTH_R / 180.0
        dlat = dy_m / R_deg
        dlon = dx_m / (R_deg * math.cos(math.radians(cam_lat)))
        return cam_lat + dlat, cam_lon + dlon

    def _publish_latlon_goal(self) -> None:
        if self._pin_start is None or self._pin_end is None:
            return
        payload = json.dumps({
            "start_lat": self._pin_start[0],
            "start_lon": self._pin_start[1],
            "end_lat":   self._pin_end[0],
            "end_lon":   self._pin_end[1],
        })
        msg = String()
        msg.data = payload
        self._goal_pub.publish(msg)
        self.get_logger().info(
            f"Goal published: ({self._pin_start[0]:.5f},{self._pin_start[1]:.5f})"
            f" → ({self._pin_end[0]:.5f},{self._pin_end[1]:.5f})"
        )

    # ── Main Pygame loop ──────────────────────────────────────────────────────

    def run(self) -> None:
        pygame.init()
        screen = pygame.display.set_mode((W, H))
        pygame.display.set_caption("ADS — Tempe City Viewer")
        clock   = pygame.time.Clock()
        font_lg = pygame.font.SysFont("monospace", 20, bold=True)
        font_sm = pygame.font.SysFont("monospace", 14)

        net = self._load_sumo_net(screen, font_lg)
        with self._lock:
            self._net = net
            if self._route_latlng and net is not None:
                self._route_xy = [
                    net.convertLonLat2XY(lon, lat)
                    for lat, lon in self._route_latlng
                ]

        zoom = ZOOM_DEFAULT

        # Pan / drag state
        cam_free       = False   # True = user panned away from ego
        cam_lat_free   = self._default_lat
        cam_lon_free   = self._default_lon
        dragging       = False
        drag_mouse0    = (0, 0)
        drag_cam0      = (self._default_lat, self._default_lon)
        drag_moved     = False   # distinguish click from drag

        # Cache camera for click-to-latlon (updated each frame)
        cam_lat = self._default_lat
        cam_lon = self._default_lon

        R_deg = math.pi * EARTH_R / 180.0   # metres per degree latitude
        next_net_retry = time.time() + 1.0

        while True:
            if self._net is None and time.time() >= next_net_retry:
                next_net_retry = time.time() + 1.0
                retried_net = self._try_load_sumo_net()
                if retried_net is not None:
                    with self._lock:
                        self._net = retried_net
                        if self._route_latlng:
                            self._route_xy = [
                                retried_net.convertLonLat2XY(lon, lat)
                                for lat, lon in self._route_latlng
                            ]

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        return
                    if event.key == pygame.K_c:        # C = snap back to ego
                        cam_free = False
                    if event.key == pygame.K_r:        # R = reset pins
                        self._pin_start = None
                        self._pin_end   = None
                    if event.key == pygame.K_SPACE:    # SPACE = spawn / restart ego
                        msg = String()
                        msg.data = "GO"
                        self._confirm_pub.publish(msg)
                        cam_free = False   # snap camera back to follow ego
                        self.get_logger().info("GO sent — ego spawning.")
                if event.type == pygame.MOUSEWHEEL:
                    zoom = max(ZOOM_MIN, min(ZOOM_MAX, zoom * (1.12 ** event.y)))
                if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    dragging    = True
                    drag_moved  = False
                    drag_mouse0 = event.pos
                    drag_cam0   = (cam_lat_free, cam_lon_free)
                if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                    dragging = False
                    if not drag_moved:   # click (no significant movement) → set start
                        self._pin_start = self._screen_to_latlon(
                            event.pos[0], event.pos[1], cam_lat, cam_lon, zoom
                        )
                        self._pin_end = None   # reset end so user picks fresh end
                if event.type == pygame.MOUSEBUTTONDOWN and event.button == 3:
                    # Right click → set end point and publish goal
                    self._pin_end = self._screen_to_latlon(
                        event.pos[0], event.pos[1], cam_lat, cam_lon, zoom
                    )
                    self._publish_latlon_goal()
                if event.type == pygame.MOUSEMOTION and dragging:
                    dx_px = event.pos[0] - drag_mouse0[0]
                    dy_px = event.pos[1] - drag_mouse0[1]
                    if abs(dx_px) > 5 or abs(dy_px) > 5:
                        drag_moved = True
                    R_lon = R_deg * math.cos(math.radians(drag_cam0[0]))
                    cam_lat_free = drag_cam0[0] + dy_px / zoom / R_deg
                    cam_lon_free = drag_cam0[1] - dx_px / zoom / R_lon
                    cam_free = True

            wall_now = time.time()

            with self._lock:
                ego        = self._ego
                last_t     = self._last_state_time
                traffic    = list(self._traffic)
                route_ll   = list(self._route_latlng)
                start_a    = self._start_addr
                end_a      = self._end_addr
                dist_km    = self._distance_m / 1000.0

            # Dead reckoning — extrapolate ego position from last known
            # speed/heading so the dot moves smoothly at 30 FPS even when
            # SUMO updates arrive at only 5-10 Hz.
            if ego is not None:
                dt = min(wall_now - last_t, 1.5)   # cap at 1.5 s to avoid runaway
                if ego.speed > 0.2 and dt > 0:
                    dr_x = ego.x + math.cos(ego.heading) * ego.speed * dt
                    dr_y = ego.y + math.sin(ego.heading) * ego.speed * dt
                else:
                    dr_x, dr_y = ego.x, ego.y
                ego_lat, ego_lon = self._sumo_to_latlon(dr_x, dr_y)
            else:
                ego_lat, ego_lon = self._default_lat, self._default_lon

            # Camera: follow ego unless user panned
            if cam_free:
                cam_lat, cam_lon = cam_lat_free, cam_lon_free
            else:
                cam_lat, cam_lon = ego_lat, ego_lon
                cam_lat_free, cam_lon_free = cam_lat, cam_lon   # keep in sync
            # expose for click → latlon conversion (used in next frame's events)
            # variables already named cam_lat / cam_lon, no extra assignment needed

            osm_zoom = _best_osm_zoom(zoom, cam_lat)
            mpt = _metres_per_tile(cam_lat, osm_zoom)

            screen.fill((200, 200, 200))   # grey fallback while tiles load

            # ── OSM tile background ───────────────────────────────────────
            self._draw_tiles(screen, cam_lat, cam_lon, zoom, mpt, osm_zoom)

            # ── Planned route ─────────────────────────────────────────────
            if len(route_ll) >= 2:
                pts = [
                    self._latlon_to_screen(lat, lon, cam_lat, cam_lon, mpt, zoom)
                    for lat, lon in route_ll
                ]
                pygame.draw.lines(screen, ROUTE_LINE, False, pts, max(3, int(zoom * 5)))
                for pt in pts:
                    pygame.draw.circle(screen, WAYPOINT_DOT, pt, max(3, int(zoom * 5)))

            # ── Click pins ───────────────────────────────────────────────
            for pin, color, label in (
                (self._pin_start, (0, 220, 80),  "S"),
                (self._pin_end,   (220, 50, 50), "E"),
            ):
                if pin is not None:
                    px, py = self._latlon_to_screen(pin[0], pin[1], cam_lat, cam_lon, mpt, zoom)
                    pygame.draw.circle(screen, (255, 255, 255), (px, py), 14)
                    pygame.draw.circle(screen, color,           (px, py), 11)
                    lbl = font_sm.render(label, True, (255, 255, 255))
                    screen.blit(lbl, (px - lbl.get_width() // 2, py - lbl.get_height() // 2))

            # ── Ego vehicle ───────────────────────────────────────────────
            if ego is not None:
                if cam_free:
                    ex, ey = self._latlon_to_screen(
                        ego_lat, ego_lon, cam_lat, cam_lon, mpt, zoom
                    )
                else:
                    ex, ey = W // 2, H // 2
                r = max(8, int(zoom * 9))
                pygame.draw.circle(screen, EGO_OUTER, (ex, ey), r + 3)
                pygame.draw.circle(screen, EGO_INNER, (ex, ey), r)
                hlen = r * 2.8
                hx = ex + int(math.cos(ego.heading) * hlen)
                hy = ey - int(math.sin(ego.heading) * hlen)
                pygame.draw.line(screen, EGO_INNER, (ex, ey), (hx, hy), 3)

            # ── HUD ───────────────────────────────────────────────────────
            self._draw_hud(screen, font_lg, font_sm, ego, dist_km, start_a, end_a,
                           zoom, len(traffic), cam_free)

            pygame.display.flip()
            clock.tick(FPS)

    # ── Tile rendering ────────────────────────────────────────────────────────

    def _draw_tiles(self, screen, cam_lat, cam_lon, zoom, mpt, osm_zoom: int) -> None:
        tile_px_size = mpt * zoom            # rendered size of one tile in pixels
        if tile_px_size < 1:
            return

        # Centre tile (fractional) in OSM tile coordinates.
        n       = 2 ** osm_zoom
        cx_tile = (cam_lon + 180.0) / 360.0 * n
        lat_r   = math.radians(cam_lat)
        cy_tile = (1.0 - math.log(math.tan(lat_r) + 1.0 / math.cos(lat_r)) / math.pi) / 2.0 * n

        # How many tiles fit on screen in each direction.
        tx_range = int(math.ceil(W / tile_px_size / 2)) + 1
        ty_range = int(math.ceil(H / tile_px_size / 2)) + 1

        ctx = int(cx_tile)
        cty = int(cy_tile)

        scaled_size = max(1, int(tile_px_size))

        for dx in range(-tx_range, tx_range + 1):
            for dy in range(-ty_range, ty_range + 1):
                tx = ctx + dx
                ty = cty + dy
                if tx < 0 or ty < 0 or tx >= n or ty >= n:
                    continue

                surf = self._tile_cache.get(tx, ty, osm_zoom)
                if surf is None:
                    continue

                # Pixel position of tile NW corner on screen.
                px = int((tx - cx_tile) * tile_px_size + W / 2)
                py = int((ty - cy_tile) * tile_px_size + H / 2)

                if scaled_size != TILE_PX:
                    surf = pygame.transform.scale(surf, (scaled_size, scaled_size))

                screen.blit(surf, (px, py))

    # ── HUD ───────────────────────────────────────────────────────────────────

    def _draw_hud(self, screen, font_lg, font_sm, ego, dist_km,
                  start_a, end_a, zoom, n_traffic, cam_free: bool = False) -> None:
        lines = []
        if ego is not None:
            lines.append((font_lg, f"{ego.speed * 3.6:5.1f} km/h", HUD_FG))
            lines.append((font_sm, f"Gear: {ego.gear}   Dist: {dist_km:.3f} km", HUD_DIM))
        if start_a:
            s = start_a.split(",")[0]
            e = end_a.split(",")[0]
            lines.append((font_sm, f"{s}  →  {e}", HUD_DIM))
        lines.append((font_sm, f"Zoom: {zoom:.2f}x", HUD_DIM))
        if cam_free:
            lines.append((font_sm, "FREE VIEW — press C to re-center on ego", (255, 200, 60)))
        if self._pin_start is None:
            pin_hint = "Left-click = set START"
            hint_col = (180, 230, 180)
        elif self._pin_end is None:
            pin_hint = "Right-click = set END  |  R = reset"
            hint_col = (255, 220, 120)
        else:
            pin_hint = "SPACE = start ego  |  R = reset & repick"
            hint_col = (100, 200, 255)
        lines.append((font_sm, pin_hint, hint_col))
        lines.append((font_sm, "Drag=pan   Scroll=zoom   C=re-center   ESC=quit", HUD_DIM))

        y = 10
        for font, text, color in lines:
            surf = font.render(text, True, color)
            # Semi-transparent background bar.
            bg = pygame.Surface((surf.get_width() + 10, surf.get_height() + 4), pygame.SRCALPHA)
            bg.fill((0, 0, 0, 140))
            screen.blit(bg, (8, y - 2))
            screen.blit(surf, (13, y))
            y += surf.get_height() + 6

        # Legend bottom-left
        legend = [
            (EGO_INNER,  "Ego vehicle"),
            (ROUTE_LINE, "Planned route"),
        ]
        ly = H - 14 - len(legend) * 22
        for color, label in legend:
            pygame.draw.circle(screen, color, (20, ly + 7), 7)
            surf = font_sm.render(label, True, HUD_DIM)
            bg = pygame.Surface((surf.get_width() + 10, surf.get_height() + 4), pygame.SRCALPHA)
            bg.fill((0, 0, 0, 140))
            screen.blit(bg, (32, ly))
            screen.blit(surf, (35, ly + 2))
            ly += 22

    # ── SUMO network (for coordinate conversion only) ─────────────────────────

    def _load_sumo_net(self, screen, font):
        screen.fill((30, 30, 40))
        msg = font.render("Loading coordinate system...", True, HUD_FG)
        screen.blit(msg, (W // 2 - msg.get_width() // 2, H // 2))
        pygame.display.flip()
        return self._try_load_sumo_net()

    def _try_load_sumo_net(self):
        """Attempt to load the SUMO net; returns None on failure without logging
        at error level, since the net file may simply not exist yet if the
        viewer started before map_loader finished. Called once at startup and
        retried from the main loop until it succeeds."""
        try:
            import sumolib
            net = sumolib.net.readNet(str(self._net_path), withInternal=False)
            self.get_logger().info("SUMO network loaded for coordinate conversion.")
            return net
        except Exception as exc:
            self.get_logger().warn(f"SUMO net not ready yet ({exc}); will retry.")
            return None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PygameViewer()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    # Run ROS2 spin on a background thread so it catches messages the moment
    # DDS delivers them, independent of the Pygame frame rate.
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
