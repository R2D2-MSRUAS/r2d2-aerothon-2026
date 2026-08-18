"""Occupancy mapping from OAK-D Lite stereo depth — R2D2 Mission 2.

Pipeline per frame (runs on Jetson, depth comes off the Myriad X):

  depth (mm, HxW) --back-project--> camera-frame points
                  --rotate/translate (45 deg tilt + drone pose)--> world ENU
                  --height filter--> obstacle cells
                  --log-odds update--> 2D occupancy grid (for A*)

Grid: fixed-size local world grid, log-odds per cell, origin set at first
update (home). ``inflate()`` grows obstacles by the drone radius so A* can
treat cells as free/blocked directly. Pure numpy, ~2-4 ms per 400x640 depth
frame at 4x decimation.

Usage:
    m = OccupancyMapper(MapConfig())
    m.update(depth_mm, pose)        # pose from MavlinkBridge.get_state()
    grid = m.planner_grid()         # 0 free / 1 occupied (inflated)
"""
import math
import numpy as np


class MapConfig:
    # OAK-D Lite mono/stereo intrinsics @ 640x400 (approx; calibrate!)
    fx = 452.9; fy = 452.9; cx = 320.0; cy = 200.0
    depth_min_m = 0.4          # stereo min range
    depth_max_m = 8.0          # discard noisy far returns
    decimate = 4               # subsample depth pixels

    cam_tilt_deg = 45.0        # OAK-D pitched down 45 deg from horizontal
    cam_offset_body = (0.10, 0.0, -0.03)  # camera pos in body frame (m, FRD)

    # Grid: local world, ENU, drone starts at centre
    size_m = 60.0
    res_m = 0.25
    obst_z_min_m = 0.3         # ignore ground returns below this AGL
    obst_z_max_m = 6.0         # ignore canopy above corridor height

    lo_hit = 0.85              # log-odds increments
    lo_miss = -0.4
    lo_clamp = 4.0
    lo_occ_thresh = 1.0
    inflate_m = 0.45           # drone radius + margin


class OccupancyMapper:
    def __init__(self, cfg: MapConfig = None):
        self.cfg = cfg or MapConfig()
        c = self.cfg
        self.n = int(c.size_m / c.res_m)
        self.logodds = np.zeros((self.n, self.n), np.float32)
        self.origin = None      # (lat, lon) anchoring grid centre
        # camera->body rotation: pitch down by tilt (FRD body frame)
        t = math.radians(c.cam_tilt_deg)
        self._R_cb = np.array([[math.cos(t), 0, math.sin(t)],
                               [0, 1, 0],
                               [-math.sin(t), 0, math.cos(t)]], np.float32)
        # precompute pixel rays for the decimated grid lazily
        self._rays = None
        self._ray_shape = None

    # ------------------------------------------------------------ geometry
    def _pixel_rays(self, h, w):
        if self._ray_shape == (h, w):
            return self._rays
        c, d = self.cfg, self.cfg.decimate
        vs, us = np.mgrid[0:h:d, 0:w:d]
        x = (us - c.cx) / c.fx           # camera: x right, y down, z forward
        y = (vs - c.cy) / c.fy
        self._rays = (x.ravel(), y.ravel(), us.shape)
        self._ray_shape = (h, w)
        return self._rays

    def _latlon_to_xy(self, lat, lon):
        """Local ENU metres from grid origin (small-area approximation)."""
        lat0, lon0 = self.origin
        east = (lon - lon0) * 111_320.0 * math.cos(math.radians(lat0))
        north = (lat - lat0) * 111_320.0
        return east, north

    def _world_to_cell(self, e, n):
        half = self.cfg.size_m / 2
        i = ((n + half) / self.cfg.res_m).astype(int)   # row = north
        j = ((e + half) / self.cfg.res_m).astype(int)   # col = east
        return i, j

    # -------------------------------------------------------------- update
    def update(self, depth_mm: np.ndarray, pose: dict):
        """depth_mm: HxW uint16 depth in mm (OAK-D `depth` output).
        pose: needs lat, lon, relative_alt_m, and yaw_deg (add ATTITUDE
        parsing in the bridge, or pass VFR_HUD heading)."""
        c = self.cfg
        if pose.get("lat") is None:
            return
        if self.origin is None:
            self.origin = (pose["lat"], pose["lon"])

        d = c.decimate
        z = depth_mm[::d, ::d].astype(np.float32) / 1000.0
        xr, yr, _ = self._pixel_rays(*depth_mm.shape)
        z = z.ravel()
        ok = (z > c.depth_min_m) & (z < c.depth_max_m)
        if not ok.any():
            return
        # camera frame -> body FRD: cam(x right,y down,z fwd) -> body(x fwd,
        # y right, z down), then tilt
        pc = np.stack([z[ok], xr[ok] * z[ok], yr[ok] * z[ok]])  # fwd,right,dn
        pb = self._R_cb @ pc + np.array(c.cam_offset_body,
                                        np.float32)[:, None]
        # body -> world ENU with yaw
        yaw = math.radians(pose.get("yaw_deg", 0.0))
        cy_, sy = math.cos(yaw), math.sin(yaw)
        e0, n0 = self._latlon_to_xy(pose["lat"], pose["lon"])
        alt = pose.get("relative_alt_m") or 0.0
        north = pb[0] * cy_ - pb[1] * sy + n0
        east = pb[0] * sy + pb[1] * cy_ + e0
        up = alt - pb[2]                       # FRD z-down -> ENU up

        hit = (up > c.obst_z_min_m) & (up < c.obst_z_max_m)
        i, j = self._world_to_cell(east, north)
        inb = (i >= 0) & (i < self.n) & (j >= 0) & (j < self.n)

        occ_i, occ_j = i[hit & inb], j[hit & inb]
        np.add.at(self.logodds, (occ_i, occ_j), c.lo_hit)
        # cheap free-space: ground-plane returns decay cells (full ray-trace
        # skipped for the 30 ms budget)
        free = (~hit) & inb & (up <= c.obst_z_min_m)
        np.add.at(self.logodds, (i[free], j[free]), c.lo_miss)
        np.clip(self.logodds, -c.lo_clamp, c.lo_clamp, out=self.logodds)

    # -------------------------------------------------------------- output
    def occupancy(self) -> np.ndarray:
        """Binary grid: 1 = occupied."""
        return (self.logodds > self.cfg.lo_occ_thresh).astype(np.uint8)

    def planner_grid(self) -> np.ndarray:
        """Occupancy inflated by drone radius — feed straight to A*."""
        occ = self.occupancy()
        r = max(1, int(round(self.cfg.inflate_m / self.cfg.res_m)))
        # box dilation via shifted ORs (no scipy dependency)
        out = occ.copy()
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                if di == 0 and dj == 0:
                    continue
                s = np.zeros_like(occ)
                si = slice(max(di, 0), self.n + min(di, 0))
                ti = slice(max(-di, 0), self.n + min(-di, 0))
                sj = slice(max(dj, 0), self.n + min(dj, 0))
                tj = slice(max(-dj, 0), self.n + min(-dj, 0))
                s[ti, tj] = occ[si, sj]
                out |= s
        return out

    def cell_of(self, lat, lon):
        """Grid cell for a lat/lon — start/goal for A*."""
        e, n = self._latlon_to_xy(lat, lon)
        i, j = self._world_to_cell(np.array([e]), np.array([n]))
        return int(i[0]), int(j[0])


def astar(grid: np.ndarray, start, goal):
    """8-connected A* over planner_grid(). Returns list of (i,j) or None."""
    import heapq
    n, m = grid.shape
    if grid[start] or grid[goal]:
        return None
    h = lambda a: math.hypot(a[0] - goal[0], a[1] - goal[1])
    openq = [(h(start), 0.0, start, None)]
    came, gcost = {}, {start: 0.0}
    while openq:
        _, g, cur, par = heapq.heappop(openq)
        if cur in came:
            continue
        came[cur] = par
        if cur == goal:
            path = [cur]
            while came[path[-1]] is not None:
                path.append(came[path[-1]])
            return path[::-1]
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                if di == dj == 0:
                    continue
                nb = (cur[0] + di, cur[1] + dj)
                if not (0 <= nb[0] < n and 0 <= nb[1] < m) or grid[nb]:
                    continue
                ng = g + math.hypot(di, dj)
                if ng < gcost.get(nb, 1e18):
                    gcost[nb] = ng
                    heapq.heappush(openq, (ng + h(nb), ng, nb, cur))
    return None


if __name__ == "__main__":
    # smoke test: synthetic wall 3 m ahead, plan around it
    cfg = MapConfig()
    m = OccupancyMapper(cfg)
    depth = np.full((400, 640), 0, np.uint16)
    depth[100:300, 200:440] = 3000            # wall at 3 m, centre of view
    pose = {"lat": 13.03, "lon": 77.56, "relative_alt_m": 3.0, "yaw_deg": 0}
    for _ in range(4):                        # a few frames to pass threshold
        m.update(depth, pose)
    occ = m.occupancy()
    assert occ.sum() > 0, "no obstacles mapped"
    grid = m.planner_grid()
    assert grid.sum() > occ.sum(), "inflation failed"
    s = m.cell_of(13.03, 77.56)
    g = (s[0] + 40, s[1])                     # 10 m north
    path = astar(grid, s, g)
    assert path and path[0] == s and path[-1] == g, "A* failed"
    print(f"mapped {occ.sum()} occ cells -> {grid.sum()} inflated; "
          f"A* path {len(path)} cells around wall. OK")
