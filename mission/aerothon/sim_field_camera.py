"""
FieldSimCamera — renders the AeroTHON 2026 field (per field_spec.py) from
the drone's current pose, so the mission FSM sees banners, corridor walls,
obstacles, QR codes, the red zone and the landing pad in software sim and
in ArduPilot SITL.

Why this exists: in `--sim` mode the FSM used MockCamera, which returns a
flat grey frame forever, and with SITL the OAK-D falls back to the laptop
webcam. Neither ever contains a banner, so FIND_GREEN_BANNER could only
ever report "not found" and force corridor entry after a 360 deg sweep.

It is a small numpy ray caster (no Gazebo needed): axis-aligned boxes plus a
ground plane, RGB and z-depth (mm, like the OAK-D `depth` stream).

    cam = FieldSimCamera(fc, target_id="R2D2_TARGET")
    bgr, depth_mm = cam.read()      # same interface as the other cameras
"""

from __future__ import annotations

import dataclasses
import math

import cv2
import numpy as np

from aerothon.field_spec import FIELD, FieldSpec


@dataclasses.dataclass
class Box:
    name: str
    lo: tuple[float, float, float]  # (x, y, z) min, z up
    hi: tuple[float, float, float]
    bgr: tuple[int, int, int]
    texture: np.ndarray | None = None  # mapped onto the top face (x->rows, y->cols)


def _rgb2bgr(c):
    return (c[2], c[1], c[0])


def _qr_texture(text: str) -> np.ndarray:
    """QR image with NO quiet zone (spec: 'No white borders, just the QR')."""
    img = cv2.QRCodeEncoder.create().encode(text)  # 0/255, includes quiet zone
    ys, xs = np.where(img == 0)
    img = img[ys.min() : ys.max() + 1, xs.min() : xs.max() + 1]
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def build_field(
    f: FieldSpec = FIELD,
    start_qr_text: str = "R2D2_TARGET",
    target_qr_text: str = "R2D2_TARGET",
    decoy_qr_texts: tuple[str, ...] = ("TEAM_B", "TEAM_C", "TEAM_D", "TEAM_E"),
) -> list[Box]:
    boxes: list[Box] = []
    white = (245, 245, 245)
    t = 0.05  # wall / board thickness
    hw = f.corridor_half_width

    # take-off / landing podium (peach, 4x4 ft, 6 in high)
    p = f.PAD_SIZE_M / 2
    boxes.append(Box("pad", (-p, -p, 0), (p, p, f.PAD_HEIGHT_M), (130, 170, 240)))

    # team's start QR on the ground near take-off
    q = f.QR_SIZE_M / 2
    sx = f.START_QR_X_M
    boxes.append(
        Box("start_qr", (sx - q, -q, 0), (sx + q, q, 0.005), white, _qr_texture(start_qr_text))
    )

    x0, x1 = f.CORRIDOR_ENTRY_X_M, f.corridor_exit_x
    H = f.CORRIDOR_WALL_HEIGHT_M
    for cy, tag in ((f.FORWARD_CORRIDOR_Y_M, "fwd"), (f.RETURN_CORRIDOR_Y_M, "ret")):
        for side in (-1, 1):
            yw = cy + side * hw
            boxes.append(Box(f"{tag}_wall_{side}", (x0, yw - t / 2, 0), (x1, yw + t / 2, H), white))

    def banner(name, cy, x_face, z_lo, z_hi, rgb, ground=False, x_dir=-1):
        b = f.BANNER_BORDER_M
        L2 = f.BANNER_LENGTH_M / 2
        if ground:  # flat on the ground just outside the corridor mouth
            xa = x_face + x_dir * 0.10
            xb = xa + x_dir * f.GROUND_BANNER_WIDTH_M
            xl, xh = min(xa, xb), max(xa, xb)
            boxes.append(Box(name + "_border", (xl - b, cy - L2 - b, 0), (xh + b, cy + L2 + b, 0.004), white))
            boxes.append(Box(name, (xl, cy - L2, 0), (xh, cy + L2, 0.008), _rgb2bgr(rgb)))
        else:  # vertical foam board on the arch above the corridor mouth
            boxes.append(Box(name + "_border", (x_face - 0.02, cy - L2 - b, z_lo - b), (x_face + 0.02, cy + L2 + b, z_hi + b), white))
            xf = x_face + x_dir * 0.025
            boxes.append(Box(name, (min(xf, xf + x_dir * 0.01), cy - L2, z_lo), (max(xf, xf + x_dir * 0.01), cy + L2, z_hi), _rgb2bgr(rgb)))

    # entry corridor: green, facing the take-off side
    ah = f.ARCH_BANNER_HEIGHT_M
    banner("green_arch", f.FORWARD_CORRIDOR_Y_M, x0, H, H + ah, f.GREEN_BANNER_RGB, x_dir=-1)
    banner("green_ground", f.FORWARD_CORRIDOR_Y_M, x0, 0, 0, f.GREEN_BANNER_RGB, ground=True, x_dir=-1)
    # exit corridor: blue, at its delivery-side mouth (entered on the return lap)
    banner("blue_arch", f.RETURN_CORRIDOR_Y_M, x1, H, H + ah, f.BLUE_BANNER_RGB, x_dir=+1)
    banner("blue_ground", f.RETURN_CORRIDOR_Y_M, x1, 0, 0, f.BLUE_BANNER_RGB, ground=True, x_dir=+1)

    # 3 static obstacles staggered inside the exit corridor
    ow, oh = f.OBSTACLE_WIDTH_M, f.OBSTACLE_HEIGHT_M
    for i, (fx, oy) in enumerate(((0.25, 0.9), (0.5, -0.9), (0.75, 0.9))):
        xc = x0 + fx * f.CORRIDOR_LENGTH_M
        yc = f.RETURN_CORRIDOR_Y_M + oy
        boxes.append(Box(f"obstacle_{i}", (xc - t / 2, yc - ow / 2, 0), (xc + t / 2, yc + ow / 2, oh), (210, 210, 215)))

    # delivery zone: 5 QR codes + red zone
    dz = x1 + f.DELIVERY_ZONE_GAP_M
    texts = (target_qr_text,) + tuple(decoy_qr_texts)[: f.QR_COUNT_DELIVERY - 1]
    spots = ((dz + 8, 0.0), (dz + 4, -6.0), (dz + 14, 7.0), (dz + 18, -8.0), (dz + 24, 3.0))
    for txt, (qx, qy) in zip(texts, spots):
        boxes.append(Box(f"qr_{txt}", (qx - q, qy - q, 0), (qx + q, qy + q, 0.005), white, _qr_texture(txt)))
    r = f.RED_ZONE_SIZE_M / 2
    rx, ry = dz + 14, -1.0
    b = f.BANNER_BORDER_M
    boxes.append(Box("red_border", (rx - r - b, ry - r - b, 0), (rx + r + b, ry + r + b, 0.003), white))
    boxes.append(Box("red_zone", (rx - r, ry - r, 0), (rx + r, ry + r, 0.006), (0, 0, 255)))
    return boxes


class FieldSimCamera:
    """Forward camera pitched down `pitch_deg` (OAK-D mount is 45 deg)."""

    GROUND_BGR = (96, 112, 104)  # low-saturation grey-green, not banner-coloured

    def __init__(
        self,
        fc,
        boxes: list[Box] | None = None,
        width: int = 640,
        height: int = 480,
        hfov_deg: float = 73.0,
        pitch_deg: float = 45.0,
        noise: float = 3.0,
        target_id: str = "R2D2_TARGET",
    ) -> None:
        self.fc = fc
        self.boxes = boxes if boxes is not None else build_field(start_qr_text=target_id, target_qr_text=target_id)
        self.w, self.h = width, height
        self.pitch = math.radians(pitch_deg)
        self.noise = noise
        fx = (width / 2) / math.tan(math.radians(hfov_deg) / 2)
        us, vs = np.meshgrid(np.arange(width), np.arange(height))
        self._xn = ((us - width / 2) / fx).astype(np.float32).ravel()  # right
        self._yn = ((vs - height / 2) / fx).astype(np.float32).ravel()  # down
        self._rng = np.random.default_rng(0)

    # ---------------------------------------------------------------- pose
    def _pose(self):
        x, y, z_ned, yaw = self.fc.get_position_ned()
        alt = max(0.05, -z_ned)
        return x, y, alt, math.radians(yaw)

    def render(self, x: float, y: float, alt: float, yaw: float):
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        # camera (f=1, right=xn, down=yn) -> body FRD with pitch-down
        fwd = cp - self._yn * sp
        down = sp + self._yn * cp
        right = self._xn
        cy_, sy_ = math.cos(yaw), math.sin(yaw)
        dx = fwd * cy_ - right * sy_
        dy = fwd * sy_ + right * cy_
        dz = -down
        o = (np.float32(x), np.float32(y), np.float32(alt))
        d = (dx, dy, dz)
        n = dx.size

        t_best = np.full(n, np.inf, np.float32)
        img = np.empty((n, 3), np.float32)
        img[:] = self.GROUND_BGR
        # ground plane z = 0
        with np.errstate(divide="ignore", invalid="ignore"):
            tg = np.where(dz < -1e-6, -o[2] / dz, np.inf)
        t_best = np.minimum(t_best, tg)

        for bx in self.boxes:
            with np.errstate(divide="ignore", invalid="ignore"):
                tmin = np.full(n, -np.inf, np.float32)
                tmax = np.full(n, np.inf, np.float32)
                axis_hit = np.zeros(n, np.int8)
                for a in range(3):
                    inv = 1.0 / d[a]
                    t1 = (bx.lo[a] - o[a]) * inv
                    t2 = (bx.hi[a] - o[a]) * inv
                    ta, tb = np.minimum(t1, t2), np.maximum(t1, t2)
                    upd = ta > tmin
                    axis_hit[upd] = a
                    tmin = np.where(upd, ta, tmin)
                    tmax = np.minimum(tmax, tb)
            hit = (tmax >= tmin) & (tmin > 0) & (tmin < t_best)
            if not hit.any():
                continue
            t_best[hit] = tmin[hit]
            shade = np.where(axis_hit[hit] == 2, 1.0, 0.82)[:, None]
            if bx.texture is not None:
                col = np.empty((hit.sum(), 3), np.float32)
                col[:] = bx.bgr
                top = axis_hit[hit] == 2
                if top.any():
                    th = hit.copy()
                    th[hit] = top
                    px = o[0] + dx[th] * t_best[th]
                    py = o[1] + dy[th] * t_best[th]
                    tex = bx.texture
                    u = np.clip((px - bx.lo[0]) / (bx.hi[0] - bx.lo[0]), 0, 0.9999)
                    v = np.clip((py - bx.lo[1]) / (bx.hi[1] - bx.lo[1]), 0, 0.9999)
                    # texture row 0 at the far (+x) edge so it reads upright from take-off
                    rows = ((1 - u) * tex.shape[0]).astype(int)
                    cols = (v * tex.shape[1]).astype(int)
                    col[top] = tex[rows, cols]
                img[hit] = col * shade
            else:
                img[hit] = np.array(bx.bgr, np.float32) * shade

        sky = ~np.isfinite(t_best)
        img[sky] = (235, 206, 170)
        if self.noise:
            img += self._rng.normal(0, self.noise, img.shape).astype(np.float32)
        bgr = np.clip(img, 0, 255).astype(np.uint8).reshape(self.h, self.w, 3)
        depth = np.where(np.isfinite(t_best), t_best * 1000.0, 0)
        depth = np.clip(depth, 0, 65535).astype(np.uint16).reshape(self.h, self.w)
        depth[depth > 20000] = 0  # OAK-D stereo gives no depth this far
        return bgr, depth

    def read(self):
        return self.render(*self._pose())

    def release(self) -> None:
        pass
