"""Synthetic aerial QR frame generator.

Simulates what the OAK-D Lite RGB camera sees during Mission 2:
QR codes on the ground viewed from altitude with a 45-degree camera tilt,
under varying distance, lighting, and motion blur.

Camera model: OAK-D Lite RGB ~69 deg HFOV. At 1920 px width,
focal length f ~= 1920 / (2*tan(34.5 deg)) ~= 1397 px.
Apparent QR size in pixels = f * qr_size_m / slant_range_m.
"""
import numpy as np
import cv2
import qrcode
import random

FRAME_W, FRAME_H = 1920, 1080
FOCAL_PX = FRAME_W / (2 * np.tan(np.radians(69 / 2)))  # ~1397 px
QR_SIZE_M = 0.40  # assumed physical QR size (rulebook value pending)


def make_qr(text: str, px: int = 640) -> np.ndarray:
    """Render a QR code as a BGR image with proper quiet zone."""
    q = qrcode.QRCode(border=2, box_size=10,
                      error_correction=qrcode.constants.ERROR_CORRECT_M)
    q.add_data(text)
    q.make(fit=True)
    img = q.make_image(fill_color="black", back_color="white").convert("RGB")
    arr = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
    return cv2.resize(arr, (px, px), interpolation=cv2.INTER_AREA)


def make_background(rng: random.Random) -> np.ndarray:
    """Procedural ground texture: grass / concrete / asphalt with clutter."""
    base_choices = [(60, 110, 70), (150, 150, 145), (80, 80, 82)]  # BGR
    base = np.array(rng.choice(base_choices), dtype=np.float32)
    bg = np.full((FRAME_H, FRAME_W, 3), base, dtype=np.float32)
    noise = np.random.default_rng(rng.randrange(1 << 30)).normal(
        0, 18, (FRAME_H // 4, FRAME_W // 4, 3)).astype(np.float32)
    noise = cv2.resize(noise, (FRAME_W, FRAME_H))
    bg = np.clip(bg + noise, 0, 255).astype(np.uint8)
    # clutter: painted lines, zone rectangles, occasional red patch
    for _ in range(rng.randint(2, 6)):
        p1 = (rng.randrange(FRAME_W), rng.randrange(FRAME_H))
        p2 = (rng.randrange(FRAME_W), rng.randrange(FRAME_H))
        col = tuple(rng.randrange(60, 220) for _ in range(3))
        cv2.line(bg, p1, p2, col, rng.randint(2, 12))
    if rng.random() < 0.4:  # red zone patch
        x, y = rng.randrange(FRAME_W - 300), rng.randrange(FRAME_H - 200)
        cv2.rectangle(bg, (x, y), (x + rng.randint(120, 300), y + rng.randint(80, 200)),
                      (40, 40, 200), -1)
    return bg


def qr_pixel_size(distance_m: float, qr_size_m: float = QR_SIZE_M) -> int:
    return max(8, int(round(FOCAL_PX * qr_size_m / distance_m)))


def composite_qr(bg: np.ndarray, qr: np.ndarray, size_px: int,
                 tilt_deg: float, yaw_deg: float, rng: random.Random):
    """Perspective-warp the QR to simulate oblique viewing and paste it.

    Returns (frame, corner_points 4x2) of the pasted QR.
    """
    h, w = qr.shape[:2]
    src = np.float32([[0, 0], [w, 0], [w, h], [0, h]])

    # oblique compression from camera tilt (foreshortening along view axis)
    squash = np.cos(np.radians(tilt_deg))
    tw, th = size_px, max(6, int(size_px * squash))
    # mild trapezoid: far edge slightly narrower
    shrink = 0.06 + 0.10 * (tilt_deg / 60.0)
    dst = np.float32([[tw * shrink, 0], [tw * (1 - shrink), 0], [tw, th], [0, th]])

    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(qr, M, (tw, th), borderValue=(255, 255, 255))

    # in-plane rotation (drone yaw relative to code)
    Mr = cv2.getRotationMatrix2D((tw / 2, th / 2), yaw_deg, 1.0)
    cosv, sinv = abs(Mr[0, 0]), abs(Mr[0, 1])
    nw, nh = int(th * sinv + tw * cosv), int(th * cosv + tw * sinv)
    Mr[0, 2] += nw / 2 - tw / 2
    Mr[1, 2] += nh / 2 - th / 2
    rotated = cv2.warpAffine(warped, Mr, (nw, nh), borderValue=(255, 255, 255))

    x = rng.randrange(50, FRAME_W - nw - 50)
    y = rng.randrange(50, FRAME_H - nh - 50)
    frame = bg.copy()
    # paste through a mask so the white quiet zone stays but corners of the
    # rotated bounding box (pure white fill) blend less harshly
    frame[y:y + nh, x:x + nw] = rotated
    pts = np.float32([[x, y], [x + nw, y], [x + nw, y + nh], [x, y + nh]])
    return frame, pts


def apply_conditions(frame: np.ndarray, lighting: str, blur_px: int,
                     rng: random.Random) -> np.ndarray:
    out = frame.astype(np.float32)
    if lighting == "shade":       # dim corridor
        out = out * rng.uniform(0.35, 0.55)
    elif lighting == "sun":       # overexposed, washed out
        out = out * rng.uniform(1.35, 1.7) + rng.uniform(20, 45)
    out = np.clip(out, 0, 255).astype(np.uint8)
    if blur_px > 1:               # linear motion blur
        k = np.zeros((blur_px, blur_px), np.float32)
        k[blur_px // 2, :] = 1.0 / blur_px
        ang = rng.uniform(0, 180)
        M = cv2.getRotationMatrix2D((blur_px / 2, blur_px / 2), ang, 1.0)
        k = cv2.warpAffine(k, M, (blur_px, blur_px))
        s = k.sum()
        if s > 0:
            out = cv2.filter2D(out, -1, k / s)
    gauss = np.random.default_rng(rng.randrange(1 << 30)).normal(
        0, 4, out.shape).astype(np.float32)
    out = np.clip(out.astype(np.float32) + gauss, 0, 255).astype(np.uint8)
    return out


def generate_frame(text: str, distance_m: float, lighting: str,
                   blur_px: int, seed: int):
    """One synthetic mission frame. Returns (frame, qr_corner_pts)."""
    rng = random.Random(seed)
    bg = make_background(rng)
    qr = make_qr(text)
    size = qr_pixel_size(distance_m)
    tilt = rng.uniform(35, 50)      # around the 45 deg mount
    yaw = rng.uniform(-30, 30)
    frame, pts = composite_qr(bg, qr, size, tilt, yaw, rng)
    frame = apply_conditions(frame, lighting, blur_px, rng)
    return frame, pts
