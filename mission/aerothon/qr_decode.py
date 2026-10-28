"""
QR decoding for the AeroTHON 2026 field spec: 1 m x 1 m prints with NO
white border (quiet zone).

Without a quiet zone, ZBar and cv2.QRCodeDetector usually fail to find the
finder patterns (verified on rendered field frames). The WeChat CNN
detector + super-resolution model (opencv-contrib, same models the team's
qr-detection pipeline uses) decodes them reliably, so it runs first.
Fallbacks: thresholding at several levels turns mid-tone ground into an
artificial quiet zone for the classic decoders.
"""

from __future__ import annotations

import os

import cv2
import numpy as np

_MODEL_DIR = os.path.join(os.path.dirname(__file__), "models", "wechat")
_wechat = None
_wechat_tried = False


def _get_wechat():
    global _wechat, _wechat_tried
    if not _wechat_tried:
        _wechat_tried = True
        try:
            files = ["detect.prototxt", "detect.caffemodel", "sr.prototxt", "sr.caffemodel"]
            # The binding name differs between OpenCV builds.
            mod = getattr(cv2, "wechat_qrcode", None)
            ctor = getattr(mod, "WeChatQRCode", None) or getattr(cv2, "wechat_qrcode_WeChatQRCode", None)
            if ctor is None:
                raise ImportError("WeChat QR module not in this OpenCV build")
            _wechat = ctor(*[os.path.join(_MODEL_DIR, f) for f in files])
        except Exception:  # noqa: BLE001 - contrib build or models missing
            _wechat = None
    return _wechat


def decode_qr_codes(bgr: np.ndarray) -> list[tuple[str, np.ndarray]]:
    """[(text, 4x2 int corner array), ...]"""
    results: list[tuple[str, np.ndarray]] = []
    wc = _get_wechat()
    if wc is not None:
        texts, points = wc.detectAndDecode(bgr)
        for t, pts in zip(texts, points):
            if t:
                results.append((t.strip(), np.asarray(pts).reshape(-1, 2).astype(int)))
        if results:
            return results

    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    stages = [gray] + [np.where(gray > t, 255, 0).astype(np.uint8) for t in (50, 70, 90, 120)]
    try:
        from pyzbar.pyzbar import decode as zbar_decode  # type: ignore

        for img in stages:
            for obj in zbar_decode(img):
                if obj.type == "QRCODE" and obj.data:
                    quad = np.array([[p.x, p.y] for p in obj.polygon], dtype=int)
                    results.append((obj.data.decode("utf-8", "ignore").strip(), quad))
            if results:
                return results
    except Exception:  # noqa: BLE001, S110 - pyzbar optional
        pass

    det = cv2.QRCodeDetector()
    for img in stages:
        ok, info, pts, _ = det.detectAndDecodeMulti(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR))
        if ok and pts is not None:
            for t, p in zip(info, pts):
                if t:
                    results.append((t.strip(), p.astype(int)))
            if results:
                return results
    return results
