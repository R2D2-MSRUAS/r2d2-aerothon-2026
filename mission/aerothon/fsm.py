"""
AeroTHON 2026 — Mission Finite State Machine (Forward & Backward Passes)
================================================================================
Manages complete mission lifecycle:
  1. Takeoff to 5m
  2. Scan start QR (~1m fwd) & decode delivery location
  3. Detect green banner at corridor entrance & align
  4. Descend to 10 ft (~3m) & traverse 3.5m corridor (Forward Pass)
  5. Enter delivery arena & ascend to 10m search altitude
  6. Detect target QR among candidates & reject red no-fly zones
  7. Descend to 5m & precision visual alignment over target point
  8. Winch lower payload, release, retract cable, apply CoG pitch trim
  9. Ascend to 10m return altitude & rotate 180° towards corridor
  10. Descend to 10 ft (~3m) & traverse corridor (Backward Pass)
  11. Exit corridor, vector to Home Launch Pad (0, 0)
  12. Precision autonomous landing & disarm
"""

from __future__ import annotations

import logging
import math
import time
from typing import Any

import cv2
import numpy as np

from aerothon.config import DEFAULT_CONFIG, MissionConfig, MissionState
from aerothon.coordinates import (
    CoordinateManager,
)
from aerothon.flight_controller import FlightControllerInterface, execute_winch_delivery
from aerothon.recorder import FlightDataRecorder
from aerothon.vision import (
    CameraInterface,
    PerceptionPipeline,
    annotate_mission_frame,
    depth_at_bbox,
)

log = logging.getLogger("AeroTHON2026.FSM")


class MissionFSM:
    """
    Finite State Machine driving the AeroTHON 2026 Track 1 Mission.
    Can be stepped frame-by-frame or executed in a continuous loop.
    """

    def __init__(
        self,
        fc: FlightControllerInterface,
        camera: CameraInterface,
        coord_mgr: CoordinateManager | None = None,
        perception: PerceptionPipeline | None = None,
        recorder: FlightDataRecorder | None = None,
        config: MissionConfig = DEFAULT_CONFIG,
    ) -> None:
        self.fc = fc
        self.camera = camera
        self.config = config
        self.coord_mgr = coord_mgr or CoordinateManager(config=config)
        self.perception = perception or PerceptionPipeline(config=config)
        self.recorder = recorder

        # State Variables
        self.state = MissionState.TAKEOFF
        self.target_id = ""
        self.qr_fail_cnt = 0
        self.qr_confirm_cnt = 0
        self.banner_search_cnt = 0
        self.state_before_avoid = MissionState.FOLLOW_CORRIDOR
        self.mission_start_time = time.time()
        self.step_count = 0

    def step(self, bgr: np.ndarray, depth_mm: np.ndarray | None) -> dict[str, Any]:
        """
        Executes a single FSM iteration.
        Returns a dict containing telemetry, percepts, state, and coordinates.
        """
        self.step_count += 1

        # 1. Update Coordinates from Flight Controller
        gx, gy, gz, yaw = self.fc.get_position_ned()
        self.coord_mgr.update_from_fc(gx, gy, gz, yaw)
        self.coord_mgr.record_path_point(self.state, gx, gy, gz, yaw)

        # 2. Run Perception Pipeline
        percept = self.perception.perceive(bgr, depth_mm)
        obstacles = percept["obstacles"]
        humans = percept["humans"]
        red_zones = percept["red_zones"]
        qr_codes = percept["qr_codes"]
        banner_box = percept["banner_box"]
        _steering = percept["steering"]
        fwd_dist = percept["fwd_dist_m"]
        d_left_w = percept["d_left_wall"]
        d_right_w = percept["d_right_wall"]

        # 3. Update Local Section Coordinates
        tgt_dist = None
        for txt, qd in qr_codes:
            if txt == self.target_id and self.target_id:
                qx, qy = int(np.mean(qd[:, 0])), int(np.mean(qd[:, 1]))
                tgt_dist = depth_at_bbox(depth_mm, (qx - 30, qy - 30, 60, 60))
                break

        lpos = self.coord_mgr.compute_local_coordinates(self.state, d_left_w, d_right_w, fwd_dist, tgt_dist)

        if self.recorder is not None:
            self.recorder.record(self.state, self.coord_mgr.global_pos, lpos, percept, self.target_id)

        # 4. Global Obstacle / Human Safety Check
        safe_exempt = {
            MissionState.INIT,
            MissionState.TAKEOFF,
            MissionState.PRECISION_LAND,
            MissionState.DROP_PAYLOAD,
            MissionState.AVOID_OBSTACLES,
            MissionState.COMPLETE,
        }
        if self.state not in safe_exempt:
            threats = [t for t in (obstacles + humans) if t.get("action") == "STOP"]
            if threats:
                self.state_before_avoid = self.state
                log.warning(f"[SAFETY] Emergency STOP threat in {self.state.name} -> Entering AVOID_OBSTACLES!")
                self.fc.stop()
                self.state = MissionState.AVOID_OBSTACLES

        # ================================================================
        # STATE MACHINE TRANSITIONS & CONTROL
        # ================================================================

        # ----------------------------------------------------------------
        # PHASE 1: TAKEOFF (Rule: 5m altitude)
        # ----------------------------------------------------------------
        if self.state == MissionState.TAKEOFF:
            self.fc.arm_and_takeoff(self.config.TAKEOFF_ALT_M)
            log.info("[TAKEOFF] Target altitude reached -> SCAN_START_QR")
            self.state = MissionState.SCAN_START_QR

        # ----------------------------------------------------------------
        # PHASE 2: SCAN START QR (Rule: at 5m, move fwd ~1m and scan QR)
        # ----------------------------------------------------------------
        elif self.state == MissionState.SCAN_START_QR:
            if qr_codes:
                self.target_id = qr_codes[0][0]
                self.coord_mgr.register_landmark(
                    "START_QR",
                    self.coord_mgr.global_pos.x,
                    self.coord_mgr.global_pos.y,
                    self.coord_mgr.global_pos.z,
                )
                log.info(f"[START_QR] Target Decoded: '{self.target_id}' -> FIND_GREEN_BANNER")
                self.state = MissionState.FIND_GREEN_BANNER
                self.qr_fail_cnt = 0
            else:
                self.qr_fail_cnt += 1
                if self.qr_fail_cnt < 8:
                    self.fc.send_body_velocity(vx=0.4, vy=0.0, vz=0.0, duration=0.3)
                elif self.qr_fail_cnt < 20:
                    self.fc.set_yaw(20.0, relative=True)
                else:
                    log.warning("[START_QR] Search timeout. Defaulting target to 'TARGET_A'")
                    self.target_id = "TARGET_A"
                    self.state = MissionState.FIND_GREEN_BANNER

        # ----------------------------------------------------------------
        # PHASE 3: CORRIDOR ENTRY DETECTION (Green Banner Alignment)
        # ----------------------------------------------------------------
        elif self.state == MissionState.FIND_GREEN_BANNER:
            if banner_box is not None:
                bx, _by, bw, _bh = banner_box
                frame_cx = bgr.shape[1] // 2
                banner_cx = bx + bw // 2
                banner_dist = depth_at_bbox(depth_mm, banner_box)
                if math.isinf(banner_dist) or banner_dist > 40.0:
                    banner_dist = 6.0

                lateral_err_px = banner_cx - frame_cx
                log.info(f"[BANNER] Green Banner locked: Dist={banner_dist:.2f}m, CenterErr={lateral_err_px}px")

                if abs(lateral_err_px) > 35:
                    vy = max(-0.4, min(0.4, 0.003 * lateral_err_px))
                    self.fc.send_body_velocity(vx=0.0, vy=vy, vz=0.0, duration=0.3)
                elif banner_dist > 2.5:
                    self.fc.send_body_velocity(vx=self.config.SLOW_SPEED_MPS, vy=0.0, vz=0.0, duration=0.3)
                else:
                    self.coord_mgr.register_landmark(
                        "CORRIDOR_ENTRY",
                        self.coord_mgr.global_pos.x,
                        self.coord_mgr.global_pos.y,
                        self.coord_mgr.global_pos.z,
                    )
                    log.info("[BANNER] Corridor entry reached. Descending to 10 ft (3m) -> ENTER_CORRIDOR")
                    self.fc.set_altitude(self.config.CORRIDOR_ALT_M)
                    self.state = MissionState.ENTER_CORRIDOR
                self.banner_search_cnt = 0
            else:
                self.banner_search_cnt += 1
                self.fc.set_yaw(15.0, relative=True)
                if self.banner_search_cnt > 24:
                    log.warning("[BANNER] Full 360° sweep completed. Forcing corridor entry.")
                    self.fc.set_altitude(self.config.CORRIDOR_ALT_M)
                    self.state = MissionState.ENTER_CORRIDOR

        # ----------------------------------------------------------------
        # PHASE 4: ENTER CORRIDOR
        # ----------------------------------------------------------------
        elif self.state == MissionState.ENTER_CORRIDOR:
            if d_left_w < 3.0 and d_right_w < 3.0:
                log.info(f"[ENTER_CORRIDOR] Walls detected (L:{d_left_w:.2f}m, R:{d_right_w:.2f}m) -> FOLLOW_CORRIDOR")
                self.state = MissionState.FOLLOW_CORRIDOR
            else:
                self.fc.send_body_velocity(vx=self.config.SLOW_SPEED_MPS, vy=0.0, vz=0.0, duration=0.3)

        # ----------------------------------------------------------------
        # PHASE 5: FORWARD CORRIDOR NAVIGATION (3.5m width)
        # ----------------------------------------------------------------
        elif self.state == MissionState.FOLLOW_CORRIDOR:
            lat_center_err = (d_right_w - d_left_w) / 2.0 if (d_left_w > 0 and d_right_w > 0) else 0.0
            vy_corr = max(-0.4, min(0.4, 0.45 * lat_center_err))

            if (fwd_dist > 6.0 and d_left_w > 3.0 and d_right_w > 3.0) or (lpos.x_rel >= self.config.CORRIDOR_LENGTH_EST_M):
                self.coord_mgr.register_landmark(
                    "CORRIDOR_EXIT",
                    self.coord_mgr.global_pos.x,
                    self.coord_mgr.global_pos.y,
                    self.coord_mgr.global_pos.z,
                )
                log.info("[CORRIDOR_FWD] Exited corridor -> REACH_DELIVERY")
                self.fc.stop()
                self.state = MissionState.REACH_DELIVERY
            elif fwd_dist < self.config.OBS_SLOW_DIST_M:
                self.fc.send_body_velocity(vx=self.config.SLOW_SPEED_MPS * 0.5, vy=vy_corr, vz=0.0, duration=0.3)
            else:
                self.fc.send_body_velocity(vx=self.config.CORRIDOR_SPEED_MPS, vy=vy_corr, vz=0.0, duration=0.3)

        # ----------------------------------------------------------------
        # PHASE 6: REACH DELIVERY ARENA (Ascend to 10m altitude)
        # ----------------------------------------------------------------
        elif self.state == MissionState.REACH_DELIVERY:
            log.info(f"[DELIVERY] Ascending to {self.config.DELIVERY_SEARCH_ALT_M:.1f} m search altitude -> SCAN_TARGET_QR")
            self.fc.set_altitude(self.config.DELIVERY_SEARCH_ALT_M)
            self.state = MissionState.SCAN_TARGET_QR
            self.qr_confirm_cnt = 0

        # ----------------------------------------------------------------
        # PHASE 7: SCAN TARGET QR & AVOID RED NO-FLY ZONES
        # ----------------------------------------------------------------
        elif self.state == MissionState.SCAN_TARGET_QR:
            matched_qr = None
            for txt, quad in qr_codes:
                if txt != self.target_id:
                    continue
                qx, qy = int(np.mean(quad[:, 0])), int(np.mean(quad[:, 1]))
                in_red = any(rx <= qx <= rx + rw and ry <= qy <= ry + rh for rx, ry, rw, rh in red_zones)
                if in_red:
                    log.warning(f"[SCAN_TARGET] QR '{txt}' is in a RED ZONE! Skipping.")
                    continue
                matched_qr = quad
                break

            if matched_qr is not None:
                self.qr_confirm_cnt += 1
                log.info(f"[SCAN_TARGET] Target matched ({self.qr_confirm_cnt}/{self.config.QR_CONFIRM_COUNT})")
                if self.qr_confirm_cnt >= self.config.QR_CONFIRM_COUNT:
                    self.coord_mgr.register_landmark(
                        "TARGET_QR",
                        self.coord_mgr.global_pos.x,
                        self.coord_mgr.global_pos.y,
                        self.coord_mgr.global_pos.z,
                    )
                    log.info("[SCAN_TARGET] Target Confirmed! Descending to 5m -> ALIGN_PAYLOAD")
                    self.fc.set_altitude(self.config.PAYLOAD_DROP_ALT_M)
                    self.state = MissionState.ALIGN_PAYLOAD
            else:
                self.qr_confirm_cnt = 0
                self.fc.send_body_velocity(vx=0.3, vy=0.2, vz=0.0, duration=0.4)

        # ----------------------------------------------------------------
        # PHASE 8: PRECISION VISUAL ALIGNMENT OVER TARGET
        # ----------------------------------------------------------------
        elif self.state == MissionState.ALIGN_PAYLOAD:
            frame_cx = bgr.shape[1] // 2
            frame_cy = bgr.shape[0] // 2
            aligned = False

            for txt, quad in qr_codes:
                if txt != self.target_id:
                    continue
                qx = int(np.mean(quad[:, 0]))
                qy = int(np.mean(quad[:, 1]))
                err_x = qx - frame_cx
                err_y = qy - frame_cy

                vy = max(-0.25, min(0.25, 0.0025 * err_x))
                vx = max(-0.25, min(0.25, -0.0025 * err_y))

                if abs(err_x) < 25 and abs(err_y) < 25:
                    aligned = True
                else:
                    self.fc.send_body_velocity(vx=vx, vy=vy, vz=0.0, duration=0.2)
                break

            if aligned:
                log.info("[ALIGN] Centered over target -> DROP_PAYLOAD")
                self.fc.stop()
                self.state = MissionState.DROP_PAYLOAD

        # ----------------------------------------------------------------
        # PHASE 9: PAYLOAD DROP SEQUENCE
        # ----------------------------------------------------------------
        elif self.state == MissionState.DROP_PAYLOAD:
            execute_winch_delivery(self.fc, self.config)
            log.info("[PAYLOAD] Winch drop complete -> ASCEND_RETURN")
            self.state = MissionState.ASCEND_RETURN

        # ----------------------------------------------------------------
        # PHASE 10: ASCEND TO RETURN ALTITUDE (10m)
        # ----------------------------------------------------------------
        elif self.state == MissionState.ASCEND_RETURN:
            self.fc.set_altitude(self.config.RETURN_CRUISE_ALT_M)
            log.info(f"[RETURN] Reached {self.config.RETURN_CRUISE_ALT_M}m. Rotating 180° -> ALIGN_RETURN_CORRIDOR")
            self.fc.set_yaw(180.0, relative=True)
            self.state = MissionState.ALIGN_RETURN_CORRIDOR

        # ----------------------------------------------------------------
        # PHASE 11: ALIGN RETURN CORRIDOR
        # ----------------------------------------------------------------
        elif self.state == MissionState.ALIGN_RETURN_CORRIDOR:
            self.fc.set_altitude(self.config.RETURN_CORRIDOR_ALT_M)
            if banner_box is not None or (d_left_w < 3.0 and d_right_w < 3.0):
                log.info("[RETURN_CORRIDOR] Entrance acquired -> FOLLOW_CORRIDOR_RETURN")
                self.state = MissionState.FOLLOW_CORRIDOR_RETURN
            else:
                self.fc.send_body_velocity(vx=self.config.SLOW_SPEED_MPS, vy=0.0, vz=0.0, duration=0.3)

        # ----------------------------------------------------------------
        # PHASE 12: BACKWARD CORRIDOR NAVIGATION (Return Pass)
        # ----------------------------------------------------------------
        elif self.state == MissionState.FOLLOW_CORRIDOR_RETURN:
            lat_center_err = (d_right_w - d_left_w) / 2.0 if (d_left_w > 0 and d_right_w > 0) else 0.0
            vy_corr = max(-0.4, min(0.4, 0.45 * lat_center_err))

            if (d_left_w > 4.0 and d_right_w > 4.0) or (self.coord_mgr.global_pos.x <= 2.5):
                log.info("[RETURN_CORRIDOR] Cleared return corridor -> APPROACH_HOME")
                self.state = MissionState.APPROACH_HOME
            elif fwd_dist < self.config.OBS_SLOW_DIST_M:
                self.fc.send_body_velocity(vx=self.config.SLOW_SPEED_MPS * 0.5, vy=vy_corr, vz=0.0, duration=0.3)
            else:
                self.fc.send_body_velocity(vx=self.config.CORRIDOR_SPEED_MPS, vy=vy_corr, vz=0.0, duration=0.3)

        # ----------------------------------------------------------------
        # PHASE 13: APPROACH HOME LAUNCH PAD (0, 0)
        # ----------------------------------------------------------------
        elif self.state == MissionState.APPROACH_HOME:
            gx, gy = self.coord_mgr.global_pos.x, self.coord_mgr.global_pos.y
            dist_home = math.sqrt(gx**2 + gy**2)
            log.info(f"[HOME_APPROACH] Dist to Home: {dist_home:.2f} m")

            if dist_home < 0.6:
                log.info("[HOME_APPROACH] Over Home Pad -> PRECISION_LAND")
                self.fc.stop()
                self.state = MissionState.PRECISION_LAND
            else:
                yaw_rad = math.radians(self.coord_mgr.global_pos.yaw_deg)
                dx, dy = -gx, -gy
                vx_body = dx * math.cos(yaw_rad) + dy * math.sin(yaw_rad)
                vy_body = -dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)
                spd = min(self.config.SLOW_SPEED_MPS, max(0.2, dist_home * 0.5))
                mag = math.sqrt(vx_body**2 + vy_body**2)
                if mag > 0.01:
                    vx_b = (vx_body / mag) * spd
                    vy_b = (vy_body / mag) * spd
                    self.fc.send_body_velocity(vx=vx_b, vy=vy_b, vz=0.0, duration=0.3)
                else:
                    self.fc.send_body_velocity(vx=0.0, vy=0.0, vz=0.0, duration=0.2)

        # ----------------------------------------------------------------
        # PHASE 14: PRECISION AUTONOMOUS LANDING
        # ----------------------------------------------------------------
        elif self.state == MissionState.PRECISION_LAND:
            self.fc.land()
            log.info("[LAND] Touchdown complete -> COMPLETE")
            self.state = MissionState.COMPLETE

        # ----------------------------------------------------------------
        # REACTIVE COLLISION AVOIDANCE
        # ----------------------------------------------------------------
        elif self.state == MissionState.AVOID_OBSTACLES:
            threats = [t for t in (obstacles + humans) if t.get("action") in ("STOP", "SLOW")]
            if not threats:
                log.info(f"[AVOID] Pathway clear -> Resuming {self.state_before_avoid.name}")
                self.state = self.state_before_avoid
            else:
                closest = min(threats, key=lambda t: t["dist_m"])
                cx = closest["cx"]
                fw = bgr.shape[1]
                if cx < fw // 2:
                    self.fc.send_body_velocity(vx=0.0, vy=self.config.SLOW_SPEED_MPS, vz=0.0, duration=0.3)
                else:
                    self.fc.send_body_velocity(vx=0.0, vy=-self.config.SLOW_SPEED_MPS, vz=0.0, duration=0.3)
                if closest["dist_m"] < 0.6:
                    self.fc.send_body_velocity(vx=-0.5, vy=0.0, vz=0.0, duration=0.3)

        return {
            "state": self.state,
            "target_id": self.target_id,
            "global_pos": self.coord_mgr.global_pos,
            "local_pos": lpos,
            "percept": percept,
        }

    def run(self, headless: bool = False, max_duration_s: float | None = None) -> None:
        """Runs the continuous mission loop until completion or timeout."""
        start_time = time.time()
        timeout = max_duration_s or self.config.MAX_MISSION_TIME_S

        while self.state != MissionState.COMPLETE:
            if time.time() - start_time > timeout:
                log.warning(f"[MISSION TIMEOUT] Exceeded {timeout:.1f}s limit. Triggering RTL failsafe.")
                self.fc.return_to_launch()
                break

            rgb, depth_mm = self.camera.read()
            if rgb is None:
                time.sleep(0.02)
                continue

            res = self.step(rgb, depth_mm)

            if not headless:
                hud = annotate_mission_frame(
                    rgb,
                    res["percept"],
                    res["state"],
                    res["target_id"],
                    res["global_pos"],
                    res["local_pos"],
                    self.config,
                )
                cv2.imshow("AeroTHON 2026 — Mission Control HUD", hud)
                k = cv2.waitKey(1) & 0xFF
                if k == ord("q"):
                    log.warning("[OPERATOR ABORT] 'q' pressed -> Triggering RTL")
                    self.fc.return_to_launch()
                    break
                elif k == ord("e"):
                    log.warning("[EMERGENCY] 'e' pressed -> Immediate Land")
                    self.fc.land()
                    break

            time.sleep(0.02)

        if not headless:
            cv2.destroyAllWindows()
