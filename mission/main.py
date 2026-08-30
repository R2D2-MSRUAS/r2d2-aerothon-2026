"""
R2D2 Master Drone Main Execution Loop
AeroTHON 2026 Autonomous Systems Integration
"""

import logging
import threading
import time

from aerothon.mavlink.interface import MAVLinkInterface
from aerothon.mission.fsm import MissionFSM, R2D2MissionState
from aerothon.navigation.velocity_generator import VelocityGenerator
from aerothon.payload.winch_control import WinchController
from aerothon.perception.manager import PerceptionManager, PerceptionResult
from aerothon.slam.slam_manager import SLAMManager
from aerothon.vision.banner_detector import BannerDetector
from aerothon.vision.depth_analyzer import DepthAnalyzer
from aerothon.vision.qr_detector import QRDetector
from aerothon.vision.redzone_detector import RedZoneDetector

# Logging Setup
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s][%(levelname)s][R2D2-Main] %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("R2D2-Main")


class R2D2SystemRunner:
    """
    Orchestrates the 4 concurrent control loops specified in the R2D2 Architecture:
      1. Vision Loop
      2. SLAM / Pose Loop
      3. Navigation Loop (50-100ms)
      4. Mission FSM Loop
    """

    def __init__(self):
        # Subsystem Components
        self.perception_mgr = PerceptionManager()
        self.slam_mgr = SLAMManager()
        self.fsm = MissionFSM()
        self.mavlink = MAVLinkInterface()
        self.vel_gen = VelocityGenerator()
        self.winch = WinchController(self.mavlink)

        # Vision Detectors
        self.banner_det = BannerDetector()
        self.qr_det = QRDetector()
        self.red_det = RedZoneDetector()
        self.depth_ana = DepthAnalyzer()

        # Threading state
        self.running = False
        self._sim_x = 0.0

    def vision_loop(self):
        """Loop 1: Perception Processing Loop."""
        log.info("Vision Loop started.")
        while self.running:
            # Synthetic/Real vision processing step
            obstacles = {"left": 2.5, "forward": 5.0, "right": 2.5}
            self.perception_mgr.update(obstacles=obstacles)
            time.sleep(0.033)  # ~30 FPS

    def slam_loop(self):
        """Loop 2: SLAM / VIO Pose Estimation Loop."""
        log.info("SLAM Loop started.")
        while self.running:
            # Simulate forward displacement
            if self.fsm.state in [R2D2MissionState.ENTER_CORRIDOR, R2D2MissionState.FOLLOW_CORRIDOR]:
                self._sim_x += 0.15

            z_alt = 5.0 if self.fsm.state != R2D2MissionState.LAND else 0.0
            self.slam_mgr.process_frame(x=self._sim_x, y=0.0, z=z_alt, yaw=0.0)
            time.sleep(0.05)  # 20 Hz

    def navigation_and_fsm_loop(self):
        """Loop 3 & 4: Navigation (50-100ms) & FSM Loop."""
        log.info("Navigation & FSM Loop started.")
        while self.running:
            perception = self.perception_mgr.latest_result
            pose = self.slam_mgr.estimator.current_pose

            # Update FSM state
            current_state = self.fsm.update(perception, pose)

            # Navigation logic
            vx, vy, vz = self.vel_gen.generate_corridor_velocity(perception)
            self.mavlink.send_velocity(vx, vy, vz)

            # Winch action trigger
            if current_state == R2D2MissionState.DROP_PAYLOAD:
                self.winch.execute_full_drop_sequence()

            if current_state == R2D2MissionState.COMPLETE:
                log.info("Mission Complete! Stopping loops.")
                self.running = False

            time.sleep(0.08)  # ~12.5 Hz (within 50-100ms spec)

    def start(self):
        """Starts all system threads."""
        self.running = True
        log.info("Starting R2D2 Drone Autonomous Software System...")

        t_vision = threading.Thread(target=self.vision_loop, daemon=True)
        t_slam = threading.Thread(target=self.slam_loop, daemon=True)

        t_vision.start()
        t_slam.start()

        # Run Navigation & FSM on main thread
        self.navigation_and_fsm_loop()


if __name__ == "__main__":
    runner = R2D2SystemRunner()
    runner.start()
