"""R2D2 command & middleware layer — camera failover + MAVLink dispatch.

Typical flight wiring (on the Jetson):

    from middleware import (Config, CameraManager, MavlinkBridge, Commander,
                            OakDLiteSource, ArducamCSISource)

    cfg = Config()
    cams = CameraManager([OakDLiteSource(cfg.camera),
                          ArducamCSISource(cfg.camera)], cfg.camera)
    bridge = MavlinkBridge(cfg.link)
    cmd = Commander(bridge, cams, cfg)

    bridge.connect()
    cams.start()
    while mission_running:
        frame, source = cmd.read_frame()
        ...  # vision + FSM
        cmd.tick()
"""
from .config import Config, DEFAULT, LinkConfig, CameraConfig, FailsafeConfig
from .cameras import (CameraSource, OakDLiteSource, ArducamCSISource,
                      OpenCVSource)
from .camera_manager import CameraManager
from .commander import Commander

try:
    from .mavlink_bridge import MavlinkBridge, CommandResult
except RuntimeError:
    pass  # pymavlink missing — camera stack still usable

__all__ = [
    "Config", "DEFAULT", "LinkConfig", "CameraConfig", "FailsafeConfig",
    "CameraSource", "OakDLiteSource", "ArducamCSISource", "OpenCVSource",
    "CameraManager", "Commander", "MavlinkBridge", "CommandResult",
]
