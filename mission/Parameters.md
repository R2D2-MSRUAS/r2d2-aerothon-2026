# SAEINDIA AeroTHON 2026 — Track 1: Rotorcraft Challenge
## Mission 2: SkyScan Autonomous Rapid Delivery
### Complete Flight, Vision, Hardware & Autopilot Parameters Reference
**Team:** R2D2
**Platform:** NVIDIA Jetson TX2 NX + Pixhawk 6X Mini + Luxonis OAK-D Lite

---

# 1. Flight Altitude & Trajectory Parameters

| Parameter Name | Value (Metric / Imperial) | Description | Rulebook / Engineering Rationale |
| :--- | :---: | :--- | :--- |
| `TAKEOFF_ALT_M` | `5.0 m` | Initial vertical climb altitude upon arming | Clear ground ground-effect turbulence; position for Start QR scan |
| `CORRIDOR_ALT_M` | `3.048 m` (`10.0 ft`) | Forward corridor traversal altitude | Strict adherence to Track 1 corridor altitude rule |
| `DELIVERY_SEARCH_ALT_M` | `10.0 m` | Delivery arena QR code search altitude | Provides wide field-of-view (FOV) for camera ground coverage |
| `PAYLOAD_DROP_ALT_M` | `5.0 m` | Visual servoing & precision winch drop altitude | Minimizes wind drift of payload while maintaining propeller clearance |
| `RETURN_CRUISE_ALT_M` | `10.0 m` | Climb altitude after payload drop & 180° rotation | Safety separation from delivery zone ground obstacles |
| `RETURN_CORRIDOR_ALT_M`| `3.048 m` (`10.0 ft`) | Backward corridor traversal altitude | Reverse passage through 3.5m corridor |
| `ALTITUDE_TOLERANCE_M` | `0.30 m` | Vertical convergence deadband | Acceptable error margin before state transition triggers |
| `POSITION_TOLERANCE_M` | `0.35 m` | Horizontal waypoint convergence radius | EKF3 position convergence threshold |
| `HEADING_TOLERANCE_DEG`| `5.0°` | Yaw orientation alignment tolerance | Heading error threshold for waypoint alignment |

---

# 2. Flight Velocities & Dynamic Limits

| Parameter Name | Value | Unit | Description & Controller Usage |
| :--- | :---: | :---: | :--- |
| `CRUISE_SPEED_MPS` | `1.80` | m/s | Open arena transit speed |
| `CORRIDOR_SPEED_MPS`| `1.50` | m/s | Controlled corridor passage speed |
| `SLOW_SPEED_MPS` | `0.80` | m/s | Precision approach and lateral alignment speed |
| `MAX_SPEED_MPS` | `2.50` | m/s | Hard software clamp on all body velocity commands $(v_x, v_y, v_z)$ |
| `YAW_RATE_DPS` | `25.0` | deg/s | Controlled yaw angular rate during 180° turns and search sweeps |
| `LATERAL_KP` | `0.45` | — | Proportional gain for corridor wall centering: $v_y = K_p \times y_{\text{err}}$ |
| `SERVOING_KP_X` | `0.0025` | — | Visual servoing gain: $v_x = \text{clamp}(-K_{px} \cdot e_y, -0.25, 0.25)$ |
| `SERVOING_KP_Y` | `0.0025` | — | Visual servoing gain: $v_y = \text{clamp}(K_{py} \cdot e_x, -0.25, 0.25)$ |

---

# 3. Obstacle Avoidance & Stereo Depth Thresholds

| Parameter Name | Value | Unit | Action State Triggered | Controller Reaction |
| :--- | :---: | :---: | :---: | :--- |
| `OBS_SAFE_DIST_M` | `> 2.00` | m | `CONTINUE` | Fly at nominal cruise speed (`1.50 m/s` in corridor) |
| `OBS_SLOW_DIST_M` | `1.20 – 2.00` | m | `SLOW` | Reduce forward speed by 50% (`0.75 m/s`); steer toward open column |
| `OBS_CRITICAL_STOP_M`| `< 0.80` | m | `STOP` | Immediate zero-velocity halt; trigger `AVOID_OBSTACLES` lateral escape |
| `CORRIDOR_WIDTH_M` | `3.50` | m | Arena Dimension | Nominal physical corridor width |
| `CORRIDOR_LENGTH_EST_M`| `15.00` | m | Arena Dimension | Estimated corridor flight axis distance |
| `DEPTH_PERCENTILE` | `50.0` | % | Median Filter | Extracts 50th percentile of valid non-zero depth pixels inside bounding box |

---

# 4. Safety Timeouts & Loop Bounds

| Timeout Parameter | Value | Action on Expiration |
| :--- | :---: | :--- |
| `ARM_TIMEOUT_S` | `10.0 s` | Abort launch sequence and disarm |
| `TAKEOFF_TIMEOUT_S` | `25.0 s` | Abort climb and command immediate safe landing |
| `ALTITUDE_CHANGE_TIMEOUT_S`| `18.0 s` | Proceed with flight at current safe altitude |
| `LANDING_TIMEOUT_S` | `30.0 s` | Cut throttle upon ground contact verification |
| `FC_CONNECTION_TIMEOUT_S` | `30.0 s` | Fall back to safe `MockFlightController` |
| `QR_SCAN_TIMEOUT_S` | `10.0 s` | Initiate 360° yaw search sweep pattern |
| `MAX_MISSION_TIME_S` | `1200.0 s` (`20 min`) | Competition time limit; trigger automatic **Return-to-Launch (RTL)** |

---

# 5. Winch & Payload Drop Mechanism

| Parameter Name | Value | Description |
| :--- | :---: | :--- |
| `WINCH_SERVO_CHANNEL` | `9` | Pixhawk AUX 1 / Channel 9 output pin |
| `WINCH_LOWER_PWM` | `1700 µs` | PWM signal to unspool cable from 5m down to ground level |
| `WINCH_NEUTRAL_PWM` | `1500 µs` | Stop / hold brake position |
| `WINCH_RELEASE_PWM` | `1900 µs` | Pulse signal triggering mechanical release claw |
| `WINCH_RAISE_PWM` | `1300 µs` | PWM signal to spool cable back into undercarriage |
| `WINCH_LOWER_TIME_S` | `4.5 s` | Time required to lower cable 5.0m |
| `WINCH_HOLD_TIME_S` | `1.5 s` | Payload ground settling duration (slack verification) |
| `WINCH_REEL_TIME_S` | `4.5 s` | Cable retraction duration |

---

# 6. Mass, Inertia & Center of Gravity (CoG) Compensation

| Parameter Name | Value | Unit | Mathematical Definition / Usage |
| :--- | :---: | :---: | :--- |
| `DRONE_EMPTY_MASS_KG` | `1.850` | kg | Dry weight of UAS without payload ($< 2.0\text{ kg}$ MTOW limit) |
| `PAYLOAD_MASS_KG` | `0.100` | kg | Specified delivery payload weight ($100\text{ g}$) |
| `PAYLOAD_COG_OFFSET_X_M`| `0.120` | m | Longitudinal forward offset distance from frame center to winch ($12\text{ cm}$) |
| **Calculated CoG Shift** | **`6.49`** | **mm** | $\Delta X_{\text{CoG}} = \frac{m_{\text{payload}} \times d_{\text{offset}}}{M_{\text{empty}}} = \frac{0.100 \times 0.120}{1.850} \approx 0.006486\text{ m}$ |
| **Pitch Bias Feed-Forward**| **`+0.005`** | **m/s** | Forward pitch velocity bias injected for $1.5\text{s}$ post-release |

---

# 7. Computer Vision & Detection Parameters

### A. General & Preprocessing
* **Resolution**: `640 x 480` pixels
* **Color Preprocessing**: LAB Color Space $\rightarrow$ Contrast Limited Adaptive Histogram Equalization (CLAHE, `clipLimit=2.0`, `tileGridSize=(8, 8)`)
* **People Shape Cycling Order**: `Circle` $\rightarrow$ `Ellipse` $\rightarrow$ `Hexagon` $\rightarrow$ `Rounded Box`

### B. AeroTHON Green Banner (Corridor Entrance)
* **HSV Hue Range**: $35 \le H \le 85$
* **HSV Saturation Range**: $75 \le S \le 255$
* **HSV Value Range**: $50 \le V \le 255$
* **Minimum Contour Area**: `2500 px`
* **Aspect Ratio Threshold**: $\text{Width} / \text{Height} \ge 1.30$

### C. Red No-Fly Zones (Restricted Drop Areas)
* **HSV Lower Range**: $0 \le H \le 10$, $S \ge 130$, $V \ge 80$
* **HSV Upper Range**: $170 \le H \le 180$, $S \ge 130$, $V \ge 80$
* **Minimum Contour Area**: `500 px`
* **Morphological Kernel**: $9 \times 9$ ellipse / close + dilate

### D. QR Code Detection & Validation
* **Confirmation Consecutive Frames**: `3 frames` (`QR_CONFIRM_COUNT`)
* **Detection Stages**:
  1. Full-resolution grayscale `pyzbar`
  2. $2.5\times$ bicubic upscaled grayscale `pyzbar`
  3. Adaptive Gaussian thresholding ($31 \times 31$, $C=4$)
  4. OpenCV multi-scale `cv2.QRCodeDetector.detectAndDecodeMulti()` fallback
* **Red Zone Filter**: Target QR rejected if bounding center falls inside any detected Red Zone contour.

### E. Module Implementation Reference

This subsection documents the two CV components (banner detection and QR detection/selection) referenced above at the function level.

#### E.1 Banner Detection — HSV Color Filtering & Contours

**Pipeline:**

```text
Camera Frame → BGR→HSV → HSV Color Filtering → Binary Mask →
Noise Removal → Contour Detection → Area + Aspect Ratio Filtering →
Largest Valid Banner
```

**Function:**

```python
detect_largest_banner(
    clean_mask,
    min_area=800,
    min_aspect_ratio=1.5,
    max_aspect_ratio=4.0
)
```

| Parameter | Description | Default |
|---|---|---:|
| `clean_mask` | Cleaned binary mask containing the detected color | Required |
| `min_area` | Minimum contour area to be considered | `800` |
| `min_aspect_ratio` | Minimum allowed width/height ratio | `1.5` |
| `max_aspect_ratio` | Maximum allowed width/height ratio | `4.0` |

> Note: The competition-tuned values used at runtime are in **7.B** above (`min_area=2500`, `aspect_ratio ≥ 1.30`) — the defaults here are the module's generic fallback values.

HSV range is tuned live via six trackbars (Lower/Upper H, S, V) feeding `cv2.inRange()`. Noise removal uses `MORPH_OPEN` + `MORPH_CLOSE` with a `5×5` kernel. Contours are filtered by minimum area and aspect ratio (`width / height`); the largest valid contour is selected.

**Returns:** `(best_contour, best_bbox)`
- `best_contour` — OpenCV contour of the largest valid banner, or `None`
- `best_bbox` — `(x, y, w, h)` bounding box, or `None`

**Example:**
```python
best_contour, best_bbox = detect_largest_banner(clean_mask)
if best_bbox:
    print(best_bbox)   # e.g. (245, 180, 320, 100)
```

#### E.2 QR Code Detection

**Architecture:**

```text
Frame → Perception → Detect & Decode QR Codes → List of Valid QR Targets →
Cognition → Select Required QR
```

**Function:**

```python
get_qr_targets(frame)
```

| Parameter | Description |
|---|---|
| `frame` | OpenCV image/frame in which QR codes should be detected |

Uses `cv2.QRCodeDetector()` with `detectAndDecodeMulti()` to detect multiple QR codes. Entries with no decodable data (`if not data: continue`) are discarded — only successfully decoded QR codes are returned.

**Returns:** `(found_boolean, targets)`
- `found_boolean` — `True` if at least one decodable QR code found, else `False`
- `targets` — list of dicts, one per decoded QR:
```python
{
    "data": data,      # decoded string
    "cx": cx,           # center X (mean of corner X coords)
    "cy": cy,           # center Y (mean of corner Y coords)
    "corners": pts       # four corner coordinates
}
```

**Example:**
```python
(
    True,
    [{"data": "TARGET_01", "cx": 420, "cy": 250, "corners": ...}]
)
```

#### E.3 Selecting a Specific QR Code

**Function:**

```python
select_target_qr(targets, target_code)
```

| Parameter | Description |
|---|---|
| `targets` | List returned by `get_qr_targets()` |
| `target_code` | QR data/string that the system is looking for |

Compares `target["data"]` (whitespace-stripped) against `target_code`. Returns the matching target dict, or `None` if not present.

#### E.4 Typical Usage

```python
found, targets = get_qr_targets(frame)

if found:
    target = select_target_qr(targets, "TARGET_01")
    if target:
        print("Target found!")
        print("Center:", target["cx"], target["cy"])
        print("QR Data:", target["data"])
```

#### E.5 Summary of Outputs

| Module | Function | Input | Output |
|---|---|---|---|
| Banner Detection | `detect_largest_banner()` | Clean binary mask | `best_contour`, `best_bbox` |
| QR Detection | `get_qr_targets()` | Image/frame | `found`, `targets` |
| QR Selection | `select_target_qr()` | QR targets + target code | Matching target or `None` |

**Overall data flow:** the banner module provides the location and size of the detected banner; the QR module provides the identity and location of a specific QR target. Both outputs feed the higher-level navigation/decision system.

---

# 8. Autopilot & Pixhawk ArduPilot Parameters

Set these parameters on the Pixhawk 6X Mini via **Mission Planner** or **QGroundControl**:

```ini
# Telemetry 2 MAVLink Connection
SERIAL2_PROTOCOL = 2        # MAVLink 2 Protocol
SERIAL2_BAUD     = 921      # 921600 Baud Rate for low-latency commands

# AUX 1 / Pin 9 Winch Servo Configuration
SERVO9_FUNCTION  = 0        # RC Passthrough / Direct Companion Computer MAVLink Control
SERVO9_MIN       = 1000     # Minimum PWM (µs)
SERVO9_MAX       = 2000     # Maximum PWM (µs)
SERVO9_TRIM      = 1500     # Neutral / Hold PWM (µs)

# Flight Dynamics & Safety Limits
PILOT_SPEED_UP   = 250      # Max climb speed (cm/s)
PILOT_SPEED_DN   = 150      # Max descent speed (cm/s)
ANGLE_MAX        = 3000     # Maximum bank angle (30.0°)
ARMING_CHECK     = 1        # Standard Pre-Arm Safety Checks
BRD_SAFETYENABLE = 0        # Hardware Safety Switch (0 = Disabled for automated bench testing)
```

---

# 9. MAVLink Protocol & Bitmask Specifications

### 1. Velocity Streaming Command
* **Message**: `SET_POSITION_TARGET_LOCAL_NED` (Message **#84**)
* **Coordinate Frame**: `MAV_FRAME_BODY_NED` (Body-fixed Forward, Right, Down)
* **Type Mask / Bitmask**: `0b0000111111000111` (`0x0FC7`)
  * Bit 0 (Pos X): `1` (Ignore)
  * Bit 1 (Pos Y): `1` (Ignore)
  * Bit 2 (Pos Z): `1` (Ignore)
  * Bit 3 (Vel X): `0` (**ENABLE** linear velocity $v_x$)
  * Bit 4 (Vel Y): `0` (**ENABLE** linear velocity $v_y$)
  * Bit 5 (Vel Z): `0` (**ENABLE** linear velocity $v_z$)
  * Bits 6-11 (Accel, Yaw, Yaw Rate): `1` (Ignore)

### 2. Controlled Heading Turn
* **Command**: `COMMAND_LONG` (#76) $\rightarrow$ `MAV_CMD_CONDITION_YAW` (#115)
* **Parameters**: `(Target_Angle, YAW_RATE_DPS=25.0, Direction=1, Relative=1)`

### 3. Winch Servo Actuation
* **Command**: `COMMAND_LONG` (#76) $\rightarrow$ `MAV_CMD_DO_SET_SERVO` (#183)
* **Parameters**: `(Servo_Num=9, PWM_Value=1300..1900)`

---

# 10. Summary Verification Commands

```powershell
# 1. Run all 23 unit tests verifying parameter math & state transitions
python -m unittest discover tests -v

# 2. Run full mission simulator with parameters
python sim_test.py

# 3. Run interactive mission HUD with laptop webcam
python drone_mission.py --camera webcam --sim

# 4. Deploy to real Pixhawk hardware with safety override flag
python drone_mission.py --connect /dev/ttyUSB0 --enable-real-flight
```
