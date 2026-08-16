import cv2
import numpy as np


def nothing(x):
    pass


# Initialize video file
cap = cv2.VideoCapture(0)

# Create a window for the sliders
cv2.namedWindow("Trackbars")

# OpenCV Hue range is 0-179, Saturation and Value are 0-255
# Initial values set to a broad starting range for Green
cv2.createTrackbar("Lower H", "Trackbars", 35, 179, nothing)
cv2.createTrackbar("Lower S", "Trackbars", 50, 255, nothing)
cv2.createTrackbar("Lower V", "Trackbars", 50, 255, nothing)

cv2.createTrackbar("Upper H", "Trackbars", 85, 179, nothing)
cv2.createTrackbar("Upper S", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("Upper V", "Trackbars", 255, 255, nothing)


def detect_largest_banner(clean_mask, min_area=800, min_aspect_ratio=1.5, max_aspect_ratio=4.0):
    contours, _ = cv2.findContours(clean_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    largest_area = 0
    best_contour = None
    best_bbox = None

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue

        # Rotated rect -> used ONLY to validate true aspect ratio
        rot_rect = cv2.minAreaRect(cnt)
        (rw, rh) = rot_rect[1]
        if rw == 0 or rh == 0:
            continue

        rotated_aspect_ratio = max(rw, rh) / min(rw, rh)
        if not (min_aspect_ratio <= rotated_aspect_ratio <= max_aspect_ratio):
            continue

        # Axis-aligned rect -> used for the actual returned bbox (drawing/offset)
        x, y, w, h = cv2.boundingRect(cnt)

        if area > largest_area:
            largest_area = area
            best_contour = cnt
            best_bbox = (x, y, w, h)

    return best_contour, best_bbox


def get_banner_target(clean_mask):
    best_contour, best_bbox = detect_largest_banner(clean_mask)

    if best_bbox is None:
        return False, None, None, None

    x, y, w, h = best_bbox
    cx = x + w // 2
    cy = y + h // 2

    return True, cx, cy, best_bbox


while True:
    ret, frame = cap.read()
    if not ret:
        # Loop video if it ends
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        continue

    # 1. Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 2. Get current position of all 6 trackbars
    l_h = cv2.getTrackbarPos("Lower H", "Trackbars")
    l_s = cv2.getTrackbarPos("Lower S", "Trackbars")
    l_v = cv2.getTrackbarPos("Lower V", "Trackbars")

    u_h = cv2.getTrackbarPos("Upper H", "Trackbars")
    u_s = cv2.getTrackbarPos("Upper S", "Trackbars")
    u_v = cv2.getTrackbarPos("Upper V", "Trackbars")

    # 3. Create lower and upper bound numpy arrays
    lower_bound = np.array([l_h, l_s, l_v])
    upper_bound = np.array([u_h, u_s, u_v])

    # 4. Create the binary mask
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    frame = cv2.resize(frame, (800, 600))
    mask = cv2.resize(mask, (800, 600))

    # CLEANING Noise
    kernel = np.ones((5, 5), np.uint8)
    opened_mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    clean_mask = cv2.morphologyEx(opened_mask, cv2.MORPH_CLOSE, kernel)

    # Calculate frame center for alignment
    frame_h, frame_w = frame.shape[:2]
    frame_cx, frame_cy = frame_w // 2, frame_h // 2

    # Draw a crosshair for the frame's true center (optional, but helpful for debugging)
    cv2.drawMarker(frame, (frame_cx, frame_cy), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)

    # Use the normalized wrapper to get target data
    found, cx, cy, best_bbox = get_banner_target(clean_mask)

    if found:
        x, y, w, h = best_bbox

        # Draw bounding box
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Draw centroid of the banner
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

        # Compute alignment offset
        offset_x = cx - frame_cx
        offset_y = cy - frame_cy

        # Overlay visual feedback
        text = f"Offset: X={offset_x}, Y={offset_y}"
        cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # Draw a line from frame center to banner center to visualize the error
        cv2.line(frame, (frame_cx, frame_cy), (cx, cy), (0, 165, 255), 2)

    # Display original frame and filtered mask side-by-side
    cv2.imshow("Original Frame", frame)
    cv2.imshow("Filtered Mask", clean_mask)

    if cv2.waitKey(30) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


