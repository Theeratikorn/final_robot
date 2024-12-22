import cv2
import numpy as np

# Initialize a global list to store detected object attributes
object_attributes = []

def detect_objects(frame, lower_bound, upper_bound, color_name, color_index):
    global object_attributes

    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Threshold the image to isolate the color
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        # Ignore small objects
        if cv2.contourArea(contour) < 500:
            continue

        # Approximate the contour to reduce number of points
        epsilon = 0.04 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Get the centroid
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
        else:
            cx, cy = 0, 0

        # Draw the contour and the centroid
        cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)
        cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)

        # Display number of corners
        num_corners = len(approx)
        cv2.putText(frame, f'{color_name}: {num_corners}', (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Append object attributes to the global list
        object_attributes.append([color_index, num_corners, cx, cy])

    return frame

def nothing(x):
    pass

def process_frame(cap, colors, blur_level):
    global object_attributes

    ret, frame = cap.read()
    if not ret:
        return None

    if blur_level > 0:
        frame = cv2.GaussianBlur(frame, (2 * blur_level + 1, 2 * blur_level + 1), 0)

    object_attributes.clear()  # Clear previous frame's data

    for i, color in enumerate(colors):
        # Get current positions of the sliders for each color
        h_upper = cv2.getTrackbarPos(f'H Upper {color}', 'HSV Sliders')
        s_upper = cv2.getTrackbarPos(f'S Upper {color}', 'HSV Sliders')
        v_upper = cv2.getTrackbarPos(f'V Upper {color}', 'HSV Sliders')

        h_lower = cv2.getTrackbarPos(f'H Lower {color}', 'HSV Sliders')
        s_lower = cv2.getTrackbarPos(f'S Lower {color}', 'HSV Sliders')
        v_lower = cv2.getTrackbarPos(f'V Lower {color}', 'HSV Sliders')

        lower = np.array([h_lower, s_lower, v_lower])
        upper = np.array([h_upper, s_upper, v_upper])

        # Process the frame for the current color
        frame = detect_objects(frame, lower, upper, color, i + 1)

    return frame

# Mouse callback to display HSV values on right-click
def show_hsv(event, x, y, flags, param):
    if event == cv2.EVENT_RBUTTONDOWN:
        hsv_frame = cv2.cvtColor(param, cv2.COLOR_BGR2HSV)
        hsv_value = hsv_frame[y, x]
        h, s, v = hsv_value

        # Create a popup frame with color
        popup_frame = np.zeros((200, 400, 3), dtype=np.uint8)
        popup_frame[:] = cv2.cvtColor(np.uint8([[[h, s, v]]]), cv2.COLOR_HSV2BGR)

        # Add text with HSV values
        text = f"H: {h}, S: {s}, V: {v}"
        cv2.putText(popup_frame, text, (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # Display the popup frame
        cv2.imshow("Picked Color HSV", popup_frame)
def img_preocessing():
    # Create a window for sliders
    cv2.namedWindow('HSV Sliders', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('HSV Sliders', 400, 800)

    cv2.namedWindow('Blur Control', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Blur Control', 400, 100)

    # Create sliders for each color range
    colors = ['Color 1', 'Color 2', 'Color 3', 'Color 4']
    for color in colors:
        cv2.createTrackbar(f'H Upper {color}', 'HSV Sliders', 179, 179, nothing)
        cv2.createTrackbar(f'S Upper {color}', 'HSV Sliders', 255, 255, nothing)
        cv2.createTrackbar(f'V Upper {color}', 'HSV Sliders', 255, 255, nothing)

        cv2.createTrackbar(f'H Lower {color}', 'HSV Sliders', 0, 179, nothing)
        cv2.createTrackbar(f'S Lower {color}', 'HSV Sliders', 0, 255, nothing)
        cv2.createTrackbar(f'V Lower {color}', 'HSV Sliders', 0, 255, nothing)

    # Add slider for blur level in a separate window
    cv2.createTrackbar('Blur Level', 'Blur Control', 0, 20, nothing)

    # Initialize webcam
    cap = cv2.VideoCapture(2)
    cap.set(3, 640)  # Set width
    cap.set(4, 480)  # Set height

    def create_popup():
        popup = np.zeros((200, 400, 3), dtype=np.uint8)
        cv2.putText(popup, 'Press S to Save', (50, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(popup, 'Press Q to Stop', (50, 140), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.imshow('Options', popup)

    # Main loop
    cv2.namedWindow('Object Detection')
    create_popup()
    while True:
        blur_level = cv2.getTrackbarPos('Blur Level', 'Blur Control')
        frame = process_frame(cap, colors, blur_level)
        if frame is None:
            break

        # Set the mouse callback to display HSV values
        cv2.setMouseCallback('Object Detection', show_hsv, param=frame)

        # Display the result
        cv2.imshow('Object Detection', frame)

        # Check for key presses
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            if object_attributes is not None:
                cap.release()
                cv2.destroyAllWindows()
                return object_attributes
            break
        elif key == ord('s'):
            print("Saved attributes:", object_attributes)

    cap.release()
    cv2.destroyAllWindows()
