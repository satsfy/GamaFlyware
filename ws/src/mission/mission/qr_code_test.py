import cv2
import numpy as np

# Initialize the QR Code detector
qr_detector = cv2.QRCodeDetector()

# Open the default webcam (usually the first webcam)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Could not open webcam")
    exit()

while True:
    # Read frame from the webcam
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Detect multiple QR codes in the frame
    retval, decoded_info, points, _ = qr_detector.detectAndDecodeMulti(frame)

    if points is not None:
        for bbox, data in zip(points, decoded_info):
            print({"bbox": bbox, "data": data})
            # Convert bbox to integer type
            bbox = bbox.astype(int)

            # Number of points in bbox (should be 4 for a QR code)
            n = len(bbox)
            for i in range(n):
                # Extract start and end point for the line
                start_point = tuple(bbox[i])
                end_point = tuple(bbox[(i + 1) % n])

                # Draw the bounding box
                cv2.line(frame, start_point, end_point, (255, 0, 0), 2)

            # If data is decoded, print it
            if data:
                print(f"Decoded Data: {data}")

    # Display the resulting frame
    cv2.imshow("QR Code Detector", frame)

    # Press 'q' to exit the video window
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# When everything done, release the capture and close windows
cap.release()
cv2.destroyAllWindows()
