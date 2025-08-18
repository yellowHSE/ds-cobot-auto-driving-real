import cv2

cap = cv2.VideoCapture(0)  # Change index if needed

while True:
    ret, frame = cap.read()
    if not ret:
        break

    cv2.imshow('Camera Frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

