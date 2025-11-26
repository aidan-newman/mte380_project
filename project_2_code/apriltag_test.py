import cv2
from pupil_apriltags import Detector

detector = Detector(
    families="tag36h11",
    nthreads=1,
    quad_decimate=1.0,
    refine_edges=1
)

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # Use default camera

while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    tags = detector.detect(gray)

    for tag in tags:
        (x, y) = tag.center
        cv2.circle(frame, (int(x), int(y)), 5, (0, 255, 0), -1)
        cv2.putText(frame, str(tag.tag_id), (int(x)+10, int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    cv2.imshow("AprilTag Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()