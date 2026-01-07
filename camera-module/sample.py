import cv2

pipeline = (
    "libcamerasrc ! "
    "video/x-raw,width=640,height=640,framerate=30/1 ! "
    "videoconvert ! "
    "appsink"
)

capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if not capture.isOpened():
    print("We can't open the camera.")

while True:
    ret, frame = capture.read()

    cv2.imshow("Camera", frame)

    if cv2.waitKey(1) == ord('q'):
        break

capture.release()
cv2.destroyAllWindows()