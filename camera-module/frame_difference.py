import cv2

pipeline = (
    "libcamerasrc ! "
    "video/x-raw,width=1296,height=972,framerate=30/1 ! "
    "videoconvert ! "
    "appsink"
)

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
ret, prev_frame = cap.read()
prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)

while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)    
    diff = cv2.absdiff(prev_gray, gray)
    _, thresh = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)    
    cv2.imshow("Motion", thresh)    
    motion_pixels = (thresh > 0).sum()
    
    if (motion_pixels > 500):
        print(f"Motion detected: {motion_pixels}")
    
    prev_gray = gray.copy()
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
cap.release()
cv2.destroyAllWindows()
    