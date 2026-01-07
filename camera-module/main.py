from ultralytics import YOLO
import cv2
import uuid
from datetime import datetime
import threading
import queue
import sys
import os
from influxdb_client_3 import InfluxDBClient3, InfluxDBError, Point, WritePrecision, WriteOptions, write_client_options

model = YOLO("/home/yeifermp/pet-monitor/yolo11n.pt")

pipeline = (
    "libcamerasrc ! "
    "video/x-raw,width=1296,height=972,framerate=30/1 ! "
    "videoconvert ! "
    "appsink"
)

capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
frame_count = 0
frame_queue = queue.Queue()
metadata_queue = queue.Queue()
points_queue = queue.Queue()
host = os.getenv('INFLUX_HOST')
token = os.getenv('INFLUX_TOKEN')
database = os.getenv('INFLUX_DATABASE')

if not capture.isOpened():
    print("We can't open the camera.")
    sys.exit(1)
    
def save_points_worker():
    while True:
        points = points_queue.get()
        with InfluxDBClient3(host=host,
                             token=token,
                             database=database,
                             org="Acme") as client:
            
            client.write(points, write_precision='s')
    
def metadata_worker():
    while True:
        metadata = metadata_queue.get()
        print ('Saving metadata...')
        
        points = [Point("home").tag("room", "living_room")
                  .field("box0", metadata["box0"])
                  .field("box1", metadata["box1"])
                  .field("box2", metadata["box2"])
                  .field("box3", metadata["box3"])
                  .field("date", metadata["date"])
                  .field("image_id", metadata["image_id"])]
        
        points_queue.put(points)
        

def inference_worker():
    while True:
        frame = frame_queue.get()
        print('Doing predictions...')        
        
        results = model.predict(frame)
        image_id = uuid.uuid1()

        for result in results:
            img = result.plot()

            for box, cls, conf in zip(result.boxes.xyxy, result.boxes.cls, result.boxes.conf):
                name = model.names[int(cls)]
                
                if name == 'cat':
                    print('A cat has been found in your home.')                    
                    cv2.imwrite(f"/home/yeifermp/pet-monitor/images/predictions/{image_id}.png", frame)
                    
                    metadata = {'box0': box[0],
                                'box1': box[1],
                                'box2': box[2],
                                'box3': box[3],
                                'date': f'{datetime.now()}',
                                'image_id': image_id}
                    
                    metadata_queue.put(metadata)
                    
                if name == 'person':
                    print('A human has been found in your home.')
                    cv2.imwrite(f"images/predictions/{image_id}.png", frame)
                        
def movement_worker():
    ret, prev_frame = capture.read()
    
    if not ret:
        print("We can't get a frame from the camera.")
        sys.exit(1)
    
    prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
    frame_count = 0
    
    while True:
        ret, frame = capture.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)    
        diff = cv2.absdiff(prev_gray, gray)
        _, thresh = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)      
        motion_pixels = (thresh > 0).sum()
        prev_gray = gray.copy()
        
        if (motion_pixels > 500):       
            print(f"Motion detected: {motion_pixels}")        
            points = [Point("home").tag("room", "living_room").field("movement", True).field("date", f'{datetime.now()}').field("intensity", motion_pixels)]
            points_queue.put(points)
            frame_count += 1
            
            if frame_count % 5 == 0:
                frame_queue.put(frame)
                
        if cv2.waitKey(1) == ord('q'):
            sys.exit(0)
        
threading.Thread(target=inference_worker, daemon=True).start()
threading.Thread(target=metadata_worker, daemon=True).start()
threading.Thread(target=save_points_worker, daemon=True).start()

thread_movement = threading.Thread(target=movement_worker, daemon=True)
thread_movement.start()
thread_movement.join()

capture.release()
cv2.destroyAllWindows()