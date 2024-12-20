#Detect Code
import cv2
# from matplotlib import pyplot as plt
import numpy as np
from deep_sort_realtime.deepsort_tracker import DeepSort
from ultralytics import YOLO

import math

#Class For Detecting Tomato
class DetectTomato:
    # Constructor (initializer) to define attributes
    def __init__(self, image, resizeMag=1):
        self.classNames = ["unripe", "ripe", "unknownCLS"]
        self.classColor = [(255,255,0),(0,255,255),(255,255,0)]
        self.resizeMagForText=resizeMag
        
        # Load the YOLO model
        self.model_tomato = YOLO("aeronbest.pt")
        self.confidenceThreshold=0.5
        
        self.tracker = DeepSort(max_iou_distance=0.7, max_age=30, n_init=3)
        # Initialize the needed set
        self.GetNewTargetTomato=True
        self.height, self.width, self.channels = image.shape
        self.TopTomatoTrack = None
        self.TopTomatoTrackY = self.height #(y最底下, y最大的地方)目標是找y最小的
        
        self.TargetTomatoTrack = None
        self.TargetTomatoTrackID = None

    #Public Function
    def changeGetNewTargetTomato(self,flag):
        self.GetNewTargetTomato=flag
        print("Now GetNewTargetTomato: "+str(self.GetNewTargetTomato))
        
    # Method to display car information
    def DetectTomato(self, image):
        results = self.model_tomato(image)
        detections = []
        
        for result in results:
            boxes = result.boxes.xyxy  # Bounding box coordinates (x_min, y_min, x_max, y_max)
            scores = result.boxes.conf  # Confidence scores
            classes = result.boxes.cls  # Class indices (e.g., 0 for tomato)
        
            # Loop through detections
            for box, score, cls in zip(boxes, scores, classes):
                x_min, y_min, x_max, y_max = box.tolist()
                label = f"Class {int(cls)}: {score:.2f}"
                width = x_max - x_min
                height = y_max - y_min
                if(score>self.confidenceThreshold): #如果confidenceScore夠大, 才加入track中
                    bbox = [x_min, y_min, width, height]  # Bounding box in [x1, y1, x2, y2]
                    detections.append((bbox, score.item(), int(cls.item())))
        tracked_objects = self.tracker.update_tracks(detections, frame=image)
        image = self.__ExtractTrackAndReturnTopAndTarget(tracked_objects, image)
        
        if (self.TopTomatoTrack is not None):
            image = self.__VisualizeTrackBoundingBox(self.TopTomatoTrack, image, 20, True, (0,100,100))
            print(f"The Top Tomato is{self.TopTomatoTrack.to_tlbr()}")
            
            if(self.GetNewTargetTomato==True): #如果現在是要找到新的Tomato的話, TargetTomato就是更新為TopTomato
                self.TargetTomatoTrack=self.TopTomatoTrack
                self.TargetTomatoTrackID=self.TargetTomatoTrack.track_id
                
                print("Get New Tomato")
                self.changeGetNewTargetTomato(False)  #拿到新Tomato之後才默認不找新tomato了 如果要找要再開 如果沒找到就繼續找
            if (self.TargetTomatoTrack is not None):
                image = self.__VisualizeTrackBoundingBox(self.TargetTomatoTrack, image, 5, True, (0,50,255))
                print(f"The Target Tomato is{self.TargetTomatoTrack.to_tlbr()}")
                x1_target, y1_target, x2_target, y2_target = map(int, self.TargetTomatoTrack.to_tlbr())
                TargetBox=[x1_target, y1_target, x2_target, y2_target]
                if (TargetBox is not None):
                    print("TargetBox: "+str(TargetBox))
                    return TargetBox, image #return a bounding box (x1,y1,x2, y2)
                else: 
                    print("TargetBox2: "+str(TargetBox))
                    return TargetBox, image
            else: 
                # self.TargetTomatoTrack = None
                return None, image #如果Target Tomato 突然消失, 但也沒有叫他重新拿新的tomato, 就回傳none
        else: 
            self.TargetTomatoTrack=None
            print("There are No unmargin tomato") #根本沒有內圈的tomato的話
            return None, image #就回傳None 表示沒有Tomato
            
            
    #Private Function
    def __VisualizeTrackBoundingBox(self,TomatoTrack, image, BoxThickness=20, colorFlag=False, color=(0,0,0)): #只會劃出top的tomato 就是一個bounding box而已
        bbox= TomatoTrack.to_tlbr()
        det_class_Tomato = TomatoTrack.det_class
        det_conf_Tomato = TomatoTrack.det_conf
        track_id_Tomato = TomatoTrack.track_id

        if track_id_Tomato is None:
            track_id_Tomato = 100  #
        if det_class_Tomato is None:
            det_class_Tomato = 2  # Default class name if none is found
        if det_conf_Tomato is None:
            det_conf_Tomato = 0.0  # Default confidence if none is found

        x1_Tomato,y1_Tomato,x2_Tomato,y2_Tomato = map(int, bbox)
        

        if (colorFlag==False): #用內定顏色
            color = self.classColor[det_class_Tomato]
        else: 
            color = color #用自訂顏色
        #Draw bounding box
        cv2.rectangle(image, (x1_Tomato, y1_Tomato), (x2_Tomato, y2_Tomato), color, math.ceil(BoxThickness*self.resizeMagForText))  # Draw red box (0,50,255)
        #劃出資訊
        label = f"ID: {track_id_Tomato} {self.classNames[det_class_Tomato]}"
        label2= f"CONF: {det_conf_Tomato:.2f}"
        # Get text size
        (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 1.0*self.resizeMagForText, math.ceil(2*self.resizeMagForText))
        (text_width2, text_height2), baseline2 = cv2.getTextSize(label2, cv2.FONT_HERSHEY_SIMPLEX, 1.0*self.resizeMagForText, math.ceil(2*self.resizeMagForText))
        # Draw background rectangle
        cv2.rectangle(image, (x1_Tomato, y1_Tomato - text_height-text_height2 - int(20*self.resizeMagForText)), (x1_Tomato + max(text_width, text_width2), y1_Tomato), color, -1)
        cv2.rectangle(image, (x1_Tomato, y1_Tomato - text_height - int(10*self.resizeMagForText)), (x1_Tomato + max(text_width, text_width2), y1_Tomato), color, -1)
        
        # Put label text
        cv2.putText(image, label, (x1_Tomato, y1_Tomato - int(40*self.resizeMagForText)), cv2.FONT_HERSHEY_SIMPLEX, 1.0*self.resizeMagForText, (0, 0, 0), math.ceil(2*self.resizeMagForText))
        cv2.putText(image, label2, (x1_Tomato, y1_Tomato - int(10*self.resizeMagForText)), cv2.FONT_HERSHEY_SIMPLEX, 1.0*self.resizeMagForText, (0, 0, 0), math.ceil(2*self.resizeMagForText))
        
        
        return image
        
    def __ExtractTrackAndReturnTopAndTarget(self,tracked_objects, image):
        for track in tracked_objects:
            if not track.is_confirmed():
                # print("no track")#############################################################
                continue  # Skip unconfirmed tracks
        
            # Extract track details
            track_id = track.track_id
            bbox = track.to_tlbr()  # Convert bbox to [top, left, bottom, right]
            # print(f"Track ID: {track_id}, BBox: {bbox}")######################################
            # Print all attributes and methods of the track object
            # print(dir(track.__dict__))
            # Draw the bounding box and track ID on the image
            x1, y1, x2, y2 = map(int, bbox)
            
            # cv2.putText(image, f"ID: {track_id}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
            # Add background for the label to improve readability
            # If det_class or det_conf is None, set a default value
            det_class = track.det_class
            det_conf = track.det_conf
            if track_id is None:
                # track.track_id = 100
                track_id = 100  #
            if det_class is None:
                # track.det_class = 2 
                det_class = 2  # Default class name if none is found
            if det_conf is None:
                # track.det_conf = 0.0 #<Debug>不知道為啥 救世會有det_conf=none的狀況, 所以就在這修理... 嗎 不行 這樣會爆走 再visualization修正就好
                det_conf = 0.0  # Default confidence if none is found
            
            if(x1>int(self.width/15) and x2<int(self.width-self.width/15)): #detect x not in the margin
                if(y1>int(self.height/15) and y2<int(self.height-self.height/15)): #detect y not in the margin
                    #Top Tomato
                    if(y1<self.TopTomatoTrackY) and self.classNames[det_class]=="ripe": #is ripe and is the toppest but not in the margin tomato
                        self.TopTomatoTrack = track
                        self.TopTomatoTrackY = y1 #(y最底下, y最大的地方)目標是找y最小的
        
            if(self.GetNewTargetTomato==False): #如果是要繼續trace前一個tomato ID的話那就把targetTomato在這裡看同個ID的那個tomato
                if(track_id==self.TargetTomatoTrackID): #如果ID一樣的話
                    self.TargetTomatoTrack=track #就存下來
            
            #Visualize the detected tomato
            image  = self.__VisualizeTrackBoundingBox(track, image, 10)
        return image
        # return self.TopTomatoTrack, self.GetNewTargetTomato


if __name__ == '__main__':
    #Load image
    video_path = "Database/tomato_video2.mp4"  # Path to your video
    cap = cv2.VideoCapture(video_path)

    resizeMag=1/3 
    #main
    ret, frame1 = cap.read()
    if not ret:
        print("End of video.")
    
    frame1_resized = cv2.resize(frame1, (int(frame1.shape[1]*resizeMag), int(frame1.shape[0]*resizeMag)))
    #initialize
    MyTomatoDetector = DetectTomato (frame1_resized, resizeMag) #給一個初始的圖片
    MyTomatoDetector.changeGetNewTargetTomato(True) 
    # TargetBox, image = MyTomatoDetector.DetectTomato(frame1_resized) #先得到初始的ID
    # MyTomatoDetector.changeGetNewTargetTomato(False) 

    while(True): 
        #main
        ret, frame1 = cap.read()
        if not ret:
            print("End of video.")
            break
        frame1_resized = cv2.resize(frame1, (int(frame1.shape[1]*resizeMag), int(frame1.shape[0]*resizeMag)))

        TargetBox, image = MyTomatoDetector.DetectTomato(frame1_resized) #先得到初始的ID

        if TargetBox is None:
            print("No tomato detected.")
        cv2.imshow('Tomato Image', image)
        cv2.waitKey(1)





