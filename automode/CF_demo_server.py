import rospy
from geometry_msgs.msg import Twist

import websockets
import asyncio
import json

import cv2, base64
from ultralytics import YOLO

from sensor_msgs.msg import Image

import numpy as np
from openni import openni2

import pyttsx3
import speech_recognition as sr
import nltk
from nltk.tokenize import sent_tokenize, word_tokenize
import os
import transformers

import time
from std_msgs.msg import String 

nlp = transformers.pipeline("conversational", model="microsoft/DialoGPT-medium")

os.environ["TOKENIZERS_PARALLELISM"] = "true"

openni2.initialize()
dev = openni2.Device.open_any()

color_stream = dev.create_color_stream()
color_stream.start()

depth_stream = dev.create_depth_stream()
depth_stream.start()

recognizer = sr.Recognizer()
mic = sr.Microphone(0)

speaker = pyttsx3.init()

port = 5000

print("Started server on port : ", port)

# max speed and turn
speed = 0.6
turn = 1

is_navigating = False
sound_time = None
sound_msg = rospy.Publisher("playsound", String, queue_size=10)

class ServerState():
    def __init__(self) -> None:
        rospy.init_node("serverState")

        # image frame
        self.frame = np.array([])

        # yolo detect params
        self.track = False
        self.track_id = True
        self.model = YOLO("yolov8n.pt")
        self.detect_result = {}
        self.detect_r = {}

        # follow obj
        self.follow = False
        self.follow_obj_id = -1
        self.follow_time = 0

        # moving
        self.move_cmd = Twist()
        self.control_speed = 0
        self.control_turn = 0
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)

        # auto mode
        self.auto = False
        self.isAsk = 0

        # demo
        self.direction = ""

    # whether yolo start to track obj or not
    def setTrack(self, state:bool):
        self.track = state
        self.follow = False

        if state == False:
            self.detect_result = {}
            self.detect_r = {}

    # yolo track object function    
    def trackObj(self, frame):
        results = self.model.track(frame, conf=0.3,classes=[0], iou=0.5, persist=True, tracker="bytetrack.yaml")
        res_plotted = results[0].plot()

        self.detect_result = results[0].tojson()

        current_time = time.time()

        if serverState.auto == True:
            if not is_navigating and self.follow_obj_id != -2:
                if (current_time - self.follow_time) >= 5:
                    find_follow_obj = False 

                    for result in results:
                        a = result.boxes.cpu().numpy()
                        if a.id == self.follow_obj_id:
                            self.follow_time = current_time
                            find_follow_obj = True
                    
                    if find_follow_obj == False:
                        for result in results:
                            a = result.boxes.cpu().numpy()
                            if a.is_track == True:
                                serverState.setFollowObj(a.id[0])
                                self.isAsk = 0
                                self.follow_time = current_time
                                break
                    
        return res_plotted
    
    # set Follow obj id  
    def setFollowObj(self, id):
        self.follow = True
        self.follow_obj_id = id

        depth_img = get_depth_img()

        x1 = -1
        y1 = -1
        x2 = -1
        y2 = -1

        results = json.loads(self.detect_result)

        for result in results:
            if("track_id" in result):
                if self.follow_obj_id == result["track_id"]:
                    x1 = result["box"]["x1"]
                    y1 = result["box"]["y1"]
                    x2 = result["box"]["x2"]
                    y2 = result["box"]["y2"]

        current_depth_img = depth_img[int(y1):int(y2),int(x1):int(x2)]


        self.depth_img = depth_img

        self.depth_img = depth_img[int(y1):int(y2),int(x1):int(x2)]

        self.depth = np.median(self.depth_img)

    # follow obj
    def followObj(self):
        depth_img = get_depth_img()

        x1 = -1
        y1 = -1
        x2 = -1
        y2 = -1

        results = json.loads(self.detect_result)

        for result in results:
            if("track_id" in result):
                if self.follow_obj_id == result["track_id"]:
                    x1 = result["box"]["x1"]
                    y1 = result["box"]["y1"]
                    x2 = result["box"]["x2"]
                    y2 = result["box"]["y2"]

        current_depth_img = depth_img[int(y1):int(y2),int(x1):int(x2)]
        current_depth = np.median(current_depth_img)

        depth_threshold = 1200

        spd_x = 0
        spd_y = 0

        if current_depth > (depth_threshold+3000):
            spd_x = 0.7
        elif current_depth > depth_threshold:
            spd_x = 0.3
        else:
            spd_x = 0

        if x1 == -1 or y1 == -1 or x2 == -1 or y2 == -1:
            print("center")
            if self.control_speed != 0 or self.control_turn != 0:
                spd_x = 0
                spd_y = 0
        else:
            obj_center = (x1+x2)/2

            dif_center = obj_center - 320

            if dif_center > 0:
                if dif_center < 20:
                    print("center")
                elif dif_center > 50:
                    spd_y=-0.4
                elif dif_center > 160:
                    spd_y=-0.6
                elif dif_center > 250:
                    spd_y=-1
                else:
                    spd_y=-((abs(dif_center)-20)/320)
            else:
                if abs(dif_center) < 20:
                    print("center")
                elif abs(dif_center) > 50:
                    spd_y = 0.4
                elif abs(dif_center) > 160:
                    spd_y= 0.6 
                elif abs(dif_center) > 250:
                    spd_y=1 
                else:
                    spd_y=(abs(dif_center)-20)/320

        print("Current depth: ", current_depth)
        if serverState.auto == True and current_depth < depth_threshold and current_depth > 500:
            if self.isAsk < 1:
                speaker.say("Can I help you?")
                speaker.runAndWait()
            self.isAsk += 1

            print(self.isAsk)

            if self.isAsk == 10:
                self.follow_obj_id = -2
                self.follow = False

        self.change_speed({
            "x":spd_x,
            "y":spd_y
        }) 

    def is_near_to_robot(self):
        depth_img = get_depth_img()
        
        results = json.loads(self.detect_result)
        for result in results:
            if("track_id" in result):
                x1 = result["box"]["x1"]
                y1 = result["box"]["y1"]
                x2 = result["box"]["x2"]
                y2 = result["box"]["y2"]

                current_depth_img = depth_img[int(y1):int(y2),int(x1):int(x2)]
                current_depth = np.median(current_depth_img)
                if current_depth < 2000:
                    return True

        return False

        

    # moving function
    def move(self):
        if True:
            twist = Twist()
            twist.linear.x = self.control_speed; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z =self.control_turn
            self.cmd_vel_pub.publish(twist)

    # change speed function
    def move_forward(self):
        self.direction = "forward"
    
    def move_backward(self):
        self.direction = "backward"
    
    def move_left(self):
        self.direction = "left"
    
    def move_right(self):
        self.direction = "right"

    def change_speed(self, spd_info):
        print(spd_info)
        x = spd_info["x"]
        y = spd_info["y"]

        if abs(y) < 0.2:
            y = 0
        
        if abs(x) < 0.2:
            x = 0 

        target_speed = speed * x
        target_turn = turn * y

        if x == 0 and y == 0:
            self.control_speed = 0
            self.control_turn = 0

        if target_speed > self.control_speed:
            self.control_speed = min( target_speed, self.control_speed + 0.02 )
        elif target_speed <  self.control_speed:
            self.control_speed = max( target_speed,  self.control_speed - 0.02 )
        else:
            self.control_speed = target_speed

        if target_turn > self.control_turn:
            self.control_turn = min( target_turn, self.control_turn + 0.1 )
        elif target_turn < self.control_turn:
            self.control_turn = max( target_turn, self.control_turn - 0.1 )
        else:
            self.control_turn = target_turn
    
    # set auto mode
    def setAutoMode(self, automode):
        self.auto = automode

        if automode == True:
            asyncio.create_task(self.autoMode())

    async def autoMode(self):
        global sound_msg, sound_time, is_navigating
        print("automode")

        self.track = True
        print("Scanning")

       # for not follow
        self.follow_obj_id = -2

        move_count = 0
        
        while self.auto: 
            if (self.direction == ""):
                move_count = 0
            else:
                if(move_count >= 50):
                    self.direction = ""
                    self.change_speed({
                        "x":0,
                        "y":0
                    })
                if self.direction == "forward":
                    self.change_speed({
                        "x":0.4*((50-move_count)/50),
                        "y":0
                    })
                elif self.direction == "backward":
                    self.change_speed({
                        "x":-0.4*((50-move_count)/50),
                        "y":0
                    })
                elif self.direction == "right":
                    self.change_speed({
                        "x":0,
                        "y":-0.5*((50-move_count)/50)
                    })
                elif self.direction == "left":
                    self.change_speed({
                        "x":0,
                        "y":0.5*((50-move_count)/50)
                    })
                move_count += 1

            if self.follow_obj_id == -1:
                self.change_speed({
                    "x":0,
                    "y":0.3
                })


            if is_navigating:
                near = self.is_near_to_robot()
                if near and time.perf_counter() - sound_time > 10:
                    sound_time = time.perf_counter()
                    sound_msg.publish('play')

            
            await asyncio.sleep(0.1)
            
        
        if self.auto == False:
            self.track = False
            self.follow = False
            self.follow_obj_id = -1
            self.change_speed({
                "x":0,
                "y":0
            })

            is_navigating = False


async def transmit(websocket, path):
    global serverState
    print("Client Connected !")      

    if path == '/control':
        print("/control connected")
        try :
            while True:
                message = await websocket.recv()
                info = json.loads(message)

                if "x" in info.keys():
                    if serverState.auto == False:
                        serverState.change_speed(info)

                if "track" in info.keys():
                    if info["track"] == -1:
                        serverState.setTrack(False)
                    elif info["track"] == 1:
                        serverState.setTrack(True)

                if "follow_obj_id" in info.keys():
                    serverState.setFollowObj(info["follow_obj_id"])
                
                if "auto_touring" in info.keys():
                    serverState.setAutoMode(info["auto_touring"])

        except websockets.connection.ConnectionClosed as e:
            print("Client Disconnected !")  

    elif path == '/move':
        print("/move connected")
        try :
            while True:
                serverState.move()
                await asyncio.sleep(0.1)
        except websockets.connection.ConnectionClosed as e:
            print("Client Disconnected !") 

    elif path == '/track':
        while True:
            if serverState.track:
                detect_result = json.dumps({"data":serverState.detect_result})
                await websocket.send(detect_result)
            await asyncio.sleep(1)
    else:
        print("Video streaming connected")
        
        while True:
            frame = color_stream.read_frame()

            frame_data = frame.get_buffer_as_uint8()
            
            dtype = np.dtype("uint8") # Hardcode to 8 bits...

            image_opencv = np.ndarray(shape=(480, 640,3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                dtype=dtype, buffer=frame_data)

            frame = cv2.cvtColor(image_opencv , cv2.COLOR_BGR2RGB)

            frame = np.flip(frame,axis=1)

            if serverState.track == True:
                frame = serverState.trackObj(frame)
            
            if serverState.follow == True:
                serverState.followObj()
            
            encoded = cv2.imencode('.jpg', frame)[1]

            data = str(base64.b64encode(encoded))
            data = data[2:len(data)-1]
            
            await websocket.send(data)
            await asyncio.sleep(0.05)

def get_depth_img():
    frame = depth_stream.read_frame()
    frame_data = frame.get_buffer_as_uint16()
    
    dtype = np.dtype("uint16") # Hardcode to 8 bits...

    image_opencv = np.ndarray(shape=(480, 640), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
        dtype=dtype, buffer=frame_data)

    # image_opencv = ((image_opencv / np.max(image_opencv)) * 255).astype(np.uint8)
  
    return image_opencv

def callback(recognizer, audio):
    global is_navigating, sound_time

    location_topic = rospy.Publisher("location", String, queue_size=5)
    try:
        text = recognizer.recognize_google(audio)
        print("Detected: ",text) #Output

        res = ""
        
        if any(i in text for i in ["thank","thanks"]):
            res = np.random.choice(["you're welcome!","anytime!","no problem!","cool!","I'm here if you need me!","mention not"])
        elif any(i in text for i in ["exit","close"]):
            res = np.random.choice(["Tata","Have a good day","Bye","Goodbye","Hope to meet soon","peace out!"])
        elif "hello" in text:
            res = "hello"
        elif "toilet" in text:
            res = "toilet task"
            location_topic.publish("toilet")
            is_navigating = True
            sound_time = time.perf_counter()
        elif "bk1" in text:
            res = "bk1 task"
            location_topic.publish("bk1")
            is_navigating = True
            sound_time = time.perf_counter()
        elif "office" in text:
            res = "office task"
            location_topic.publish("office")
            is_navigating = True
            sound_time = time.perf_counter()
        elif "corner" in text:
            res = "cube task"
            location_topic.publish("cube")
            is_navigating = True
            sound_time = time.perf_counter()
        elif "follow me" in text:
            if serverState.auto == True:
                serverState.follow_obj_id = -1
            res = 'follow me task'
            is_navigating = False
        elif "move forward" in text:
            if serverState.auto == True:
                serverState.move_forward()
            res = "move forward"
        elif "move backward" in text:
            if serverState.auto == True:
                serverState.move_backward()
            res = "move backward"
        elif "turn right" in text:
            if serverState.auto == True:
                serverState.move_right()
            res = "turn right"
        elif "turn left" in text:
            if serverState.auto == True:
                serverState.move_left()
            res = "turn left"
        elif text == "ERROR":
            res="Sorry, come again?"
        elif text == "auto mode":
            print("auto mode")
        elif text == 'can I help you':
            res = ""
        # else:
        #     chat = nlp(transformers.Conversation(text), pad_token_id=50256)
        #     res = str(chat)
        #     res = res[res.find("assistant:")+11:].strip()
        #     print(res)

        if res != "":
            speaker.say(res)
            speaker.runAndWait()
    except sr.UnknownValueError:
        pass


async def speech_handler():
    speaker.setProperty ('rate', 100)
    speaker.setProperty('voice',f"english")

    # speaker.say('Yes')
    speaker.runAndWait()

    with mic as source:
        recognizer.adjust_for_ambient_noise(source)

    stop_listening = recognizer.listen_in_background(mic, callback)

    while True:
        if serverState.auto == True:
            break    
        await asyncio.sleep(0.1)

if __name__ == '__main__':
    global serverState
        
    serverState = ServerState()
    # listener()

    start_server = websockets.serve(transmit, host="10.164.38.36", port=port)

    loop = asyncio.get_event_loop()
    
    loop.create_task(speech_handler())

    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()

