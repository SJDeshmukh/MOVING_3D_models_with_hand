import math
from math import atan2, degrees
import cv2
import numpy as np
import pyautogui as pa
import pygame as pg
from cvzone.HandTrackingModule import HandDetector
from PIL import Image
from matrix_function import *
from object_3d import *
from cam_c import *
from asyncio.windows_events import NULL
from math import atan2,degrees
import cv2

cam=cv2.VideoCapture(0)
class Camera:
    def __init__(self, render, position):
        self.render = render
        self.position = np.array([*position, 1.0])
        self.forward = np.array([0, 0, 1, 1])
        self.right = np.array([1, 0, 0, 1])
        self.left=np.array([0,1,1,0])
        self.up=np.array([0,1,0,1])
        self.h_fov = math.pi / 3
        self.v_fov = self.h_fov * (render.HEIGHT / render.WIDTH)
        self.near_plane = 0.1
        self.far_plane = 100
        self.moving_speed = 10
        self.rotation_speed = 0.09
        
        self.anglePitch = 0
        self.angleYaw = 0
        self.angleRoll = 0

    def control(self):
        try:
            self.p=self.control_c()

            #leftMOve
            if self.p==2:
                self.position -= self.right * self.moving_speed
            #rightMove
            elif self.p==3:
                self.position += self.right * self.moving_speed
            #MoveUp
            elif self.p==4:
                 self.camera_pitch(-self.rotation_speed)
            #MoveDown
            elif self.p==5:
                self.camera_pitch(self.rotation_speed)


            else:
                self.position += self.forward * self.p*10
        except:
            pass
            
            

    def control_c(self):
        n=0
   
        try:
            start_dis=0
            detector=HandDetector(detectionCon=0.8)
            while True:
                success,img=cam.read()
                img1=img
                try:
                    hand,img=detector.findHands(img)
    #                 if len(hand)==2:
                    try:
                        lmList1=hand[0]["lmList"] 
                    except:
                        pass
                    try:
                        lmList2=hand[1]["lmList"]
                    except:
                        pass
                    if detector.fingersUp(hand[0])==[1,1,0,0,0] and detector.fingersUp(hand[1])==[1,1,0,0,0]:

                        length,info,img=detector.findDistance(lmList1[8][0:2],lmList2[8][0:2],img)    
                        scale=int((length-170.32)//2)

                        a=[-1,-2,-3,-4,-5,-6,-7,1,2,3,4,5,6,7]
                        if scale in a:
                            n=0
                            cv2.imshow("Img",img)
                            return n
                        else:
                            n=1
                            cv2.imshow("Img",img)
                            return scale
                    elif detector.fingersUp(hand[0])==[1,1,1,1,1] and detector.fingersUp(hand[1])==[0,0,0,0,0]:
                        n=2
                        cv2.imshow("Img",img)
                        return n
                    elif detector.fingersUp(hand[1])==[1,1,1,1,1] and detector.fingersUp(hand[0])==[0,0,0,0,0]:
                        n=3
                        cv2.imshow("Img",img)
                        return n
                    elif detector.fingersUp(hand[0])==[0,1,0,0,0]: #and detector.fingersUp(hand[1])==[0,0,0,0,0]:
                        n=4
                        cv2.imshow("Img",img)
                        return n
                    elif detector.fingersUp(hand[1])==[0,1,0,0,0]: #and detector.fingersUp(hand[1])==[0,1,0,0,0]:
                        n=5
                        cv2.imshow("Img",img)
                        return n

                    else :
                        cv2.imshow("Img",img)
                except:
                    cv2.imshow("Img",img)
    #                 print(0)
                if cv2.waitKey(1)==27:
                    break
            cam.release()
            cv2.destroyAllWindows()
        except:
            pass
#     control_c()
# Camera.control_c()

    
    
    
    
    
    
    
    
    
    
    
    def camera_yaw(self, angle):
        self.angleYaw += angle

    def camera_pitch(self, angle):
        self.anglePitch += angle

    def axiiIdentity(self):
        self.forward = np.array([0, 0, 1, 1])
        self.up = np.array([0, 1, 0, 1])
        self.right = np.array([1, 0, 0, 1])

    def camera_update_axii(self):
        # rotate = rotate_y(self.angleYaw) @ rotate_x(self.anglePitch)
        rotate = rotate_x(self.anglePitch) @ rotate_y(self.angleYaw)  # this concatenation gives right visual
        self.axiiIdentity()
        self.forward = self.forward @ rotate
        self.right = self.right @ rotate
        self.up = self.up @ rotate

    def camera_matrix(self):
        self.camera_update_axii()
        return self.translate_matrix() @ self.rotate_matrix()

    def translate_matrix(self):
        x, y, z, w = self.position
        return np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [-x, -y, -z, 1]
        ])

    def rotate_matrix(self):
        rx, ry, rz, w = self.right
        fx, fy, fz, w = self.forward
        ux, uy, uz, w = self.up
        return np.array([
            [rx, ux, fx, 0],
            [ry, uy, fy, 0],
            [rz, uz, fz, 0],
            [0, 0, 0, 1]
        ])