import cv2
import time
import numpy as np
import handtrackingmodule as htm
import math
import pyautogui
from comtypes import CLSCTX_ALL
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume


wcam, hcam = 640, 480
frameR = 80
smoothening = 3

detector = htm.handDetector(detectionCon=0.7, maxHands=1)


devices = AudioUtilities.GetSpeakers()
interface = devices.Activate(
    IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
volume = interface.QueryInterface(IAudioEndpointVolume)
#volume.GetMute()
#volume.GetMasterVolumeLevel()
volrange = volume.GetVolumeRange()


minvol = volrange[0]
maxvol = volrange[1]
vol = 0
volbar = 400
volper = 0
area = 0

cap = cv2.VideoCapture(0)
cap.set(1, wcam)
cap.set(1, hcam)

wscr , hscr = pyautogui.size()
#print(wscr, hscr)

ptime = 0
ctime = 0
plocx, plocy = 0, 0
clocx, clocy= 0, 0

while True:
    success, img = cap.read()

    # find hand
    img = detector.findHands(img)
    lmlist, bbox = detector.findPosition(img, draw= True)
    if len(lmlist) != 0:

        #filter based on size
        area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])
        print(area)
        if 10000 < area < 100000:
            #print("yes")
            # find distance bw index and thumb
            length, img, lineinfo = detector.findDistance(4,8,img)

            # convert volume


            volbar = np.interp(length, [50, 300], [400, 150])
            volper = np.interp(length, [50, 300], [0, 100])


            # reduce resolution
            smoothness = 10
            volper = smoothness * round(volper/smoothness)

            # check fingers up
            fingers = detector.fingersup()
            #print(fingers)

            #pinky finger up
            if not fingers[4]:
                volume.SetMasterVolumeLevelScalar(volper / 100, None)
                cv2.circle(img, (lineinfo[4], lineinfo[5]), 10, (255, 255, 255), cv2.FILLED)




            if length < 50:
                cv2.circle(img, (lineinfo[4], lineinfo[5]), 10, (255, 255, 0), cv2.FILLED)
    #mouse pointer

    if len(lmlist) != 0:
        x1, y1 = lmlist[12][1:]
        x2, y2 = lmlist[16][1:]

        #finger up check
        fingers = detector.fingersup()
        #print(fingers)

        if fingers[2] == 1 and fingers[3] == 0:
            cv2.rectangle(img, (frameR, frameR), (wcam - frameR, hcam - frameR), (255, 0, 255, 2))
            x3 = np.interp(x1, (frameR, wcam- frameR), (0, wscr))
            y3 = np.interp(y1, (frameR, hcam- frameR), (0, hscr))
            #smoothen values
            clocx = plocx + (x3 - plocx) / smoothening
            clocy = plocy + (y3 - plocy) / smoothening

            pyautogui.moveTo(wscr - clocx, clocy)
            cv2.circle(img, (x1, y1), 20, (0,255,255),cv2.FILLED)
            plocx, plocy = clocx, clocy

        if fingers[2] == 1 and fingers[3] == 1:
            length, img, _ = detector.findDistance(12,16, img)
            print(length)
            if length < 40:
                cv2.circle(img, (x1,y1), 15, (0,255,0), cv2.FILLED)
                pyautogui.click()

        if fingers[2] == 1 and fingers[1] == 0:
            pyautogui.rightClick()


    cv2.rectangle(img, (50,150), (85, 400), (0,255,0), 3)
    cv2.rectangle(img, (50, int(volbar)), (85, 400), (0, 255, 0), cv2.FILLED)
    cv2.putText(img, f'{int(volper)}%', (30, 450), cv2.FONT_ITALIC, 2, (255, 255, 255), 3)
    cvol = int(volume.GetMasterVolumeLevelScalar()*100)
    cv2.putText(img, f'vol set:{int(cvol)}', (300, 50), cv2.FONT_ITALIC, 2, (0, 0, 255), 3)


    ctime = time.time()
    fps = 1/(ctime-ptime)
    ptime = ctime
    cv2.putText(img, f'FPS: {int(fps)}', (10, 70), cv2.FONT_ITALIC, 2, (255, 0, 255), 3)
    cv2.imshow("Img", img)
    cv2.waitKey(1)
