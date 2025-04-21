import cv2
import mediapipe as mp
import time
import handtrackingmodule as htm

ptime = 0
ctime = 0
cap = cv2.VideoCapture(0)
detector = htm.handDetector()
while True:
    success, img = cap.read()
    img = detector.findHands(img)
    lmlist = detector.findPosition(img)
    if len(lmlist) != 0:
        print(lmlist[0])

    ctime = time.time()
    fps = 1 / (ctime - ptime)
    ptime = ctime

    cv2.putText(img, str(int(fps)), (10, 70), cv2.FONT_ITALIC, 3, (255, 0, 255), 3)
    cv2.imshow("Image", img)
    cv2.waitKey(1)