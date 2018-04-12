import cv2
cam = cv2.VideoCapture(0)
cam.set(3,1920)
cam.set(4,1080)
cam.set(cv2.CAP_PROP_FPS,30)
w = cam.get(cv2.CAP_PROP_FRAME_WIDTH)
h = cam.get(cv2.CAP_PROP_FRAME_HEIGHT)
r = cam.get(cv2.CAP_PROP_FPS)
print(w,h,r)
while cam.isOpened():
    err,img = cam.read()
    cv2.imshow("lalala", img)
    k = cv2.waitKey(1) & 0xff
    if k==27:
        break