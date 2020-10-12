import cv2

num_cameras = [2,4]
cam_lst = []

for n in num_cameras:
# for n in range(num_cameras):
    print(n)
    cam = cv2.VideoCapture(n)
    if cam.isOpened():
        cam_lst.append(cam)

while True:

    for index, c in enumerate(cam_lst):
        ret, frame = c.read()
        cv2.imshow('camera %i.jpg' %(index + 1), frame)

    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        break

for cam in cam_lst:
    cam.release()
cv2.destroyAllWindows()
