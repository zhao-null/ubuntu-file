# coding: utf-8
import cv2
img = cv2.imread("/home/zcy/follow_car/src/simu_follower/scripts/camera.jpg")


# print img.shape

def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        xy = "%d,%d" % (x, y)
        print(xy)
        cv2.circle(img, (x, y), 3, (255, 0, 0), thickness=-1)
        cv2.putText(img, xy, (x, y), cv2.FONT_HERSHEY_PLAIN,
                    1.0, (0, 0, 0), thickness=1)
        cv2.imshow("image", img)


cv2.namedWindow("image")
cv2.setMouseCallback("image", on_EVENT_LBUTTONDOWN)
cv2.imshow("image", img)

while (True):
    try:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break
    except Exception:
        cv2.destroyAllWindows()
        break

cv2.waitKey(0)
cv2.destroyAllWindows()