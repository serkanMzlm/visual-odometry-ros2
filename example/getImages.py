import cv2

cap = cv2.VideoCapture(0)

num = 0

while cap.isOpened():
    success, img = cap.read()
    k = cv2.waitKey(5)

    if k == 27 or k == ord('q'):
        break
    elif k == ord('s'):
        cv2.imwrite('/home/serkan/source_code/visual-odometry-ros2/example/images/image' + str(num) + '.png', img)
        print("image saved!")
        num += 1

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)

    cv2.putText(img, f"Width: {width}, Height: {height}, FPS: {fps}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow("Image", img)

cap.release()
cv2.destroyAllWindows()
