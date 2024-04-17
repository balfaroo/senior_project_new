import cv2
import os


camera = 0
cap = cv2.VideoCapture(camera)
count = 0
print(os.getcwd())

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)


    # Display the resulting frame
    cv2.imshow('frame', frame)

    # Hit q to quit.
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    elif cv2.waitKey(1) & 0xFF  == ord('s'):
        filename = 'img_%d.jpg' % count
        count += 1
        cv2.imwrite(os.path.join(os.getcwd(), filename), frame)
        print('saved')

# Release the capture
cap.release()
cv2.destroyAllWindows()