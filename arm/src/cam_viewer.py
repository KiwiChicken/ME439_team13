#! /usr/bin/env python3

import cv2


CV_CAP_PROP_FRAME_WIDTH = 3
CV_CAP_PROP_FRAME_HEIGHT = 4
CV_CAP_PROP_FPS = 5

cap = cv2.VideoCapture(0)
cap.set(CV_CAP_PROP_FRAME_WIDTH,900)
cap.set(CV_CAP_PROP_FRAME_HEIGHT,600)
cap.set(CV_CAP_PROP_FPS,3.0)

print('\n\nPress ''q'' in the display window to quit.\n\n')

def main(): 
    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
    
    #    # Our operations on the frame come here
    #    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #
    #    # Display the resulting frame
    #    cv2.imshow('frame',gray)
    #    if cv2.waitKey(1) & 0xFF == ord('q'):
    #        break
    
        # Display the resulting frame
        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    try: 
        main()
    except: 
        pass
        
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()