import cv2
import pygame
import threading
import time

# sound list
WHISPERER = ["./src/ghost1.mp3",
             "./src/ghost2.mp3",
             "./src/ghost3.mp3",
             "./src/ghost5.mp3"]

MANIFESTATION = ["./src/creepylaugh.mp3",
                 "./src/demonhaunting.mp3",
                 "./src/hellsound.mp3",
                 "./src/piano.mp3",
                 "./src/poltergeist.mp3",
                 "./src/siren.mp3",
                 "./src/alarm.mp3",
                 "./src/security-alarm.mp3"]


def play_ghost(ghost):
    ghost.play(-1)


def play_valak(valak):
    valak.play(-1)


def stop_ghost(ghost):
    ghost.stop()


def valak_manifestation(frame):
    img = cv2.imread("./src/image.jpg")

    # if cannot find the image file
    if img is None:
        print("VALAK FAILED TO MANIFEST!")
        return

    # resize the image
    img = cv2.resize(img, (frame.shape[1], frame.shape[0]))

    # adjust brightness prepare to make valak blink
    bright_img = cv2.convertScaleAbs(img, alpha=3.0, beta=100)

    # display on the window
    while True:
        # cv2.imshow("Feed", img)
        # blink valak on the screen
        cv2.imshow("Feed", img)
        if cv2.waitKey(50) == ord('q'):
            break

        cv2.imshow("Feed", bright_img)
        if cv2.waitKey(50) == ord('q'):
            break

    cv2.destroyAllWindows()


def motion_detection(
        camera_idx,
        threshold_val=100,
        max_val=255,
        kernel_size=(5, 5),
        std_dev=0,
        counting=10,
        alarm_triggered=False,
        movement=False,
        sound_played=False
):
    cap = cv2.VideoCapture(camera_idx)

    # Camera cannot open error handling
    if not cap.isOpened():
        print("Cannot open the camera! Exiting program...")
        exit()

    # define sounds
    pygame.mixer.init()
    whispers = [pygame.mixer.Sound(whisper) for whisper in WHISPERER]
    manifestations = [pygame.mixer.Sound(manifest) for manifest in MANIFESTATION]

    # take two frames
    ret, frame1 = cap.read()
    ret, frame2 = cap.read()

    # define width and height frame
    height, width, _ = frame1.shape

    # Initialize local variables
    whisper_start = None
    countdown_start = None

    # handling cam
    try:
        while True:
            if not ret:
                break

            # count absolute difference between two frames
            diff = cv2.absdiff(frame1, frame2)

            # change BGR to GRAY
            gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)

            # blur with gaussian blur
            blur = cv2.GaussianBlur(gray, kernel_size, std_dev)

            # make threshold
            _, thresh = cv2.threshold(blur, threshold_val, max_val, cv2.THRESH_BINARY)

            # apply dilated image
            dilated = cv2.dilate(thresh, None, iterations=3)

            # find the contours
            contours, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # draw the contours
            for contour in contours:
                (x, y, w, h) = cv2.boundingRect(contour)

                # check whether the contour area is bigger than 700
                if cv2.contourArea(contour) > 900:
                    continue

                cv2.rectangle(img=frame1, pt1=(x, y), pt2=(x + w, y + h), color=(0, 255, 0), thickness=2)
                cv2.putText(frame1, "Movement Detected!", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

                if not movement:
                    movement = True

                    whisper_start = time.time()

            # Countdown before manifestation
            if whisper_start is not None:
                elapsed_time_whisper = time.time() - whisper_start
                remaining_time_whisper = max(0, counting - int(elapsed_time_whisper))

                # draw countdown on the window
                cv2.putText(frame1, f"{remaining_time_whisper}", (width // 2 - 50, height // 2 + 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 5, (0, 0, 255), 5)

                if remaining_time_whisper == 0:
                    whisper_start = None

                    # alarm goes off when there are motions detected
                    print("MOTION DETECTED! ALARM TRIGGERED!")  # debug print
                    alarm_triggered = True
                    if not sound_played:
                        sound_played = True

                        # Threading for whisperers
                        for sound in whispers:
                            threading.Thread(target=play_valak, args=(sound,)).start()

                        # starting countdown
                        countdown_start = time.time()

            # Countdown before manifestation
            if countdown_start is not None:
                elapsed_time = time.time() - countdown_start
                remaining_time = max(0, counting - int(elapsed_time))

                # draw countdown on the window
                cv2.putText(frame1, f"{remaining_time}", (width // 2 - 50, height // 2 + 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 5, (0, 0, 255), 5)

                if remaining_time == 0:
                    # stop whispering
                    for sound in whispers:
                        stop_ghost(sound)

                    # play manifestation audio
                    for valak in manifestations:
                        threading.Thread(target=play_ghost, args=(valak,)).start()

                    # Valak manifestation
                    valak_manifestation(frame1)

                    print("Sending email...")  # COMING SOON

                    countdown_start = None
                    movement = False
                    sound_played = False

            # window in full screen mode
            cv2.namedWindow("Feed", cv2.WND_PROP_FULLSCREEN)
            cv2.setWindowProperty("Feed", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

            cv2.imshow("Feed", frame1)

            # insert value from frame2 into frame 1
            frame1 = frame2

            # read new value from frame2
            ret, frame2 = cap.read()

            # close the camera gracefully
            if cv2.waitKey(10) == ord('q'):
                break

        # release the capture after everything is done
        cap.release()
        cv2.destroyAllWindows()

    except Exception as e:
        print(f"An error occurred: {e}")


if __name__ == "__main__":
    motion_detection(camera_idx=0)
