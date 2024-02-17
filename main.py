import os
import time
from spot_controller import SpotController

ROBOT_IP = "10.0.0.3"#os.environ['ROBOT_IP']
SPOT_USERNAME = "admin"#os.environ['SPOT_USERNAME']
SPOT_PASSWORD = "2zqa8dgw7lor"#os.environ['SPOT_PASSWORD']
import cv2


def detect_object(image):
    # detect a qr code in the image and return how far it is from the center (depth perception)
    # return how many meters it is from the image center
    # if no qr code is found,
    # the paper is an a4 paper (210 x 297 mm) and the camera is 0.6 meter above the ground
    # this can be used to calculate the distance to the paper
    # if the paper is not found, 0, 0 is returned
    qcd = cv2.QRCodeDetector()
    retval, decoded_info, points, straight_qrcode = qcd.detectAndDecodeMulti(image)

    if retval:
        # calculate distance to the qr code
        print("QR code found!")
        print("its content is: " + str(decoded_info))
        print("its dimensions are: " + str(points))
        image_dimensions = image.shape
        image_center = (image_dimensions[1] / 2, image_dimensions[0] / 2)
        qr_code_center = (sum(points[0][0]) / 4, sum(points[0][1]) / 4)
        print(f"Image center: {image_center}")
        print(f"QR code center: {qr_code_center}")
        # return the distance to the qr code in METERS
        return (qr_code_center[0] - image_center[0], qr_code_center[1] - image_center[1]) 
    else:
        print("No QR code found")
        return (0,0)

    


def main():
    print(cv2.__version__)
    #example of using micro and speakers
    
    # Capture image
    camera_capture = cv2.VideoCapture(0)
    rv, image = camera_capture.read()
    # add image processing here to detect object
    
    coords = detect_object(image)
    camera_capture.release()
    timer = int(time.time())
    while (coords[0] == 0 and coords[1] == 0):
        # keep trying again for five seconds
        if (int(time.time()) - timer > 5):
            print("No QR code found")
            break
        rv, image = camera_capture.read()
        # add image processing here to detect object
        coords = detect_object(image)
        camera_capture.release()
        
        

    # Use wrapper in context manager to lease control, turn on E-Stop, power on the robot and stand up at start
    # and to return lease + sit down at the end
    with SpotController(username=SPOT_USERNAME, password=SPOT_PASSWORD, robot_ip=ROBOT_IP) as spot:
        # Move head to specified positions with intermediate time.sleep

        # make the head level and look straight
        spot.move_head_in_points(yaws=[0, 0],
                                 pitches=[0, 0],
                                 rolls=[0, 0],
                                 sleep_after_point_reached=1)
        time.sleep(1)

        while (coords[0] != 0 and coords[1] != 0):
            # move head up and down to signal that it is searching for the object
            spot.move_head_in_points(yaws=[0, 0],
                                     pitches=[0.0, 0.0],
                                     rolls=[0.3, -0.3],
                                     sleep_after_point_reached=1)
            time.sleep(1)
            print(f"QR code is {coords[0]} mm to the right and {coords[1]} mm below the center of the image") 
            # move the robot to the object
            # move the robot to the object
            if coords[0] > 0:
                spot.move_by_velocity_control(v_x=0.1, v_y=0, v_rot=0, cmd_duration=0.5)
            else:
                spot.move_by_velocity_control(v_x=-0.1, v_y=0, v_rot=0, cmd_duration=0.5)
            if coords[1] > 0:
                spot.move_by_velocity_control(v_x=0, v_y=0.1, v_rot=0, cmd_duration=0.5)
            else:
                spot.move_by_velocity_control(v_x=0, v_y=-0.1, v_rot=0, cmd_duration=0.5)
            time.sleep(1)
            # Capture image
            camera_capture = cv2.VideoCapture(0)
            rv, image = camera_capture.read()
            # add image processing here to detect object
            coords = detect_object(image)
            camera_capture.release()

        
        time.sleep(2)

        # move head left and right to signal that it hasn't found the object
        spot.move_head_in_points(yaws=[0.2, -0.2],
                                 pitches=[0.0, 0.0],
                                 rolls=[0.0, 0.0],
                                 sleep_after_point_reached=1)
        # save image locally 
        # describe image in ascii
        cv2.imwrite("image.jpg", image)


if __name__ == '__main__':
    main()
