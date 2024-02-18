import os
import time
from spot_controller import SpotController

ROBOT_IP = "10.0.0.3"#os.environ['ROBOT_IP']
SPOT_USERNAME = "admin"#os.environ['SPOT_USERNAME']
SPOT_PASSWORD = "2zqa8dgw7lor"#os.environ['SPOT_PASSWORD']
import cv2
import numpy as np


def estimate_distance(points, real_qr_size=21, focal_length=50, sensor_size=24, image_size=1080):
    """
    Estimates the distance of a QR code from the camera based on the QR code's pixel coordinates.

    Parameters:
    - points: numpy.ndarray representing the coordinates of the four corners of the detected QR Code.
    - real_qr_size: The real-world length of the QR code's side in centimeters. Default is 21 cm for A4 paper width.
    - focal_length: The focal length of the camera in millimeters. Default is 50 mm.
    - sensor_size: The size of the camera sensor in millimeters. Default is 24 mm, assuming a full-frame sensor's height.
    - image_size: The size of the image in pixels along the dimension that corresponds to the sensor_size. Default is 1080 pixels for the height in a 1920x1080 resolution.

    Returns:
    - A list of estimated distances in centimeters for each QR code detected in the image.
    """
    distances = []
    for qr in points:
        # Calculate the width and height of the QR code in pixels
        width_px = np.linalg.norm(qr[0] - qr[1])
        height_px = np.linalg.norm(qr[0] - qr[3])
        # Use the average of width and height for a more accurate estimate
        avg_size_px = (width_px + height_px) / 2
        
        # Convert focal length to centimeters
        focal_length_cm = focal_length / 10
        
        # Calculate the distance using the formula
        distance = (focal_length_cm * real_qr_size * image_size) / (avg_size_px * sensor_size)
        
        distances.append(distance)
    
    return distances


def detect_object(image):
  
    qcd = cv2.QRCodeDetector()
    retval, decoded_info, points, straight_qrcode = qcd.detectAndDecodeMulti(image)

    if retval:
        print(f"QR code found at {points[0]}")
        distance = estimate_distance(points)
        print(f"QR code found at {points[0]} with estimated distance of {distance} cm")
        return distance[0]
        
    else:
        print("No QR code found")
        return -1

def try_to_detect():
# Capture image
    camera_capture = cv2.VideoCapture(0)
    rv, image = camera_capture.read()
    # add image processing here to detect object
    
    distance = detect_object(image)
    camera_capture.release()
    timer = int(time.time())
    while (distance == -1):
        # keep trying again for five seconds
        print("Trying to detect QR code...")
        if (int(time.time()) - timer > 5):
            print("No QR code found")
            break
        camera_capture = cv2.VideoCapture(0)
        rv, image = camera_capture.read()
        # add image processing here to detect object
        distance = detect_object(image)
        camera_capture.release()
    return distance

def main():
    print(cv2.__version__)
    #example of using micro and speakers
    distance = try_to_detect()

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

        while (distance > 0):
            # move head up and down to signal that it is searching for the object
            spot.move_head_in_points(yaws=[0, 0],
                                     pitches=[0.0, 0.0],
                                     rolls=[0.3, -0.3],
                                     sleep_after_point_reached=1)
            time.sleep(1)
            print(f"QR code is {coords[0]} mm to the right and {coords[1]} mm below the center of the image") 
            if (distance < 15):
                print("Object is close enough to the robot")
                break
            else:
                print("Object is away from  the robot")
            # move forward
                spot.move_to_goal(goal_x=distance/100, goal_y=0)
            
                
            time.sleep(1)
            # Capture image
            distance = try_to_detect()
        
        time.sleep(2)

        # move head left and right to signal that it hasn't found the object
        spot.move_head_in_points(yaws=[0.2, -0.2],
                                 pitches=[0.0, 0.0],
                                 rolls=[0.0, 0.0],
                                 sleep_after_point_reached=1)
        # save image locally 
        # describe image in ascii


if __name__ == '__main__':
    main()
