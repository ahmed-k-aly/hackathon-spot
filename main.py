import os
import time
from spot_controller import SpotController

ROBOT_IP = "10.0.0.3"#os.environ['ROBOT_IP']
SPOT_USERNAME = "admin"#os.environ['SPOT_USERNAME']
SPOT_PASSWORD = "2zqa8dgw7lor"#os.environ['SPOT_PASSWORD']
TIMEOUT_LIMIT = 60 # IN SECONDS
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

def try_to_detect(spot):
    print("Trying to detect QR code...")
        # search algorithm
    distance = search(spot)
    return distance


def search(spot):
    """A depth first search algorithm to search for the object in the environment by ONLY turning the head of the robot.

    Args:
        spot (SpotController): an instance of the SpotController class to control the robot's movement. 
    """
    # The robot should only rotate its head in increments of 30 degrees to allow continuous scanning of the environment. Angles are in radians.
    possibleAngles = [0, 0.523599, 1.0472, 1.5708, 2.0944, 2.61799, 3.14159, -0.523599, -1.0472, -1.5708, -2.0944, -2.61799, -3.14159]
    possibleDirections = ["left", "right", "up", "down", "doNothing"]
    
    LOOP_TIMEOUT = 20  # in seconds
    
    # map angles and directions to yaws and pitches
    timer = int(time.time())
    frontier = []
    explored = []
    # The robot should start by looking straight ahead
    frontier.append((0, "doNothing"))
    while frontier:
        if (int(time.time()) - timer) > LOOP_TIMEOUT: # 10 seconds
            print("Time out")
            return -1
        camera_capture = cv2.VideoCapture(0)
        # Pop the last element from the frontier
        current = frontier.pop()
        # move the head to the current state
        if current[1] == "left":
            print("Moving head left")
            spot.move_head_in_points(yaws=[current[0], current[0]],
                                     pitches=[0.0, 0.0],
                                     rolls=[0.3, -0.3],
                                     sleep_after_point_reached=1)
        elif current[1] == "right":
            print("Moving head right")
            spot.move_head_in_points(yaws=[current[0], current[0]],
                                     pitches=[0.0, 0.0],
                                     rolls=[-0.3, 0.3],
                                     sleep_after_point_reached=1)
        elif current[1] == "up":
            print("Moving head up")
            spot.move_head_in_points(yaws=[current[0], current[0]],
                                     pitches=[0.3, -0.3],
                                     rolls=[0.0, 0.0],
                                     sleep_after_point_reached=1)
        elif current[1] == "down":
            print("Moving head down")
            spot.move_head_in_points(yaws=[current[0], current[0]],
                                     pitches=[-0.3, 0.3],
                                     rolls=[0.0, 0.0],
                                     sleep_after_point_reached=1)
        rv, image = camera_capture.read()
        camera_capture.release()

            
        # Check if the current state is the goal state
        distance = detected_qr_code(image)
        if distance != -1:
            print("QR code found")
            return distance
        # Add the current state to the explored set
        explored.append(current)
        # Generate the next states from the current state
        for angle in possibleAngles:
            for direction in possibleDirections:
                if (angle, direction) not in explored:
                    frontier.append((angle, direction))

               
    print("QR code not found")
    return -1

        
    
def detected_qr_code(image):
    distance = detect_object(image)
    return distance


def main():
    print(cv2.__version__)
    #example of using micro and speakers

    # Use wrapper in context manager to lease control, turn on E-Stop, power on the robot and stand up at start
    # and to return lease + sit down at the end
    with SpotController(username=SPOT_USERNAME, password=SPOT_PASSWORD, robot_ip=ROBOT_IP) as spot:

        # make the head level and look straight
        spot.move_head_in_points(yaws=[0, 0],
                                 pitches=[0, 0],
                                 rolls=[0, 0],
                                 sleep_after_point_reached=1)
        # Move head to specified positions with intermediate time.sleep
        time.sleep(1)
        timer = int(time.time())
        distance = try_to_detect(spot)

        while (distance > 0):
            if (int(time.time()) - timer) > TIMEOUT_LIMIT : 
                print("Time out at outer loop")
                return -1
            # move head up and down to signal that it is searching for the object
            spot.move_head_in_points(yaws=[0, 0],
                                     pitches=[0.0, 0.0],
                                     rolls=[0.3, -0.3],
                                     sleep_after_point_reached=1)
            time.sleep(1)
            if (distance < 25):
                print("Object is close enough to the robot")
                print(f"Distance to object is {distance} cm")
                # move back 
                spot.move_to_goal(goal_x=-distance/100, goal_y=0)
            else:
                print("Object is away from  the robot")
                print(f"Distance to object is {distance} cm")
            # move forward
                spot.move_to_goal(goal_x=distance/100, goal_y=0)
            
                
            time.sleep(1)
            # Capture image
            distance = try_to_detect(spot)
        
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
