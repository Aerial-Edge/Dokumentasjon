"""
This script demonstrates color filtering, circle detection using Hough transform,
FPS calculation, and displaying distance, x and y position of three color balls -
orange, deep red (VINRØD in Norwegian) and green (GRØNN in Norwegian).
"""

import cv2 as cv
from cvzone.FPS import FPS
import imutils
import math
import numpy as np

def calculateDistance(ballRadius_px):
    """
    Calculate the distance from the camera to the object based on its radius in pixels.
    
    Args:
        ballRadius_px: The radius of the ball in pixels.
        
    Returns:
        The calculated distance in cm.
    """
    return int(faktor / ballRadius_px)


def detect_colored_object(colorLower, colorUpper, min_radius, max_radius):
    """
    Detect a colored object within a given color range and size.
    
    Args:
        colorLower: The lower boundary of the color range in HSV format.
        colorUpper: The upper boundary of the color range in HSV format.
        min_radius: The minimum radius of the object to detect.
        max_radius: The maximum radius of the object to detect.
        
    Returns:
        The (x, y) coordinates and radius of the detected object, or None if no object is detected.
    """
    # Create a mask based on the given color range
    mask = cv.inRange(hsv, colorLower, colorUpper)
    
    # Erode the mask to remove noise
    mask = cv.erode(mask, None, iterations=2)
    
    # Dilate the mask to fill gaps
    mask = cv.dilate(mask, None, iterations=2)

    # Convert the frame to grayscale and apply median blur
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    gray = cv.medianBlur(gray, 5)

    # Detect circles using the Hough transform
    circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, 20, param1=100, param2=30,
                              minRadius=min_radius, maxRadius=max_radius)

    # Check if any circles were detected
    if circles is not None:
        circles = np.uint16(np.around(circles))

        for circle in circles[0, :]:
            x, y, radius = circle

            # Check if the circle's center is within the mask's boundaries
            if 0 <= x < mask.shape[1] and 0 <= y < mask.shape[0] and mask[y, x] > 0:
                return (x, y, radius)
    return None


def display_object_info(frame, x, y, radius, distance, color, text_offset):
    """
    Display information about the detected object on the frame.
    
    Args:
        frame: The frame on which to display the information.
        x: The x-coordinate of the object.
        y: The y-coordinate of the object.
        radius: The radius of the object.
        distance: The distance to the object.
        color: The color of the circle to be drawn around the object.
        text_offset: The offset of the distance text under the FPS indicator.
    """
    if x is not None and y is not None:
        # Draw a circle around the detected object
        cv.circle(frame, (x, y), radius, color, 2)
        
        # Prepare the coordinate text
        coordinates_text = f"X: {x}, Y: {y}"
        
        # Prepare the distance text
        distance_text = f"Distance: {distance} cm"
		
		# Put the coordinate text on the frame
        cv.putText(frame, coordinates_text, (x + 10, y), cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Put the distance text on the frame
        cv.putText(frame, distance_text, (22, 70 + text_offset), cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


# Distance measurement parameters:
ballRadius = 3.25   # cm (radius of the ball)
cameraFOV = 62.2    # degrees (field of view of the camera)
faktor = (1280 / 2) * (ballRadius / math.tan(math.radians(cameraFOV / 2)))

# Color detection settings:
colors = {
    'green': {   # "GRØNN" in Norwegian
        'lower': (72, 70, 32), #(L-H, L-S, L-V)
        'upper': (99, 244, 107), #(U-H,  U-S, U-V)
        'min_radius': 0, # ex between 20 
        'max_radius': 0, # to 60 pixels
        'color': (0, 255, 0), # Color of the circle around object
        'text_offset': 0, # Distance text position under FPS
    },
    'orange': {
        'lower': (0, 115, 99), #(L-H, L-S, L-V)
        'upper': (18, 255, 255), #(U-H,  U-S, U-V)
        'min_radius': 0,
        'max_radius': 0,
        'color': (0, 102, 255), # Color of the circle around object
        'text_offset': 20, # Distance text position under FPS
    },
    'red': {  # "VINRØD" in Norwegian, which means deep red or wine red
        'lower': (119, 37, 0), #(L-H, L-S, L-V)
        'upper': (179, 179, 147), #(U-H,  U-S, U-V)
        'min_radius': 0,
        'max_radius': 0,
        'color': (0, 0, 255), # Color of the circle around object
        'text_offset': 40,  # Distance text position under FPS
    },
}

fpsreader = FPS() # Initialize FPS reader
videoCap = cv.VideoCapture(0) # Initialize video capture
# Set the frame width and height
videoCap.set(3, 1280)
videoCap.set(4, 720)

# Main loop
while True:
    (grabbed, frame) = videoCap.read() # Read a frame from the video capture
    fps, img = fpsreader.update(frame, color=(255, 0, 0)) # Update the FPS overlay on the frame
    frame = imutils.resize(frame, width=1280) # Resize the frame
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV) # Convert the frame to HSV format

    # Initialize x, y, and ballRadius_px values for each color
    for color_info in colors.values():
        color_info['x'] = None
        color_info['y'] = None
        color_info['ballRadius_px'] = None

    # Iterate through the defined colors and detect objects
    for idx, (color_name, color_info) in enumerate(colors.items()):
        # Call the detect_colored_object function to find objects in the frame
        obj = detect_colored_object(color_info['lower'], color_info['upper'], color_info['min_radius'],
                                    color_info['max_radius'])

        if obj: # If an object is detected
            x, y, ballRadius_px = obj # Get the coordinates and radius of the object
            distance = calculateDistance(ballRadius_px) # Calculate the distance to the object
            
            # Update the color_info dictionary with the new values
            color_info['x'] = x
            color_info['y'] = y
            color_info['ballRadius_px'] = ballRadius_px
            color_info['distance'] = distance

        # Display information about the detected object on the frame
        display_object_info(frame, color_info['x'], color_info['y'], color_info['ballRadius_px'],
                            color_info.get('distance'), color_info['color'], color_info['text_offset'])

    # Show the frame with the detected objects and information
    cv.imshow("Frame", frame)
    
    # Wait for a key press
    key = cv.waitKey(1)
    
    # Stop the program when the 'q' key is pressed
    if key == ord("q"):
        break


