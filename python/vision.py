import cv2
import numpy as np
import os
import pandas as pd

def get_diff_y(img):
    """
    Calculates the vertical difference (diff_y) between the image center
    and the detected circle center from a numpy image array.
    Returns a dictionary with details or None if no circle is detected.
    """
    if img is None:
        return None

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    N, M = img.shape[:2]

    centerM = int(M/2)-90
    gunN, gunM = int(N*0.22), int(M*0.15)
    gray[-gunN:, :] = 255
    
    # Thresholding (as per notebook logic: binary = (gray < 50))
    binary_circle = (gray < 50).astype(np.uint8)

    binary_full = (gray > 100).astype(np.uint8)

    # Find contours
    contours, _ = cv2.findContours(binary_circle.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    detected = False
    center = None
    radius = 0
    
    if contours:
        # Pick the largest contour
        largest = max(contours, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(largest)
        center = (int(round(x)), int(round(y)))
        radius = int(round(radius))
        detected = True
    else:
        # Fallback: HoughCircles
        blur = cv2.GaussianBlur(binary_circle, (9, 9), 2)
        circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, dp=1.2, minDist=20,
                                   param1=50, param2=30, minRadius=0, maxRadius=0)
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            best = max(circles, key=lambda c: c[2])
            center = (int(best[0]), int(best[1]))
            radius = int(best[2])
            detected = True

    if not detected:
        return None

    # Calculate diff_y
    # Image center (M//2, N//2) -> (width//2, height//2)
    # img.shape is (height, width, channels)
    h, w = img.shape[:2]
    img_center = (w // 2, h // 2)
    
    if np.sum(binary_full)/(N*M) < 0.9:
        diff_y = 1e6
    else:
        diff_y = img_center[1] - center[1]
    
    return {
        'diff_y': diff_y,
        'circle_center': center,
        'img_center': img_center,
        'radius': radius,
        'ratio': np.sum(binary_full)/(N*M)
    }

def process_image(image_path):
    """
    Processes an image to find the vertical difference (diff_y) between
    the image center and the detected circle center.
    """
    if not os.path.exists(image_path):
        print(f"Warning: Image not found at {image_path}")
        return None

    img = cv2.imread(image_path)
    if img is None:
        print(f"Warning: Failed to load image at {image_path}")
        return None

    result = get_diff_y(img)
    if result:
        return result['diff_y']
    return None

def main():
    results = []

    min_angle = 0
    max_angle = 180
    
    # Loop from 0 to 180 (inclusive)
    for angle in range(min_angle, max_angle + 1):
        
        # Using static image for simulation as per plan
        image_path = os.path.join(os.path.dirname(__file__), "..", "foto_center.jpg")
        
        diff_y = process_image(image_path)
        
        if diff_y is not None:
            results.append({'angle': angle, 'y_diff': diff_y})
        else:
            # If detection fails, we might want to record None or skip
            # For now, let's skip but print a warning if it was a real scenario
            pass

    # Create DataFrame
    df = pd.DataFrame(results)
    
    # Print DataFrame
    print(df)
    
    # Save DataFrame to CSV
    df.to_csv('results.csv', index=False)

if __name__ == "__main__":
    main()
