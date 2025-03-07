import numpy as np
import cv2
import os

# List of obstacles with their coordinates and radii
Liobs = [((120, -50), 35), ((-152, -6), 55), ((110, 135), 50), ((12, -102), 30), ((92, 170), 30), ((-92, 176), 40), ((-40, 220), 32), ((-44, -95), 32), ((-30, -150), 32)]

# Create a white image
image_height, image_width = 1000, 1000
image = np.ones((image_height, image_width, 3), dtype=np.uint8) * 255

# Draw black circles on the image
for (center, radius) in Liobs:
    # Convert coordinates to image coordinates
    center_image = (int(center[0] + image_width // 2), int(image_height // 2 - center[1]))
    cv2.circle(image, center_image, int(radius/1.2), (0, 0, 0), -1)

# Save the image
output_path = os.path.join(os.path.dirname(__file__), 'output_image.png')
cv2.imwrite(output_path, image)

# Display the image
cv2.imshow('Image with Circles', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
