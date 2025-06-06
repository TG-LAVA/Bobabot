import cv2
import numpy as np

# Create a simple image
image = np.zeros((300, 300, 3), dtype=np.uint8)
cv2.putText(image, 'OpenCV Test', (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 
            1, (255, 255, 255), 2, cv2.LINE_AA)

# Create a named window (optional, but good for debugging)
cv2.namedWindow('Test Window', cv2.WINDOW_NORMAL)

# Show the image
cv2.imshow('Test Window', image)

# Wait for a key press indefinitely or for 5000 ms
print("Press any key in the image window to close...")
key = cv2.waitKey(5000)  # You can change this to 0 for infinite wait

# Destroy all windows
cv2.destroyAllWindows()

print("Done.")
