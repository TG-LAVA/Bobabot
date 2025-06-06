import os
import cv2
import numpy as np

# Configuration
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
IMAGE_DIR = os.path.join(BASE_DIR, "datasets", "custom", "images", "train")
LABEL_DIR = os.path.join(BASE_DIR, "datasets", "custom", "labels", "train")

IMAGE_PATH = os.path.join(IMAGE_DIR, "redcup_20250524_143420.jpg")
LABEL_PATH = os.path.join(LABEL_DIR, "redcup_20250524_143420.txt")
# IMAGE_PATH = os.path.join(IMAGE_DIR, "000000000034.jpg")
# LABEL_PATH = os.path.join(LABEL_DIR, "000000000034.txt")

CLASS_COLORS = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]  # Colors for up to 3 classes


def load_yolo_seg_labels(label_path, image_shape):
    h, w = image_shape[:2]
    polygons = []

    with open(label_path, "r") as file:
        for line in file:
            parts = line.strip().split()
            class_id = int(parts[0])
            coords = list(map(float, parts[1:]))

            # Convert back to absolute pixel coordinates
            pts = np.array(coords).reshape(-1, 2)
            pts[:, 0] *= w
            pts[:, 1] *= h
            polygons.append((class_id, pts.astype(int)))

    return polygons


def visualize(image_path, label_path):
    image = cv2.imread(image_path)
    if image is None:
        print(f"Error loading image: {image_path}")
        return

    polygons = load_yolo_seg_labels(label_path, image.shape)

    for class_id, polygon in polygons:
        color = CLASS_COLORS[class_id % len(CLASS_COLORS)]
        cv2.polylines(image, [polygon], isClosed=True, color=color, thickness=2)
        cv2.putText(image, f"Class {class_id}", tuple(polygon[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    cv2.imshow("Labeled Image", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    visualize(IMAGE_PATH, LABEL_PATH)
