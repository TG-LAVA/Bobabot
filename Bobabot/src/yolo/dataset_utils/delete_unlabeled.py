"""
Deletes images in the training set that do not have corresponding labels.
"""

import os

BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
IMAGE_DIR = os.path.join(BASE_DIR, "datasets", "custom", "images", "train")
LABEL_DIR = os.path.join(BASE_DIR, "datasets", "custom", "labels", "train")

image_extensions = {".jpg", ".jpeg", ".png"}

# Get label basenames (without extension)
label_basenames = {os.path.splitext(f)[0] for f in os.listdir(LABEL_DIR) if f.endswith(".txt")}

# Loop through images and delete those without corresponding labels
for image_file in os.listdir(IMAGE_DIR):
    name, ext = os.path.splitext(image_file)
    if ext.lower() in image_extensions and name not in label_basenames:
        image_path = os.path.join(IMAGE_DIR, image_file)
        os.remove(image_path)
        print(f"Deleted: {image_path}")