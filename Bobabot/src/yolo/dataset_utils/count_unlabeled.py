"""
Counts how many images in a folder are still unlabeled (no corresponding txt file)
"""

import os

# Config
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
IMAGE_DIR = os.path.join(BASE_DIR, "datasets", "custom", "images", "train")
LABEL_DIR = os.path.join(BASE_DIR, "datasets", "custom", "labels", "train")

def main():
    num_unlabeled = 0
    for filename in sorted(os.listdir(IMAGE_DIR)):
        label_path = os.path.join(LABEL_DIR, os.path.splitext(filename)[0] + ".txt")
        if not os.path.exists(label_path):
            num_unlabeled += 1

    print(f"Number of unlabeled images: {num_unlabeled}")


if __name__ == "__main__":
    main()