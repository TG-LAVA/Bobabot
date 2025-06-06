"""
Instructions for use:
- Default mode is auto. In this mode, left click to select an object
- To switch to manual mode, press TAB. In this mode, left click to select points of a polygon
- To close the polygon and finish selection of the object, right click
- If the flag `one_class_only` is set to True, all polygons will be automatically assigned class 0
- If `one_class_only` is False, the selected polygon will be red, indicating it is unlabelled
- Assign a class to the polygon by pressing a number key (0-9)
- The color of the polygon will change based on the class assigned
- 0 is green, 1 is yellow. Other classes (there are no other classes as of now) will be orange
- To switch back to auto mode, press TAB again
- To undo the last action, press 'z'
- To delete the current image, press 'd'
- To go to the next image, press ENTER
"""

import os
import cv2
import numpy as np
import torch
from segment_anything import sam_model_registry, SamPredictor

# Settings
one_class_only = True  # If True, auto-label objects with class 0

# Constants
TAB_KEY = 9  # Key code for TAB
CARRIAGE_RETURN = 13  # Key code for Enter

# Config
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
IMAGE_DIR = os.path.join(BASE_DIR, "datasets", "custom", "images", "val")
LABEL_DIR = os.path.join(BASE_DIR, "datasets", "custom", "labels", "val")

SAM_CHECKPOINT = os.path.join(BASE_DIR, "dataset_utils", "sam_vit_b_01ec64.pth")
MODEL_TYPE = "vit_b"
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

os.makedirs(LABEL_DIR, exist_ok=True)

# Load SAM
sam = sam_model_registry[MODEL_TYPE](checkpoint=SAM_CHECKPOINT).to(DEVICE)
predictor = SamPredictor(sam)

# Globals
clicked = []
polygons = []
manual_polygon = []
image = None
rgb_image = None
w, h = 0, 0
mode = "auto"  # default mode


def normalise_coords(coords, width, height):
    coords[:, 0] /= width
    coords[:, 1] /= height
    return coords.flatten()

def all_polygons_classified(polygons):
    return all(isinstance(p, tuple) for p in polygons)

def mask_to_polygon(mask):
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        if len(cnt) >= 3:
            return cnt.squeeze()
    return None

def on_click(event, x, y, flags, param):
    global clicked, polygons, manual_polygon, image, rgb_image, w, h, mode

    if event == cv2.EVENT_LBUTTONDOWN:
        if mode == "auto":
            clicked.append((x, y))
            input_point = np.array([[x, y]])
            input_label = np.array([1])
            masks, _, _ = predictor.predict(
                point_coords=input_point,
                point_labels=input_label,
                multimask_output=False
            )
            polygon = mask_to_polygon(masks[0])
            if polygon is not None:
                # Automatically assign class 0 if one_class_only is True
                if one_class_only:
                    polygons.append((polygon, 0))
                else:
                    polygons.append(polygon)
        else:  # manual mode
            manual_polygon.append((x, y))
    elif event == cv2.EVENT_RBUTTONDOWN and mode == "manual":
        if len(manual_polygon) >= 3:
            # Automatically assign class 0 if one_class_only is True
            if one_class_only:
                polygons.append((np.array(manual_polygon, dtype=np.int32), 0))
            else:
                polygons.append(np.array(manual_polygon, dtype=np.int32))
            manual_polygon.clear()

def draw_polygons(img, polys, current_manual=[], clicked_points=[]):
    overlay = img.copy()
    for poly_info in polys:
        # Unpack tuple (poly, class_id) or handle raw poly (before class assignment)
        if isinstance(poly_info, tuple):
            poly, class_id = poly_info
            # Different colors for different classes
            if class_id == 0:
                color = (0, 255, 0)      # Green for class 0
            elif class_id == 1:
                color = (0, 255, 255)    # Yellow for class 1
            else:
                color = (0, 165, 255)    # Orange for other classes
        else:
            poly = poly_info
            color = (0, 0, 255) 

        # Ensure poly is numpy array before reshaping
        if not isinstance(poly, np.ndarray):
            poly = np.array(poly)
            
        poly = poly.reshape(-1, 2)
        cv2.polylines(overlay, [poly.astype(np.int32)], isClosed=True, color=color, thickness=2)
        cv2.fillPoly(overlay, [poly.astype(np.int32)], (*color, 50))  # Semi-transparent fill

    # Draw manual polygon points and lines
    for pt in current_manual:
        cv2.circle(overlay, pt, 3, (0, 0, 255), -1)  # Increased size for better visibility
    if len(current_manual) > 1:
        for i in range(len(current_manual) - 1):
            cv2.line(overlay, current_manual[i], current_manual[i+1], (0, 0, 255), 1)

    # Draw clicked points for auto mode
    for pt in clicked_points:
        cv2.circle(overlay, pt, 3, (0, 0, 0), -1)  # Increased size to match manual points
            
    return overlay

def main():
    global clicked, polygons, manual_polygon, image, rgb_image, w, h, mode

    print(f"Loading images from: {IMAGE_DIR}")

    for filename in sorted(os.listdir(IMAGE_DIR)):
        image_deleted = False  # Flag to track if image is deleted; if deleted, NO label should be made
        if not filename.lower().endswith((".jpg", ".png", ".jpeg")):
            continue

        # Skip image if label file already exists
        label_path = os.path.join(LABEL_DIR, os.path.splitext(filename)[0] + ".txt")
        if os.path.exists(label_path):
            print(f"⏭️  Skipping {filename} (already labeled)")
            continue

        print("🖼️  Loading:", filename)
        image_path = os.path.join(IMAGE_DIR, filename)
        image = cv2.imread(image_path)
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        h, w = rgb_image.shape[:2]
        predictor.set_image(rgb_image)

        clicked.clear()
        polygons.clear()
        manual_polygon.clear()
        mode = "auto"

        cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Image", 1920, 1080)
        cv2.setWindowTitle("Image", filename)
        cv2.setMouseCallback("Image", on_click)

        while True:
            display = draw_polygons(image.copy(), polygons, manual_polygon, clicked)
            cv2.imshow("Image", display)

            key = cv2.waitKey(10) & 0xFF
            if key == ord("d"):
                # Delete the image
                os.remove(image_path)
                print(f"🗑️  Deleted {filename}")
                cv2.destroyAllWindows()
                image_deleted = True  # Set flag
                break
            elif key == CARRIAGE_RETURN:
                if not polygons:
                    break  # Allow quitting if no polygons drawn
                if one_class_only or all_polygons_classified(polygons):
                    break
                else:
                    print("⚠️  Warning: Some polygons are not classified!")
                    print("💡 Use number keys (0-9) to assign classes to all polygons before proceeding.")
                    continue
            elif key == ord("z"):
                if mode == "auto" and polygons:
                    polygons.pop()
                    if clicked:
                        clicked.pop()
                    print("🔄 Undo last polygon.")
                elif mode == "manual" and manual_polygon:
                    manual_polygon.pop()
                    print("🔄 Undo last point.")
            elif key == TAB_KEY:
                mode = "manual" if mode == "auto" else "auto"
                print(f"🔁 Mode switched to: {mode.upper()}")

            elif key in [ord(str(i)) for i in range(10)]:
                class_id = int(chr(key))
                if polygons:
                    last_poly = polygons.pop()
                    polygons.append((last_poly, class_id))
                    print(f"📄 Assigned class {class_id} to last polygon.")

        cv2.destroyAllWindows()

        if image_deleted:
            continue

        if polygons:
            label_path = os.path.join(LABEL_DIR, os.path.splitext(filename)[0] + ".txt")
            with open(label_path, "w") as f:
                for poly, class_id in polygons:
                    norm = normalise_coords(poly.astype(float), w, h)
                    f.write(f"{class_id} " + " ".join(f"{x:.6f}" for x in norm) + "\n")
            print(f"📂 Saved {len(polygons)} polygons to {label_path}")
        else:
            # Create empty label file
            label_path = os.path.join(LABEL_DIR, os.path.splitext(filename)[0] + ".txt")
            with open(label_path, "w") as f:
                pass  # Create empty file
            print("📂 Created empty label file (no polygons)")

    print("✅ Done.")

if __name__ == "__main__":
    main()

