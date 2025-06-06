import os
from glob import glob
from ultralytics import YOLO
# import cv2
import yaml
from pathlib import Path
import numpy as np
from matplotlib import pyplot as plt


def evaluate_metrics(model_path, data_yaml_path):
    # Load the trained model
    model = YOLO(model_path)
    # Run evaluation on the test set
    metrics = model.val(data=data_yaml_path, split="test", imgsz=640, save=True, verbose=True)

    print("Box mAP50:", metrics.box.map50)
    print("Box mAP50-95:", metrics.box.map)
    print("Mask mAP50:", metrics.seg.map50)
    print("Mask mAP50-95:", metrics.seg.map)


def review_predictions(model_path, data_yaml_path):
    data_yaml_path = Path(data_yaml_path)
    # Load the trained model
    model = YOLO(model_path)
    
    # Run inference on the test set
    with open(data_yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    # Get the project root directory (where yolo_test.py is)
    project_root = Path(__file__)
    
    # Resolve test directory relative to project root
    dataset_root = project_root / data['path']  # use project root instead of yaml location
    test_dir = dataset_root / data['test']

    # Convert to absolute path
    test_dir = test_dir.resolve()

    # Supported image extensions
    img_extensions = ('*.jpg', '*.jpeg', '*.png')

    # Gather all image paths
    image_paths = []
    for ext in img_extensions:
        image_paths.extend(glob(os.path.join(test_dir, ext)))

    # Run inference and display results one at a time
    for img_path in image_paths:      

        results = model(img_path, imgsz=640, conf=0.25)

        # Get the result image (with masks, boxes, etc.)
        plotted_img = results[0].plot()
        if plotted_img is None:
            print("Warning: No image returned from plot(). Skipping.")
            continue
        if not isinstance(plotted_img, np.ndarray):
            print("Warning: Invalid image returned.")
            continue

        # Show the image using OpenCV
        # cv2.imshow("Segmentation Result", plotted_img)
        plt.imshow(plotted_img)

        # Wait for a key press; press ESC to exit early
        # key = cv2.waitKey(1000)  # Wait indefinitely
        # if key == 27:  # ESC key to break
        #     break

    # cv2.destroyAllWindows()


def main():
    model_path = "runs/segment/train2/weights/best.pt"
    data_yaml_path = "red_cup.yaml"
    # review_predictions(model_path, data_yaml_path)
    evaluate_metrics(model_path, data_yaml_path)


if __name__ == "__main__":
    main()
    