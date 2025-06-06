"""
This file is for prediction on the Jetson Nano using yolo
"""
from ultralytics import YOLO
import numpy as np
import cv2

class CupSegmenter:
    def __init__(self, model_path: str, seg_confidence=0.7, mask_thresh=0.9):
        self.model = YOLO(model_path)
        self.seg_confidence = seg_confidence  # How confident the model should be to consider a segmentation valid
        self.mask_thresh = mask_thresh  # Threshold for converting masks from probability to binary masks (0 or 1)

    def segment(self, img: np.ndarray) -> np.ndarray:
        """
        Segment cups from the image using the YOLO model and return an array of binary masks.
        """
        results = self.model(img, imgsz=640, conf=self.seg_confidence, save=False, verbose=False)
        # Get the first result (batch size 1 assumed)

        r = results[0]  # Assume batch_size = 1

        if r.masks is None or r.masks.data is None:
            return np.zeros((0, img.shape[0], img.shape[1]), dtype=np.uint8)

        masks = r.masks.data.cpu().numpy()  # Shape: [num_instances, H, W]
        binary_masks = (masks > self.mask_thresh).astype(np.uint8)
        return binary_masks


def main():
    model_path = "yolov8n-seg-redcup.pt"
    segmenter = CupSegmenter(model_path=model_path, seg_confidence=0.7, mask_thresh=0.9)

    # Test
    img = cv2.imread("redcup_20250524_145036.jpg")
    binary_masks = segmenter.segment(img)
    if binary_masks.shape[0] > 0:
        print("Objects were detected and segmented!")
    else:
        print("No objects detected.")
        exit(0)
    print("Binary Mask Shape:", binary_masks.shape)
    # Show the first binary mask as an image
    cv2.imshow('Binary Mask 0', binary_masks[2] * 255)  # Multiply by 255 to make mask visible (0 or 255)
    cv2.waitKey(0)  # Wait until any key is pressed
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
    