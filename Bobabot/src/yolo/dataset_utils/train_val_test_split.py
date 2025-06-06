import os
import random
import shutil
from pathlib import Path

# Config
BASE_DIR = Path(__file__).parent.parent
IMAGES_DIR = BASE_DIR / "datasets" / "custom" / "images"

train_dir = IMAGES_DIR / "train"
val_dir = IMAGES_DIR / "val"
test_dir = IMAGES_DIR / "test"
train_ratio, val_ratio, test_ratio = 0.7, 0.25, 0.05

# Make sure split ratios add up to 1.0
assert abs((train_ratio + val_ratio + test_ratio) - 1.0) < 1e-6, "Ratios must sum to 1"

# Create val and test folders if they don't exist
os.makedirs(val_dir, exist_ok=True)
os.makedirs(test_dir, exist_ok=True)

# Collect and shuffle image files from train
image_files = [f for f in os.listdir(train_dir) if f.lower().endswith(('.jpg', '.png', '.jpeg'))]
random.shuffle(image_files)

# Split
total = len(image_files)
train_cut = int(total * train_ratio)
val_cut = train_cut + int(total * val_ratio)

# Move files from train -> val and test only
val_files = image_files[train_cut:val_cut]
test_files = image_files[val_cut:]

# Move files
for file in val_files:
    shutil.move(
        str(train_dir / file),
        str(val_dir / file)
    )

for file in test_files:
    shutil.move(
        str(train_dir / file),
        str(test_dir / file)
    )

print("✅ Dataset split complete!")
print(f"Train: {train_cut} images")
print(f"Val: {len(val_files)} images")
print(f"Test: {len(test_files)} images")
