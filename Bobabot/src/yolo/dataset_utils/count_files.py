import os

def count_files(directory):
    # List all files (not directories) in the given path
    files = [f for f in os.listdir(directory) if os.path.isfile(os.path.join(directory, f))]
    print(f"Number of files in '{directory}': {len(files)}")

# Example usage
count_files("datasets/custom/images/train")

