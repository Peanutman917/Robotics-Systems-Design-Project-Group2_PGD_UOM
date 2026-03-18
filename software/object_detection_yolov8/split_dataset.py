import os
import random
import shutil
from pathlib import Path

# 数据集根目录：~/detectdata/dataset
ROOT = Path(__file__).resolve().parent / "dataset"

IMAGES_ALL = ROOT / "images" / "all"
LABELS_ALL = ROOT / "labels" / "all"

SPLITS = ["train", "val", "test"]
RATIOS = {
    "train": 0.7,
    "val": 0.2,
    "test": 0.1,
}

def clear_split_dirs():
    for split in SPLITS:
        img_dir = ROOT / "images" / split
        lbl_dir = ROOT / "labels" / split
        shutil.rmtree(img_dir, ignore_errors=True)
        shutil.rmtree(lbl_dir, ignore_errors=True)
        img_dir.mkdir(parents=True, exist_ok=True)
        lbl_dir.mkdir(parents=True, exist_ok=True)

def main():
    # 只选有对应 label 的图片
    image_files = sorted([
        f for f in IMAGES_ALL.glob("*.*")
        if f.suffix.lower() in [".jpg", ".jpeg", ".png"]
        and (LABELS_ALL / (f.stem + ".txt")).exists()
    ])

    n = len(image_files)
    print(f"Found {n} labeled images in ALL.")

    if n == 0:
        print("No labeled images found, exit.")
        return

    random.seed(2025)
    random.shuffle(image_files)

    n_train = int(n * RATIOS["train"])
    n_val = int(n * RATIOS["val"])
    n_test = n - n_train - n_val

    splits = {
        "train": image_files[:n_train],
        "val": image_files[n_train:n_train + n_val],
        "test": image_files[n_train + n_val:],
    }

    print(
        f"Split -> train: {len(splits['train'])}, "
        f"val: {len(splits['val'])}, test: {len(splits['test'])}"
    )

    clear_split_dirs()

    for split, files in splits.items():
        img_dst = ROOT / "images" / split
        lbl_dst = ROOT / "labels" / split
        img_dst.mkdir(parents=True, exist_ok=True)
        lbl_dst.mkdir(parents=True, exist_ok=True)

        for img_path in files:
            label_path = LABELS_ALL / (img_path.stem + ".txt")
            shutil.copy2(img_path, img_dst / img_path.name)
            shutil.copy2(label_path, lbl_dst / label_path.name)

    print("Done splitting.")

if __name__ == "__main__":
    main()
