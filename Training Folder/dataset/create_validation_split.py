import os
import shutil
import random
import argparse
from pathlib import Path
import logging

def setup_logging():
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        datefmt='%H:%M:%S'
    )
    return logging.getLogger(__name__)

import yaml

def update_data_yaml(dest_dir, classes_file):
    """
    Updates data.yaml with classes from classes.txt.
    """
    logger = logging.getLogger(__name__)
    dest_path = Path(dest_dir).resolve()
    yaml_path = dest_path / 'data.yaml'
    
    if not classes_file or not classes_file.exists():
        logger.warning("classes.txt not found. Skipping data.yaml update.")
        return

    # Read classes
    with open(classes_file, 'r') as f:
        classes = [line.strip() for line in f.readlines() if line.strip()]
    
    logger.info(f"Found {len(classes)} classes in {classes_file.name}")
    
    # Read existing YAML or create new
    if yaml_path.exists():
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f) or {}
    else:
        data = {}
        
    # Update data
    data['train'] = 'train/images'
    data['val'] = 'valid/images'
    # data['test'] = '' # Optional
    data['nc'] = len(classes)
    data['names'] = classes
    
    # Write back
    with open(yaml_path, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)
        
    logger.info(f"Updated {yaml_path}")

def create_split(source_dir, dest_dir, split_percentage=0.2, seed=42):
    """
    Copies images and labels from source directory to destination directory,
    splitting them into train and validation sets.
    """
    logger = setup_logging()
    random.seed(seed)
    
    source_path = Path(source_dir).resolve()
    dest_path = Path(dest_dir).resolve()
    
    # Source paths
    src_images_dir = source_path / 'images'
    
    # Check for 'labels' or 'label' directory
    src_labels_dir = None
    if (source_path / 'labels').exists():
        src_labels_dir = source_path / 'labels'
    elif (source_path / 'label').exists():
        src_labels_dir = source_path / 'label'
    
    classes_file = None
    if src_labels_dir:
         if (src_labels_dir / 'classes.txt').exists():
             classes_file = src_labels_dir / 'classes.txt'
    
    if not src_labels_dir:
        logger.warning(f"No labels directory found in {source_path}. Looking for 'labels' or 'label'.")

    if not src_images_dir.exists():
        logger.error(f"Images directory not found: {src_images_dir}")
        return

    # Destination paths
    train_images_dir = dest_path / 'train' / 'images'
    train_labels_dir = dest_path / 'train' / 'labels'
    valid_images_dir = dest_path / 'valid' / 'images'
    valid_labels_dir = dest_path / 'valid' / 'labels'
    
    for d in [train_images_dir, train_labels_dir, valid_images_dir, valid_labels_dir]:
        d.mkdir(parents=True, exist_ok=True)
        
    # Get list of images
    supported_extensions = {'.jpg', '.jpeg', '.png', '.bmp', '.tif', '.tiff', '.webp'}
    images = [f for f in os.listdir(src_images_dir) if Path(f).suffix.lower() in supported_extensions]
    
    total_images = len(images)
    if total_images == 0:
        logger.info("No images found in source directory.")
        return

    # Shuffle
    random.shuffle(images)
    
    # Split
    num_valid = int(total_images * split_percentage)
    valid_images = set(images[:num_valid])
    train_images = set(images[num_valid:])
    
    logger.info(f"Total images: {total_images}")
    logger.info(f"Training: {len(train_images)}")
    logger.info(f"Validation: {len(valid_images)}")
    
    def copy_files(image_list, dest_img_dir, dest_lbl_dir):
        count = 0
        for img_name in image_list:
            # Copy image
            shutil.copy2(src_images_dir / img_name, dest_img_dir / img_name)
            
            # Copy label if exists
            if src_labels_dir:
                label_name = Path(img_name).stem + '.txt'
                src_label = src_labels_dir / label_name
                if src_label.exists():
                    shutil.copy2(src_label, dest_lbl_dir / label_name)
                else:
                    # Optional: Log missing label only if expecting strict labeling
                    pass
            count += 1
        return count

    logger.info("Copying validation files...")
    copy_files(valid_images, valid_images_dir, valid_labels_dir)
    
    logger.info("Copying training files...")
    copy_files(train_images, train_images_dir, train_labels_dir)
    
    # Update data.yaml
    if classes_file:
        logger.info("Updating data.yaml...")
        update_data_yaml(dest_dir, classes_file)
    
    logger.info("Done!")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Split dataset into train and validation.")
    parser.add_argument('--source', type=str, default='../data', help='Source data directory (containing images/ and labels/)')
    parser.add_argument('--dest', type=str, default='.', help='Destination dataset directory')
    parser.add_argument('--val-split', type=float, default=0.2, help='Validation split percentage (default: 0.2)')
    parser.add_argument('--seed', type=int, default=42, help='Random seed')
    
    args = parser.parse_args()
    
    create_split(args.source, args.dest, args.val_split, args.seed)
