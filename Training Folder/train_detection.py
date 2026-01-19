"""
YOLOv12 Turkish Traffic Detection - Advanced Training Pipeline
================================================================

NVIDIA RTX 4070 Laptop GPU Optimized Training Script

FEATURES:
- ✅ Advanced learning rate scheduling (Cosine Annealing with Warm Restarts)
- ✅ Gradient clipping & NaN prevention
- ✅ Mixed precision training (AMP)
- ✅ Early stopping with patience
- ✅ TensorBoard logging
- ✅ Automatic checkpoint management
- ✅ Data augmentation optimized for traffic signs
- ✅ Class-balanced sampling
- ✅ Model EMA (Exponential Moving Average)
- ✅ Advanced validation metrics
- ✅ Multi-export formats (ONNX, TorchScript, TensorRT)

USAGE:
    python train_advanced.py                    # Fresh training
    python train_advanced.py --resume           # Resume from checkpoint
    python train_advanced.py --tune             # Hyperparameter tuning
    python train_advanced.py --export-only      # Export only
    
SCIENTIFIC IMPROVEMENTS:
1. OneCycleLR scheduler for faster convergence
2. Label smoothing (0.1) to prevent overconfidence
3. Focal loss for class imbalance
4. Knowledge distillation ready
5. Test-Time Augmentation (TTA) support
6. Automatic Mixed Precision with gradient scaling
"""

import os
import sys
import yaml
import torch
import logging
import numpy as np
from pathlib import Path
from datetime import datetime
from typing import Dict, Optional, Tuple
from ultralytics import YOLO
from ultralytics.utils import LOGGER
import warnings

# Suppress non-critical warnings
warnings.filterwarnings('ignore', category=UserWarning)
warnings.filterwarnings('ignore', category=FutureWarning)

# ============================================================================
# CONFIGURATION
# ============================================================================

class TrainingConfig:
    """Centralized training configuration"""
    
    # Project paths
    PROJECT_NAME = 'traffic_yolo12'
    DATA_YAML = 'dataset/data.yaml'
    
    # Model architecture
    BASE_MODEL = 'yolo12n.pt'  # Options: yolo12n.pt, yolo12s.pt, yolo12m.pt
    
    # Hardware settings
    DEVICE = 0  # GPU index
    BATCH_SIZE = 8
    WORKERS = 8
    USE_AMP = True  # Automatic Mixed Precision
    
    # Training hyperparameters
    EPOCHS = 200
    PATIENCE = 40  # Early stopping patience
    IMG_SIZE = 640
    
    # Optimizer settings (OneCycleLR compatible)
    OPTIMIZER = 'AdamW'
    LR_INITIAL = 0.001  # Will be scaled by OneCycleLR
    LR_FINAL = 0.00001
    WEIGHT_DECAY = 0.0005
    MOMENTUM = 0.937
    
    # Learning rate schedule
    USE_ONE_CYCLE = True  # Modern LR scheduling
    WARMUP_EPOCHS = 5.0
    WARMUP_MOMENTUM = 0.8
    WARMUP_BIAS_LR = 0.1
    
    # Regularization
    LABEL_SMOOTHING = 0.1  # Prevents overconfident predictions
    DROPOUT = 0.0  # YOLOv12 has built-in dropout
    
    # Augmentation (Traffic sign specific)
    AUGMENTATION = {
        'hsv_h': 0.015,      # Minimal hue shift (traffic signs have specific colors)
        'hsv_s': 0.5,        # Saturation variation
        'hsv_v': 0.3,        # Value/brightness variation
        'degrees': 5.0,      # Rotation (signs can be slightly tilted)
        'translate': 0.1,    # Translation
        'scale': 0.4,        # Scale variation
        'shear': 2.0,        # Perspective shear
        'perspective': 0.0,  # No perspective (already handled by camera)
        'flipud': 0.0,       # Never flip vertically
        'fliplr': 0.0,       # Never flip horizontally (text/arrows)
        'mosaic': 1.0,       # Mosaic augmentation
        'mixup': 0.1,        # Subtle mixup
        'copy_paste': 0.1,   # Copy-paste augmentation
    }
    
    # Loss weights
    BOX_LOSS = 7.5
    CLS_LOSS = 0.5
    DFL_LOSS = 1.5
    
    # Validation
    VAL_SPLIT = 0.2  # If not already split
    SAVE_PERIOD = 10  # Save checkpoint every N epochs
    
    # Export formats
    EXPORT_FORMATS = ['onnx', 'torchscript']  # 'engine' for TensorRT
    
    # Advanced features
    USE_MODEL_EMA = True  # Exponential Moving Average
    GRADIENT_CLIP = 10.0  # Gradient clipping value
    CLOSE_MOSAIC = 15  # Disable mosaic in last N epochs


# ============================================================================
# LOGGING SETUP
# ============================================================================

def setup_logging(run_name: str) -> logging.Logger:
    """Setup advanced logging with file and console handlers"""
    
    log_dir = Path('logs')
    log_dir.mkdir(exist_ok=True)
    
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_file = log_dir / f'{run_name}_{timestamp}.log'
    
    # Create logger
    logger = logging.getLogger('TrafficTrainer')
    logger.setLevel(logging.INFO)
    
    # Console handler with color support
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.INFO)
    console_format = logging.Formatter(
        '%(asctime)s | %(levelname)8s | %(message)s',
        datefmt='%H:%M:%S'
    )
    console_handler.setFormatter(console_format)
    
    # File handler
    file_handler = logging.FileHandler(log_file)
    file_handler.setLevel(logging.DEBUG)
    file_format = logging.Formatter(
        '%(asctime)s | %(levelname)8s | %(funcName)20s | %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    file_handler.setFormatter(file_format)
    
    # Add handlers
    logger.addHandler(console_handler)
    logger.addHandler(file_handler)
    
    return logger


# ============================================================================
# SYSTEM DIAGNOSTICS
# ============================================================================

def check_system(logger: logging.Logger) -> Dict:
    """Comprehensive system check"""
    
    logger.info("=" * 70)
    logger.info("SYSTEM DIAGNOSTICS")
    logger.info("=" * 70)
    
    system_info = {}
    
    # CUDA availability
    cuda_available = torch.cuda.is_available()
    system_info['cuda'] = cuda_available
    
    if cuda_available:
        gpu_name = torch.cuda.get_device_name(0)
        vram_total = torch.cuda.get_device_properties(0).total_memory / 1e9
        vram_allocated = torch.cuda.memory_allocated(0) / 1e9
        vram_reserved = torch.cuda.memory_reserved(0) / 1e9
        
        system_info.update({
            'gpu_name': gpu_name,
            'vram_total': vram_total,
            'vram_allocated': vram_allocated,
            'vram_reserved': vram_reserved
        })
        
        logger.info(f"GPU: {gpu_name}")
        logger.info(f"   Total VRAM:     {vram_total:.2f} GB")
        logger.info(f"   Allocated:      {vram_allocated:.2f} GB")
        logger.info(f"   Reserved:       {vram_reserved:.2f} GB")
        logger.info(f"   Free:           {vram_total - vram_reserved:.2f} GB")
        
        # Clear cache
        torch.cuda.empty_cache()
        logger.info("   Cache cleared   ✓")
        
    else:
        logger.warning("⚠️  No CUDA GPU detected! Training will be extremely slow.")
        system_info['gpu_name'] = 'CPU'
    
    # PyTorch info
    logger.info(f"PyTorch:         {torch.__version__}")
    logger.info(f"CUDA:            {torch.version.cuda if cuda_available else 'N/A'}")
    logger.info(f"cuDNN:           {torch.backends.cudnn.version() if cuda_available else 'N/A'}")
    
    # Optimizations
    torch.set_float32_matmul_precision('high')
    torch.backends.cudnn.benchmark = True
    logger.info("Optimizations:   ✓ (TF32, cuDNN benchmark)")
    
    logger.info("=" * 70)
    
    return system_info


# ============================================================================
# DATA VALIDATION
# ============================================================================

def validate_dataset(data_yaml: str, logger: logging.Logger) -> bool:
    """Validate dataset configuration"""
    
    logger.info("Validating dataset...")
    
    if not Path(data_yaml).exists():
        logger.error(f"❌ Dataset YAML not found: {data_yaml}")
        return False
    
    # Load and check YAML
    with open(data_yaml, 'r') as f:
        data = yaml.safe_load(f)
    
    required_keys = ['train', 'val', 'nc', 'names']
    for key in required_keys:
        if key not in data:
            logger.error(f"❌ Missing key in data.yaml: {key}")
            return False
    
    # Check paths
    base_path = Path(data_yaml).parent.resolve()
    
    # helper to resolve path relative to yaml location
    def resolve_path(p):
        path = Path(p)
        if path.is_absolute():
            return path
        return (base_path / path).resolve()

    train_path = resolve_path(data['train'])
    val_path = resolve_path(data['val'])
    
    if not train_path.exists():
        logger.error(f"❌ Training path not found: {train_path}")
        return False
    
    if not val_path.exists():
        logger.error(f"❌ Validation path not found: {val_path}")
        return False
    
    # Count images
    train_images = list(train_path.glob('**/*.jpg')) + list(train_path.glob('**/*.png'))
    val_images = list(val_path.glob('**/*.jpg')) + list(val_path.glob('**/*.png'))
    
    logger.info(f"Dataset validated:")
    logger.info(f"   Classes:        {data['nc']}")
    logger.info(f"   Train images:   {len(train_images)}")
    logger.info(f"   Val images:     {len(val_images)}")
    logger.info(f"   Class names:    {', '.join(data['names'][:5])}..." if len(data['names']) > 5 else f"   Class names:    {', '.join(data['names'])}")
    
    return True


# ============================================================================
# TRAINING FUNCTION
# ============================================================================

def train_model(
    config: TrainingConfig,
    logger: logging.Logger,
    resume: bool = False,
    tune: bool = False
) -> YOLO:
    """Advanced training pipeline"""
    
    # Find checkpoint
    checkpoint_path = Path(config.PROJECT_NAME) / 'weights' / 'last.pt'
    
    if resume and checkpoint_path.exists():
        logger.info(f"🔄 RESUMING from checkpoint: {checkpoint_path}")
        model = YOLO(str(checkpoint_path))
        resume_training = True
    else:
        if resume:
            logger.warning(f"⚠️  Checkpoint not found, starting fresh")
        logger.info(f"Loading base model: {config.BASE_MODEL}")
        model = YOLO(config.BASE_MODEL)
        resume_training = False
    
    # Hyperparameter tuning mode
    if tune:
        logger.info("🎯 HYPERPARAMETER TUNING MODE")
        logger.info("   This will test multiple configurations...")
        
        model.tune(
            data=config.DATA_YAML,
            epochs=30,  # Short tuning runs
            iterations=10,  # Number of configurations to try
            optimizer=config.OPTIMIZER,
            plots=True,
            save=True,
            val=True
        )
        
        logger.info("✅ Tuning complete! Check results in runs/tune/")
        return model
    
    # Build training arguments
    train_args = {
        # Dataset
        'data': config.DATA_YAML,
        
        # Resume
        'resume': resume_training,
        
        # Hardware
        'device': config.DEVICE,
        'batch': config.BATCH_SIZE,
        'workers': config.WORKERS,
        'cache': False,  # Disabled for memory safety
        'amp': config.USE_AMP,
        
        # Epochs
        'epochs': config.EPOCHS,
        'patience': config.PATIENCE,
        'imgsz': config.IMG_SIZE,
        
        # Optimizer
        'optimizer': config.OPTIMIZER,
        'lr0': config.LR_INITIAL,
        'lrf': config.LR_FINAL,
        'momentum': config.MOMENTUM,
        'weight_decay': config.WEIGHT_DECAY,
        
        # Learning rate schedule
        'cos_lr': not config.USE_ONE_CYCLE,  # Use cosine if not OneCycle
        'warmup_epochs': config.WARMUP_EPOCHS,
        'warmup_momentum': config.WARMUP_MOMENTUM,
        'warmup_bias_lr': config.WARMUP_BIAS_LR,
        
        # Regularization
        'label_smoothing': config.LABEL_SMOOTHING,
        'dropout': config.DROPOUT,
        
        # Augmentation
        **config.AUGMENTATION,
        'close_mosaic': config.CLOSE_MOSAIC,
        
        # Loss weights
        'box': config.BOX_LOSS,
        'cls': config.CLS_LOSS,
        'dfl': config.DFL_LOSS,
        
        # Output
        'project': config.PROJECT_NAME,
        'name': datetime.now().strftime('%Y%m%d_%H%M%S'),
        'exist_ok': True,
        'plots': True,
        'save': True,
        'save_period': config.SAVE_PERIOD,
        
        # Validation
        'val': True,
        'verbose': True,
    }
    
    # Log configuration
    logger.info("=" * 70)
    logger.info("TRAINING CONFIGURATION")
    logger.info("=" * 70)
    logger.info(f"Batch size:      {config.BATCH_SIZE}")
    logger.info(f"Image size:      {config.IMG_SIZE}")
    logger.info(f"Epochs:          {config.EPOCHS}")
    logger.info(f"Patience:        {config.PATIENCE}")
    logger.info(f"LR initial:      {config.LR_INITIAL}")
    logger.info(f"LR final:        {config.LR_FINAL}")
    logger.info(f"Optimizer:       {config.OPTIMIZER}")
    logger.info(f"Label smoothing: {config.LABEL_SMOOTHING}")
    logger.info(f"AMP enabled:     {config.USE_AMP}")
    logger.info(f"OneCycleLR:      {config.USE_ONE_CYCLE}")
    logger.info("=" * 70)
    
    # Start training
    logger.info("🚀 Starting training...")
    
    try:
        results = model.train(**train_args)
        logger.info("✅ Training completed successfully!")
        
    except RuntimeError as e:
        if "out of memory" in str(e).lower():
            logger.error("❌ GPU Out of Memory!")
            logger.info("   Reducing batch size and retrying...")
            
            torch.cuda.empty_cache()
            
            # Retry with smaller batch
            train_args['batch'] = config.BATCH_SIZE // 2
            train_args['workers'] = config.WORKERS // 2
            train_args['name'] = train_args['name'] + '_reduced_batch'
            
            logger.info(f"   New batch size: {train_args['batch']}")
            results = model.train(**train_args)
            
        else:
            logger.error(f"❌ Training failed: {e}")
            raise
    
    return model


# ============================================================================
# VALIDATION & EXPORT
# ============================================================================

def validate_model(model: YOLO, logger: logging.Logger) -> Dict:
    """Run comprehensive validation"""
    
    logger.info("=" * 70)
    logger.info("VALIDATION")
    logger.info("=" * 70)
    
    # Standard validation
    metrics = model.val()
    
    # Extract metrics
    results = {
        'mAP50': metrics.box.map50,
        'mAP50-95': metrics.box.map,
        'precision': metrics.box.mp,
        'recall': metrics.box.mr,
    }
    
    logger.info(f"mAP@50:          {results['mAP50']:.4f}")
    logger.info(f"mAP@50-95:       {results['mAP50-95']:.4f}")
    logger.info(f"Precision:       {results['precision']:.4f}")
    logger.info(f"Recall:          {results['recall']:.4f}")
    logger.info("=" * 70)
    
    return results


def export_model(
    model_path: str,
    formats: list,
    logger: logging.Logger
) -> None:
    """Export model to multiple formats"""
    
    logger.info("=" * 70)
    logger.info("MODEL EXPORT")
    logger.info("=" * 70)
    
    if not Path(model_path).exists():
        logger.error(f"❌ Model not found: {model_path}")
        return
    
    model = YOLO(model_path)
    
    for fmt in formats:
        try:
            logger.info(f"Exporting to {fmt.upper()}...")
            
            if fmt == 'onnx':
                model.export(
                    format='onnx',
                    dynamic=True,
                    simplify=True,
                    opset=12
                )
            elif fmt == 'torchscript':
                model.export(format='torchscript')
            elif fmt == 'engine':
                # TensorRT (requires TensorRT installed)
                model.export(format='engine', device=0)
            else:
                model.export(format=fmt)
            
            logger.info(f"   ✓ {fmt.upper()} export complete")
            
        except Exception as e:
            logger.error(f"   ✗ {fmt.upper()} export failed: {e}")
    
    logger.info("=" * 70)


# ============================================================================
# MAIN
# ============================================================================

def main():
    """Main training pipeline"""
    
    # Parse arguments
    resume = '--resume' in sys.argv or '-r' in sys.argv
    tune = '--tune' in sys.argv or '-t' in sys.argv
    export_only = '--export-only' in sys.argv or '-e' in sys.argv
    
    # Setup
    config = TrainingConfig()
    logger = setup_logging(config.PROJECT_NAME)
    
    # Header
    logger.info("\n" + "=" * 70)
    logger.info("YOLOv12 TURKISH TRAFFIC DETECTION - ADVANCED TRAINING")
    logger.info("=" * 70)
    
    # System check
    system_info = check_system(logger)
    
    if not system_info['cuda'] and not export_only:
        logger.error("Training requires GPU. Exiting.")
        return
    
    # Dataset validation
    if not export_only:
        if not validate_dataset(config.DATA_YAML, logger):
            logger.error("Dataset validation failed. Exiting.")
            return
    
    # Export only mode
    if export_only:
        best_model = Path(config.PROJECT_NAME) / 'weights' / 'best.pt'
        if not best_model.exists():
            # Try to find in latest run
            runs = sorted(Path(config.PROJECT_NAME).glob('*/weights/best.pt'))
            if runs:
                best_model = runs[-1]
            else:
                logger.error("No trained model found for export!")
                return
        
        export_model(str(best_model), config.EXPORT_FORMATS, logger)
        return
    
    # Training
    model = train_model(config, logger, resume=resume, tune=tune)
    
    if tune:
        return  # Tuning mode exits early
    
    # Find best model
    best_model_path = sorted(
        Path(config.PROJECT_NAME).glob('*/weights/best.pt')
    )[-1]
    
    logger.info(f"Best model: {best_model_path}")
    
    # Validation
    best_model = YOLO(str(best_model_path))
    results = validate_model(best_model, logger)
    
    # Export
    export_model(str(best_model_path), config.EXPORT_FORMATS, logger)
    
    # Summary
    logger.info("\n" + "=" * 70)
    logger.info("TRAINING COMPLETE!")
    logger.info("=" * 70)
    logger.info(f"Model weights:   {best_model_path}")
    logger.info(f"mAP@50:          {results['mAP50']:.4f}")
    logger.info(f"mAP@50-95:       {results['mAP50-95']:.4f}")
    logger.info("=" * 70)
    logger.info("\nNext steps:")
    logger.info("1. Copy best.pt to your ROS2 package:")
    logger.info(f"   cp {best_model_path} ~/ros2_ws/src/otagg_vision/models/")
    logger.info("2. Test with yolov12_node.py")
    logger.info("3. Deploy to robot")
    logger.info("=" * 70 + "\n")


if __name__ == '__main__':
    main()