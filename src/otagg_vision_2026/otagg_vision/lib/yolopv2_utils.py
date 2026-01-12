import cv2
import torch
import torch.nn.functional as F
import numpy as np
import os

def select_device(device='', batch_size=None):
    # Cihaz seçimi (CPU veya CUDA)
    device = str(device).strip().lower().replace('cuda:', '').replace('none', '')  # to string, 'cuda:0' to '0'
    cpu = device == 'cpu'
    if cpu:
        os.environ['CUDA_VISIBLE_DEVICES'] = '-1'  # force torch.cuda.is_available() = False
    elif device:  # non-cpu device requested
        os.environ['CUDA_VISIBLE_DEVICES'] = device  # set environment variable
        assert torch.cuda.is_available(), f'CUDA unavailable, invalid device {device} requested'  # check availability

    cuda = not cpu and torch.cuda.is_available()
    return torch.device('cuda:0' if cuda else 'cpu')

def letterbox(img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
    # Görüntüyü modele uygun boyuta (kare veya dikdörtgen) getirmek için kenarlara gri dolgu ekler (resize + pad)
    shape = img.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:  # only scale down, do not scale up (for better test mAP)
        r = min(r, 1.0)

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
    elif scaleFill:  # stretch
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
    return img, ratio, (dw, dh)

def lane_line_mask(ll):
    # Modelden çıkan ham veriyi (logits) ikili maskeye (0 veya 1) çevirir.
    # ll: (Batch, 2, Height, Width) -> Genellikle şerit ve arka plan
    if ll.ndim == 3: # Eger batch boyutu yoksa ekle
        ll = ll.unsqueeze(0)
    
    # Argmax ile en yüksek olasılıklı sınıfı seç (Şerit mi değil mi?)
    ll_seg_mask = torch.argmax(ll, dim=1)
    
    # Tensor'dan numpy'a çevir
    ll_seg_mask = ll_seg_mask.squeeze().detach().cpu().numpy()
    
    return ll_seg_mask