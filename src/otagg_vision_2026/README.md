# OTAGG Vision 2026

ROS 2 Humble tabanlı görüntü işleme ve nesne tespiti paketi. YOLO modelleri ile trafik levhası, araç ve yaya tespiti yapılır.

## 📦 Paketler

| Paket | Açıklama |
|-------|----------|
| `otagg_vision` | Ana görüntü işleme ve YOLO tabanlı tespit node'ları |
| `otagg_vision_interfaces` | Özel mesaj ve servis tanımlamaları |

## 🚀 Özellikler

- **Trafik Levhası Tespiti**: 43 farklı trafik levhası sınıflandırması
- **Nesne Tespiti**: İnsan, bisiklet, motor, araba, otobüs ve kamyon tespiti
- **Gerçek Zamanlı İşleme**: CUDA destekli GPU hızlandırma
- **ROS 2 Entegrasyonu**: CompressedImage mesajları ile çalışır

## 📋 Gereksinimler

- ROS 2 Humble
- Python 3.10+
- CUDA destekli GPU (önerilir)
- NumPy < 2.0 (**Önemli!**)

### Python Bağımlılıkları

```bash
pip install torch torchvision ultralytics opencv-python pillow
pip install 'numpy<2'  # cv_bridge uyumluluğu için gerekli
```

## 🔧 Kurulum

### 1. Workspace'e klonlama

```bash
cd ~/ros2_ws/src
git clone https://github.com/OTAGG/otagg_vision_2026.git
```

### 2. Modelleri Ekleme

Modelleri `otagg_vision/models` dizinine kopyalayın veya taşıyın.

### 3. Derleme

```bash
cd ~/ros2_ws
colcon build --packages-select otagg_vision otagg_vision_interfaces
source install/setup.bash
```

## 🎯 Kullanım

### Kamera Node'u (Basit görüntüleme)

```bash
ros2 run otagg_vision camera_node.py
```

### YOLOv12 Tespit Node'u

```bash
ros2 run otagg_vision yolov12_node.py
```

## 📡 ROS 2 Arayüzleri

### Subscribed Topics

| Topic | Mesaj Tipi | Açıklama |
|-------|-----------|----------|
| `/camera_compressed` | `sensor_msgs/CompressedImage` | Sıkıştırılmış kamera görüntüsü |


## 🏷️ Desteklenen Trafik Levhaları

<details>
<summary>43 Sınıf Listesi (Tıkla)</summary>

- Hız sınırları: 20, 30, 50, 60, 70, 80, 100, 120 km/h
- Mecburi yön: Sağ, Sol, İleri, İleri-Sağ, İleri-Sol
- Yasaklar: Sollama yasak, Park yasak, Giriş yasak, Dönüş yasak
- Uyarılar: Dikkat, Kaygan yol, Yol çalışması, Gizli buzlanma
- Trafik ışıkları: Kırmızı, Sarı, Yeşil
- Diğer: Dur, Yol ver, Yaya geçidi, Park, Durak, Dönel kavşak

</details>

## ⚠️ Bilinen Sorunlar

### NumPy Uyumsuzluğu

```
AttributeError: _ARRAY_API not found
```

**Çözüm:** NumPy'ı 2.0'dan düşük bir sürüme indirin:

```bash
pip install 'numpy<2'
```
