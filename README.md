# OTAGG Otonom Araç Projesi - Kurulum Rehberi

Bu döküman, OTAGG otonom araç projesinin (Teknofest 2025/2026) kurulumu ve yapılandırması için kapsamlı bir rehber sunmaktadır.

---

## İçindekiler

1. [Ön Gereksinimler](#ön-gereksinimler)
2. [ROS 2 Humble Kurulumu](#ros-2-humble-kurulumu)
3. [Gazebo Harmonic Kurulumu](#gazebo-harmonic-kurulumu)
4. [ros_gz Köprüsü Kurulumu](#ros_gz-köprüsü-kurulumu)
5. [Python Bağımlılıkları](#python-bağımlılıkları)
6. [Sistemi Çalıştırma](#sistemi-çalıştırma)
7. [Sistem Mimarisi ve Çalışma Mantığı](#sistem-mimarisi-ve-çalışma-mantığı)
8. [YOLO Model Eğitimi](#yolo-model-eğitimi)
9. [Sorun Giderme](#sorun-giderme)
10. [Proje Dosya Yapısı](#proje-dosya-yapısı)

## Ön Gereksinimler

- [Ubuntu 22.04 İşletim Sistemi (Performans açısından Dual Boot önerilir)](https://releases.ubuntu.com/22.04/)
- [Python 3.10.2 (22.04 varsayılan Python sürümü)](https://www.python.org/downloads/release/python-3102/)
- NVIDIA CUDA destekli dGPU

### 0. Bu repo'yu klonlayın

```bash
git clone https://github.com/MonoSyntax/School-Project---PUDO.git
```

### 1. Sistem Güncellemesi

- İlk önce tüm sistem paketlerinin güncel olduğundan emin olun.

```bash
sudo apt update && sudo apt upgrade -y
```

### 2. Gerekli Araçların Kurulumu

- ROS 2 için gerekli araçları kurun.

```bash
sudo apt install -y \
    git \
    curl \
    wget \
    build-essential \
    cmake \
    python3-pip \
    python3-venv \
    software-properties-common \
    lsb-release \
    gnupg
```

---

## ROS 2 Humble Kurulumu

### Adım 1: ROS 2 Apt Deposunu Ekleyin

```bash
# ROS 2 Humble only supports UTF-8 locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ROS 2 GPG anahtarını ekleyin
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Depoyu ekleyin
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Adım 2: ROS 2 Humble Desktop'ı Kurun

```bash
sudo apt update
sudo apt install ros-humble-desktop -y
```

### Adım 3: Nav2 ve Ek Paketleri Kurun

- Proje için gerekli ROS 2 paketlerini kurun.

```bash
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-tf2-tools \
    ros-humble-tf-transformations \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-rviz2
```

### Adım 4: colcon ve rosdep Kurulumu

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep -y
sudo rosdep init
rosdep update
```

### Adım 5: Shell Ayarları

```bash
# .bashrc dosyasına ekleyin
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Gazebo Harmonic Kurulumu

### Adım 1: OSRF Deposunu Ekleyin

```bash
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
```

### Adım 2: Gazebo Harmonic'i Kurun

```bash
sudo apt update
sudo apt install gz-harmonic -y
```

## ros_gz Köprüsü Kurulumu

> ROS 2 Humble ve Gazebo Harmonic uyumluluğu için `ros_gz` paketinin **kaynak koddan derlenmesi** gerekmektedir. Binary paketler uyumlu değildir.

### Adım 1: Ayrı Bir Workspace Oluşturun

```bash
mkdir -p ~/pkgs_ws/src
cd ~/pkgs_ws/src
```

### Adım 2: ros_gz Kaynak Kodunu İndirin

```bash
git clone https://github.com/gazebosim/ros_gz.git -b humble
```

### Adım 3: Bağımlılıkları Kurun

- Bu adım sırasında başka işlemler yapmayın.

```bash
cd ~/pkgs_ws
export GZ_VERSION=harmonic
rosdep install -r --from-paths src -i -y --rosdistro humble
```

### Adım 4: Derleyin

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Adım 5: Shell Ayarları

```bash
# .bashrc dosyasına ekleyin (ROS 2 source'undan SONRA)
echo "source ~/pkgs_ws/install/setup.bash" >> ~/.bashrc
echo "export GZ_VERSION=harmonic" >> ~/.bashrc
source ~/.bashrc
```

---

## Python Bağımlılıkları

### CUDA/GPU Desteği

```bash
# NVIDIA CUDA Toolkit (zaten kurulu değilse)
# https://developer.nvidia.com/cuda-downloads adresinden indirin ve verilen adımları takip edin
# Proje CUDA 13.1 sürümü ile hazırlanmıştır. Diğer sürümler ile uyumluluk garantisi yoktur.
```

### PyTorch CUDA sürümü

- Verilen adımları takip ederek sisteminize uyumlu olan PyTorch sürümünü kurun.
[PyTorch](https://pytorch.org/get-started/locally/)

### Görüntü İşleme Bağımlılıkları

- Görüntü işleme için gerekli olan bağımlılıkları kurun.

```bash
pip3 install \
    ultralytics \
    opencv-python \
    pillow \
    'numpy<2'
```

> [!IMPORTANT]
> **NumPy Sürümü Kritik!** `cv_bridge` ile uyumluluk için NumPy **2.0'dan düşük** olmalıdır. Aksi takdirde şu hata alınır:
>
> ```py
> AttributeError: _ARRAY_API not found
> ```

### NumPy Sürümünü Kontrol Edin

```bash
python3 -c "import numpy; print(numpy.__version__)"
# Çıktı: 1.x.x olmalı (2.x.x değil)
```

---

### Repoyu klonlama

git clone <https://github.com/MonoSyntax/School-Project---PUDO.git>>

Workspace aşağıdaki paketleri içermelidir:

```md
ros2_ws/src/
├── navigation_2025/        # Nav2 tabanlı otonom navigasyon
├── otagg_vision_2026/      # YOLO tabanlı görüntü işleme
│   ├── otagg_vision/       # Ana görüntü işleme node'ları
│   └── otagg_vision_interfaces/  # Özel mesaj tanımlamaları
├── simulation_2025/        # Gazebo simülasyon ortamı
└── tests/                  # Test paketleri
```

### Adım 3: ROS Bağımlılıklarını Kurun

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### Adım 4: Workspace'i Derleyin

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

### Adım 5: Workspace'i Source Edin

```bash
source ~/ros2_ws/install/setup.bash
```

### Adım 6: Kalıcı Shell Ayarları

```bash
# .bashrc dosyasına ekleyin
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Sistemi Çalıştırma

### 1. Simülasyon Ortamı

```bash
# Gazebo dünyasını başlatın
ros2 launch simulation_2025 teknofest_IGN.launch.py
```

### 2. Navigasyon Sistemi

```bash
# Tam Nav2 stack'i (Simülasyon + Lokalizasyon + Navigasyon)
ros2 launch navigation_2025 bringup.launch.py
```

### 3. Görüntü İşleme

```bash
# YOLO trafik işaret tespit node'u
ros2 run otagg_vision yolov12_node.py
```

### 4. RViz2 ile Görselleştirme

```bash
ros2 run rviz2 rviz2
```

---

---

## Sistem Mimarisi ve Çalışma Mantığı

Bu proje, otonom sürüş görevlerini yerine getirmek için üç ana modülden oluşur: Navigasyon, Görüntü İşleme ve Simülasyon.

### 1. Navigasyon (Navigation 2)

Navigasyon sistemi, ROS 2 Navigation Stack (Nav2) üzerine kuruludur.

- **Lokalizasyon**: `robot_localization` paketi ile IMU ve Odometri verileri birleştirilir (EKF). `amcl` ile harita üzerindeki konum doğrulanır.
- **Planlama**:
  - **Global Planner**: Hedefe giden en kısa yolu hesaplar (Smac Planner vb.).
  - **Local Planner**: Dinamik engellerden kaçınarak aracı sürer (MPPI veya TEB Controller).
- **Costmap'ler**: Statik harita, lazer tarayıcıdan gelen engel verileri ve "yasaklı bölgeler" (Keepout zones) costmap katmanları olarak işlenir.

### 2. Görüntü İşleme (Computer Vision)

Görüntü işleme sistemi, trafik işaretlerini ve ışıklarını tespit etmek için YOLOv12 mimarisini kullanır.

- **Node**: `otagg_vision/scripts/yolov12_node.py`
- **Giriş**: Kamera görüntüleri (`/camera/image_raw`)
- **Çıkış**: Tespit edilen nesneler, sınıfları ve konumları (Bounding Boxes).
- **Entegrasyon**: Tespit edilen duraklar veya ışıklar, `traffic_state_manager` tarafından navigasyon sistemine hız limiti veya durma komutu olarak iletilir.

### 3. Simülasyon (Gazebo Harmonic)

Gerçek dünya testleri öncesinde algoritmaları doğrulamak için Gazebo Harmonic kullanılır.

- **ros_gz_bridge**: Gazebo ve ROS 2 arasındaki iletişimi sağlar.
  - Kamera görüntüleri ve Lidar verileri ROS 2'ye aktarılır.
  - ROS 2'den gelen hız komutları (`cmd_vel`) Gazebo'daki araca iletilir.

---

## YOLO Model Eğitimi

Proje kapsamında trafik işaretlerini tespit etmek için özel bir YOLOv12 modeli eğitilmektedir. Eğitim dosyaları ve araçları `Training Folder` dizininde bulunur.

### Eğitim Klasör Yapısı

```
Training Folder/
├── train_detection.py    # Ana eğitim scripti
├── dataset/             
│   ├── data.yaml        # Veriseti konfigürasyonu (Sınıflar ve yollar)
│   ├── train/           # Eğitim görselleri
│   └── val/             # Doğrulama görselleri
└── traffic_yolo12/      # Eğitim çıktıları (Loglar, Ağırlıklar, Grafikler)
```

### Modeli Eğitme

**1. Hazırlık:**

- `dataset/data.yaml` dosyasının doğru görüntü yollarını içerdiğinden emin olun.
- Gerekli Python kütüphanelerini kurun (bkz. [Python Bağımlılıkları](#python-bağımlılıkları)).

**2. Eğitimi Başlatma:**
Terminali açın ve eğitim klasörüne gidin:

```bash
cd ~/ros2_ws/Training\ Folder
```

- **Sıfırdan Eğitim (Fresh Training):**

  ```bash
  python3 train_detection.py
  ```

- **Kaldığı Yerden Devam Etme (Resume):**
  Eğer eğitim yarıda kesildiyse son checkpoint'ten devam eder:

  ```bash
  python3 train_detection.py --resume
  ```

- **Hiperparametre Optimizasyonu (Tuning):**
  En iyi öğrenme oranı ve parametreleri bulmak için:

  ```bash
  python3 train_detection.py --tune
  ```

**3. Eğitim Sonrası:**

- Eğitim tamamlandığında en iyi model ağırlıkları şu konumda oluşturulur:
  `~/ros2_ws/Training Folder/traffic_yolo12/<tarih_saat>/weights/best.pt`

- **Modeli Kullanma:**
  Yeni eğitilen modeli sisteme dahil etmek için, `.pt` dosyasını vision paketine kopyalayın:

  ```bash
  cp traffic_yolo12/<tarih_saat>/weights/best.pt ~/ros2_ws/src/otagg_vision_2026/otagg_vision/models/yolo_traffic_best.pt
  ```

  *(Not: Node içerisindeki model yolu ayarını güncellemeyi unutmayın!)*

---

## Sorun Giderme

### Sık Karşılaşılan Hatalar

#### 1. NumPy Uyumsuzluğu

**Hata:**

```py
AttributeError: _ARRAY_API not found
```

**Çözüm:**

```bash
pip3 uninstall numpy
pip3 install 'numpy<2'
```

---

#### 2. ros_gz Köprü Hatası

**Hata:**

```py
[ERROR] [ros_gz_bridge]: Failed to create bridge
```

**Çözüm:**

- `ros_gz`'nin kaynak koddan derlendiğinden emin olun
- `GZ_VERSION=harmonic` ortam değişkenini kontrol edin
- Source sıralamasını kontrol edin:

  ```bash
  source /opt/ros/humble/setup.bash # ros
  source ~/pkgs_ws/install/setup.bash  # ros_gz
  source ~/ros2_ws/install/setup.bash  # workspace
  ```

#### 3. Costmap yüklenme sorunu

**Hata:**

Eğer Rviz'de costmap haritası eksik ise, ve araç hareket etmiyorsa; costmap doğru yüklenmemiş olabilir.

**Çözüm:**

Simulasyonu yeniden başlatmak genellikle yeterli olacaktır.

## Ek Kaynaklar

| Kaynak | Açıklama |
|--------|----------|
| [ROS 2 Humble Dökümantasyonu](https://docs.ros.org/en/humble/) | Resmi ROS 2 rehberi |
| [Nav2 Dökümantasyonu](https://docs.nav2.org/) | Navigation 2 stack rehberi |
| [Gazebo Sim Dökümantasyonu](https://gazebosim.org/docs) | Yeni Gazebo dökümantasyonu |
| [Ultralytics YOLO](https://docs.ultralytics.com/) | YOLO model eğitimi ve kullanımı |

---

## Proje Dosya Yapısı

```

ros2_ws/
├── src/
│   ├── navigation_2025/
│   │   ├── config/
│   │   │   ├── keepout_nav2_params.yaml   # Ana Nav2 parametreleri
│   │   │   └── ekf.yaml                   # Sensör füzyon ayarları
│   │   ├── launch/
│   │   │   ├── bringup.launch.py          # Ana launch dosyası
│   │   │   ├── localization.launch.py     # AMCL + Harita
│   │   │   └── navigation.launch.py       # Nav2 Stack
│   │   ├── maps/
│   │   │   ├── teknofestObstacle.yaml     # Harita tanımı
│   │   │   └── keepout_*.yaml             # Yasak alan tanımları
│   │   └── Behavior_tree.xml              # Davranış ağacı
│   │
│   ├── otagg_vision_2026/
│   │   ├── otagg_vision/
│   │   │   ├── models/                    # YOLO modelleri
│   │   │   └── scripts/                   # Python node'ları
│   │   └── otagg_vision_interfaces/       # Özel mesajlar
│   │
│   └── simulation_2025/
│       ├── launch/                        # Simülasyon launch'ları
│       ├── models/                        # Gazebo modelleri
│       ├── urdf/                          # Robot tanımları
│       └── meshes/                        # 3D mesh dosyaları
│
├── build/                                 # Derleme çıktıları
├── install/                               # Kurulum dosyaları
└── log/                                   # Log dosyaları
```
