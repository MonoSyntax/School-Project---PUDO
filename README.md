# OTAGG Otonom Araç Projesi - Kapsamlı Kurulum, Mimari ve Teknik Analiz

**Son Güncelleme:** 24 Ocak 2026

---

## İçindekiler

1. [Sistem Gereksinimleri ve Donanım Analizi](#1-sistem-gereksinimleri-ve-donanım-analizi)
2. [Ön Gereksinimler ve Sistem Hazırlığı](#2-ön-gereksinimler-ve-sistem-hazırlığı)
3. [ROS 2 Humble Kurulumu: Teknik Detaylar](#3-ros-2-humble-kurulumu-teknik-detaylar)
4. [Gazebo Harmonic: Yeni Nesil Simülasyon](#4-gazebo-harmonic-yeni-nesil-simülasyon)
5. [ros_gz Köprüsü: İletişim Katmanı](#5-ros_gz-köprüsü-iletişim-katmanı)
6. [Derin Öğrenme Altyapısı (CUDA & YOLO)](#6-derin-öğrenme-altyapısı-cuda--yolo)
7. [Proje Kurulumu ve Derleme (Adım Adım)](#7-proje-kurulumu-ve-derleme-adım-adım)
8. [Navigasyon Mimarisi (Nav2 & Algoritmalar)](#8-navigasyon-mimarisi-nav2--algoritmalar)
9. [Görüntü İşleme ve Karar Verme Mekanizması](#9-görüntü-işleme-ve-karar-verme-mekanizması)
10. [Sistemi Çalıştırma](#10-sistemi-çalıştırma)
11. [YOLO Model Eğitimi](#11-yolo-model-eğitimi)
12. [Sorun Giderme Kılavuzu](#12-sorun-giderme-kılavuzu)
13. [Proje Dosya Yapısı](#13-proje-dosya-yapısı)

---

## 1. Sistem Gereksinimleri ve Donanım Analizi

Otonom sürüş yazılımları, yüksek performanslı hesaplama ve gerçek zamanlı tepki süreleri gerektirir. Bu projenin donanım seçimlerinin teknik gerekçeleri şunlardır:

* **İşletim Sistemi: Ubuntu 22.04 LTS (Jammy Jellyfish)**
  * **Neden?** ROS 2 Humble sürümü, sadece Ubuntu 22.04 üzerinde "Tier 1" destek sunar. Bu, binary paketlerin (apt ile kurulanlar) doğrudan bu işletim sistemi için derlendiği ve en kararlı çalıştığı anlamına gelir.

* **GPU: NVIDIA GeForce RTX Serisi (Min. 6GB VRAM)**
  * **Neden NVIDIA?** Proje, nesne tespiti için YOLOv12 ve PyTorch kullanır. Bu kütüphaneler, matris çarpımı işlemlerini CPU yerine GPU üzerinde yapmak için NVIDIA'nın **CUDA (Compute Unified Device Architecture)** teknolojisini kullanır.
  * **Performans Farkı:** CPU üzerinde YOLOv12 modeli 1-5 FPS verirken, CUDA destekli bir RTX 3060 üzerinde 40-60+ FPS verir. Otonom bir aracın 60km/s hızla giderken saniyede 1 kare işlemesi kaza kaçınılmaz demektir.
  * **AMD Durumu:** AMD kartlar (ROCm) teorik olarak desteklense de, kurulum zorluğu ve sürücü uyumsuzlukları nedeniyle önerilmez.

* **RAM: Minimum 16GB, Önerilen 32GB**
  * **Neden?** Gazebo simülasyonu (yaklaşık 4-6GB), Rviz2 (1-2GB), Nav2 Costmap'leri (1-2GB) ve yüklenen YOLO modelleri (2-4GB VRAM taşması durumunda RAM'e geçer) aynı anda çalıştığında bellek kullanımı hızla 16GB sınırına dayanır.

---

## 2. Ön Gereksinimler ve Sistem Hazırlığı

### Repo Klonlama

```bash
git clone https://github.com/MonoSyntax/School-Project---PUDO.git ros2_ws
cd ros2_ws
```

### Sistem Güncellemesi ve Temel Araçlar

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y git curl wget build-essential cmake python3-pip python3-venv software-properties-common lsb-release gnupg
```

* **`build-essential` & `cmake`**: C++ tabanlı ROS paketlerini (örneğin `ros_gz_bridge`) kaynak koddan derlemek için zorunludur.

---

## 3. ROS 2 Humble Kurulumu: Teknik Detaylar

### Dil ve Locale Ayarları

ROS 2, topic üzerinden veri gönderirken katı bir UTF-8 standardı uygular. Sistem dili farklı (örn. Türkçe ISO-8859-9) ayarlanırsa, Python scriptleri topic verisini okurken çöker.

```bash
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### ROS 2 Paketleri ve Nedenleri

1. **Repo Ekleme:**

```bash
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

1. **Kurulum:**

```bash
sudo apt update
sudo apt install ros-humble-desktop -y
```

* **`ros-humble-desktop`**: Sadece çekirdek (`ros-base`) değil, görselleştirme araçlarını (`rviz2`, `rqt`) içerir. Geliştirme ortamı için şarttır.

1. **Kritik Navigasyon Paketleri:**

  ```bash
    sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-tf2-tools \
    ros-humble-tf-transformations
  ```

* **`robot_localization`**: Tekerlek enkoderlerinden gelen hız verisi (Odometri) zamanla kayar (drift). Bu paket, IMU (İvmeölçer/Jiroskop) verisini kullanarak bu hatayı **Extended Kalman Filter (EKF)** ile düzeltir.
* **`slam-toolbox`**: Haritalama için kullanılır. Gmapping'e göre daha modern, graf tabanlı bir SLAM algoritmasıdır ve büyük haritalarda daha az kaynak tüketir.

### Ortam Kurulumu

```bash
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

---

## 4. Gazebo Harmonic Kurulumu

```bash
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install gz-harmonic -y
```

---

## 5. ros_gz Köprüsü: İletişim Katmanı

ROS 2 ile Gazebo simulasyonu doğrudan haberleşemez. `ros_gz_bridge` paketi bu iki sistem arasındaki iletişimi sağlar.

**Neden Kaynak Koddan Derliyoruz?**
Ubuntu depolarındaki hazır `ros-humble-ros-gz` paketi genellikle eski Gazebo sürümleri (Fortress) içindir. Kaynaktan derlemek derlenen paketin sistemimizle uyumlu olmasını sağlar.

```bash
# Workspace hazırlığı
mkdir -p ~/pkgs_ws/src
cd ~/pkgs_ws/src
git clone https://github.com/gazebosim/ros_gz.git -b humble

# Bağımlılıkları kur (rosdep)
cd ~/pkgs_ws
export GZ_VERSION=harmonic
rosdep install -r --from-paths src -i -y --rosdistro humble

# Derle
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Ortama ekle
echo "source ~/pkgs_ws/install/setup.bash" >> ~/.bashrc
echo "export GZ_VERSION=harmonic" >> ~/.bashrc
```

---

## 6. Derin Öğrenme Altyapısı (CUDA & YOLO)

### Python Kütüphaneleri

```bash
pip3 install ultralytics opencv-python pillow 'numpy<2'
```

> [!WARNING]
> **NumPy Sürüm Uyarısı (`numpy<2`)**: ROS 2 Humble'ın `cv_bridge` kütüphanesi, NumPy 1.x API'sine bağımlıdır. Eğer `numpy` 2.0 veya üzeri kurulursa, `yolov12_node.py` çalıştırıldığında **"AttributeError: _ARRAY_API not found"** hatası alınır ve program çöker. Bu yüzden sürüm 2'den küçük olmaya zorlanmalıdır.

### CUDA 13.x

Sisteminizde NVIDIA sürücüsü kurulu olmalıdır (`nvidia-smi` komutu ile kontrol edin). PyTorch, sistemdeki CUDA sürümüne uygun kurulmalıdır.

---

## 7. Proje Kurulumu ve Derleme (Adım Adım)

Proje workspace'i hazırlandıktan sonra, aşağıdaki adımları sırasıyla uygulayarak kurulumu tamamlayın.

### Adım 3: ROS Bağımlılıklarını Kurun

Bu komut, `package.xml` dosyalarındaki `<depend>` etiketlerini okuyarak eksik sistem paketlerini otomatik kurar.

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### Adım 4: Workspace'i Derleyin

Projeyi derlemek için `colcon` aracını kullanıyoruz.

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

* **`--symlink-install`**: Derleme sırasında dosyaları kopyalamak yerine sembolik link oluşturur. Bu sayede Python scriptlerinde veya launch dosyalarında yaptığınız değişiklikler için tekrar derleme yapmanıza gerek kalmaz, anında aktif olur.

### Adım 5: Workspace'i Source Edin

Derlenen paketlerin terminal tarafından görünmesi için:

```bash
source ~/ros2_ws/install/setup.bash
```

### Adım 6: Kalıcı Shell Ayarları

Her yeni terminal açtığınızda `source` komutunu yazmamak için `.bashrc` dosyasına ekleyin:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 8. Navigasyon Mimarisi (Nav2 & Algoritmalar)

Bu proje, standart Nav2 yığınını Robotaksi görevlerine özel olarak modifiye etmiştir. Konfigürasyon dosyası: `src/navigation_2025/config/keepout_nav2_params.yaml`.

### A. Planlayıcı (Planner): `SmacPlannerHybrid`

* **Nedir?**: Hybrid A* algoritmasını kullanan, kinematik kısıtlamaları (dönüş yarıçapı) dikkate alan bir planlayıcıdır.
* **Neden Seçildi?**: Standart `NavFn` (Dijkstra/A*) robotu nokta gibi kabul eder ve olduğu yerde dönebildiğini varsayar. Bizim aracımız ise **Ackermann (Araba)** direksiyon sistemine sahiptir, olduğu yerde dönemez. `SmacPlannerHybrid`, aracın minimum dönüş yarıçapını (`minimum_turning_radius: 4.5m`) hesaba katarak, gerekirse ileri-geri manevralar (Reeds-Shepp eğrileri) yaparak yol çizer.

### B. Kontrolcü (Controller): `RegulatedPurePursuitController`

* **Nedir?**: Yolu takip etmek için direksiyon açısını ve hızı hesaplayan algoritma.
* **Neden Seçildi?**: Araba benzeri robotlar için endüstri standardıdır. Yoldaki eğriliğe göre hızı otomatik düşürür (`use_regulated_linear_velocity_scaling: true`), böylece virajlarda savrulmayı önler.

### C. Costmap Katmanları (Maliyet Haritası)

Robot dünyayı katmanlar halinde algılar:

1. **Static Layer**: Önceden haritalanmış duvarlar ve kaldırımlar.
2. **Obstacle Layer**: Lidar'dan gelen anlık engeller (yaya, diğer araçlar).
3. **Inflation Layer**: Engellerin etrafına "tehlike çemberi" çizer. `inflation_radius: 1.5m` olarak ayarlanmıştır.
4. **Lane Guidance Layer (Özel Eklenti)**:
    * **Dosya**: `src/navigation_2025/src/lane_guidance_layer.cpp`
    * **İşlevi**: Kamera şeritleri algıladığında, bu şeritleri sanal bir duvara dönüştürmek yerine "yüksek maliyetli alan" (`lane_cost: 150`) olarak işaretler.
    * **Mantık**: Robot şeridin üstüne basabilir (çarpışma değildir) ama basmamayı tercih eder.
    * **Temporal Decay (Zamansal Sönümleme)**: Algılanan şerit verileri `max_point_age: 0.5s` saniye sonra haritadan silinir. Bu, eski/hatalı algılamaların haritada kalıcı "hayalet duvarlar" oluşturmasını engeller.

### D. Behavior Tree (Davranış Ağacı)

* **Dosya**: `src/navigation_2025/Behavior_tree.xml`
* **Yapı**: `PipelineSequence` kullanır. Önce yolu hesaplar (`ComputePathThroughPoses`), sonra yolu takip eder (`FollowPath`).
* **Kurtarma (Recovery)**: Eğer robot sıkışırsa sırayla şunları dener:
    1. Costmap'leri temizle (hayalet engelleri sil).
    2. Bekle.
    3. Geri git (`BackUp`).

---

## 9. Görüntü İşleme ve Karar Verme Mekanizması

### A. Nesne Tespiti: `yolov12_node.py`

Bu node, otonom aracın "gözü" olarak görev yapar ve en güncel YOLOv12 mimarisini kullanır.

* **Giriş**: `/camera_compressed` topic'inden sıkıştırılmış (compressed) görüntü akışını alır.

* **Model**: `otagg_vision/models/` dizinindeki YOLOv12 modelini (`best.pt`) yükler. CUDA destekli GPU üzerinde çalışarak yüksek FPS değerlerine ulaşır.

* **Özellikler**:

  * **Mesafe Tahmini**: Tespit edilen nesnelerin bounding box yüksekliğine dayalı basit bir heuristic ile araca olan mesafesini (metre cinsinden) tahmin eder.

  * **Sınıf Eşleştirme (Class Mapping)**: Türkçe eğitilmiş etiketleri (örn: `kirmizi_isik`, `dur`) sistemin anlayacağı İngilizce etiketlere (`red`, `stop_sign`) çevirir.

  * **FPS Limitleme**: İşlem yükünü dengelemek için kare hızını (`fps_limit`) sınırlandırabilir.

  * **Kritik Uyarılar**: Yaya, kırmızı ışık veya dur tabelası gibi kritik durumlarda `/traffic/alerts` üzerinden anlık uyarı metinleri yayınlar.

* **Çıkışlar**:

  * `/traffic_signs`: Tespit edilen nesnelerin sınıfı, konumu (bbox), güven skoru ve tahmini mesafesini içeren yapılandırılmış veri (`TrafficSignArray`).
  * `/traffic/annotated/compressed`: Üzerine kutucuklar ve bilgiler çizilmiş görselleştirme görüntüsü.
  * `/traffic/alerts`: Operatör için acil durum uyarı mesajları.

### B. Trafik Yöneticisi: `traffic_state_manager_node.py`

Bu node, YOLO'dan gelen ham algılamaları analiz eder ve kararlı bir "Trafik Durumu" oluşturur.

* **Görevi**: Anlık ve hatalı algılamaları (noise) filtreleyerek, navigasyon sistemine güvenilir komutlar gönderir.

* **Çalışma Mantığı**:

  * **Algılama Toplama (Aggregation)**: `/traffic_signs` üzerinden gelen verileri sınıflandırır.

  * **Durum Takibi (LightState)**: Trafik ışıklarının durumunu (Kırmızı, Sarı, Yeşil) bir sonlu durum makinesi (FSM) ile takip eder.

  * **Zaman Filtrelemesi (SignTracker)**: Bir nesnenin varlığını doğrulamak için "Zaman Penceresi" (`time_window_sec: 2.0s`) kullanır. Örneğin, kırmızı ışığın geçerli sayılması için son 2 saniyede en az 3 kez (`min_detections`) görülmesi gerekir.

* **Navigasyon Müdahalesi**:

  * Eğer **Kırmızı Işık** (< 20m) veya **Dur Tabelası** (< 10m) kesinleşirse;

  * `/traffic_state` mesajındaki `nav_override_state` değişkenini **1 (DUR)** olarak ayarlar.

  * Bu sinyal, `velocity_override_node` tarafından okunur ve robot frenlenir.

---

## 10. Sistemi Çalıştırma

Her komut yeni bir terminal sekmesinde çalıştırılmalıdır.

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

## 11. YOLO Model Eğitimi

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

* `dataset/data.yaml` dosyasının doğru görüntü yollarını içerdiğinden emin olun.
* Gerekli Python kütüphanelerini kurun (bkz. [Derin Öğrenme Altyapısı](#6-derin-öğrenme-altyapısı-cuda--yolo)).

**2. Eğitimi Başlatma:**
Terminali açın ve eğitim klasörüne gidin:

```bash
cd ~/ros2_ws/Training\ Folder
```

* **Sıfırdan Eğitim (Fresh Training):**

  ```bash
  python3 train_detection.py
  ```

* **Kaldığı Yerden Devam Etme (Resume):**
  Eğer eğitim yarıda kesildiyse son checkpoint'ten devam eder:

  ```bash
  python3 train_detection.py --resume
  ```

* **Hiperparametre Optimizasyonu (Tuning):**
  En iyi öğrenme oranı ve parametreleri bulmak için:

  ```bash
  python3 train_detection.py --tune
  ```

**3. Eğitim Sonrası:**

* Eğitim tamamlandığında en iyi model ağırlıkları şu konumda oluşturulur:
  `~/ros2_ws/Training Folder/traffic_yolo12/<tarih_saat>/weights/best.pt`

* **Modeli Kullanma:**
  Yeni eğitilen modeli sisteme dahil etmek için, `.pt` dosyasını vision paketine kopyalayın:

  ```bash
  cp traffic_yolo12/<tarih_saat>/weights/best.pt ~/ros2_ws/src/otagg_vision_2026/otagg_vision/models/yolo_traffic_best.pt
  ```

  *(Not: Node içerisindeki model yolu ayarını güncellemeyi unutmayın!)*

---

## 12. Sorun Giderme Kılavuzu

### 1. NumPy Uyumsuzluğu

**Hata:**

```py
AttributeError: _ARRAY_API not found
```

**Çözüm:**

```bash
pip3 uninstall numpy
pip3 install 'numpy<2'
```

### 2. ros_gz Köprü Hatası

**Hata:**

```py
[ERROR] [ros_gz_bridge]: Failed to create bridge
```

**Çözüm:**

* `ros_gz`'nin kaynak koddan derlendiğinden emin olun
* `GZ_VERSION=harmonic` ortam değişkenini kontrol edin
* Source sıralamasını kontrol edin: `source /opt/ros/humble/setup.bash` -> `source ~/pkgs_ws/install/setup.bash` -> `source ~/ros2_ws/install/setup.bash`

### 3. Costmap yüklenme sorunu

**Hata:**
Eğer Rviz'de costmap haritası eksik ise ve araç hareket etmiyorsa; costmap doğru yüklenmemiş olabilir.

**Çözüm:**
Simulasyonu yeniden başlatmak genellikle yeterli olacaktır. Gazebo'da "Play" butonuna basıldığından ve `use_sim_time: True` olduğundan emin olun.
