# navigation_2025: TEKNOFEST için Gelişmiş ROS2 Nav2 Navigasyon Paketi

Bu paket, TEKNOFEST yarışma robotları için yüksek derecede optimize edilmiş bir ROS2 Nav2 tabanlı otonom navigasyon yığını sunar. Gelişmiş yasak bölge (keepout) yönetimi, sağlam costmap ve kontrolcü ayarları ile karmaşık ortamlarda yüksek performanslı ve güvenli gezinme sağlar.

---

## Klasör ve Dosya Yapısı

```
navigation_2025/
├── Behavior_tree.xml         # Davranış ağacı tanımı (BT)
├── CMakeLists.txt           # ROS2 CMake yapılandırması
├── package.xml              # ROS2 paket tanımı
├── config/
│   ├── keepout_nav2_params.yaml   # Tüm Nav2 parametreleri (yasak bölge, costmap, kontrolcü, vb.)
│   ├── ekf.yaml                  # EKF ve sensör füzyon parametreleri
│   └── ...
├── launch/
│   ├── keepout_full_nav2.launch.py   # Tüm navigasyon sistemi başlatıcı
│   ├── ekf_localization.launch.py    # EKF başlatıcı
│   └── ...
├── maps/
│   ├── teknofestObstacle.yaml        # Harita ve yasak bölge tanımları
│   └── ...
├── src/
│   └── nav_node.py                  # (Varsa) özel düğüm kodları
└── README.md
```

---

## Temel Scriptler ve Görevleri
- `src/nav_node.py`: Özel navigasyon veya kontrol düğümleriniz burada yer alır.
- `launch/keepout_full_nav2.launch.py`: Tüm navigasyon sistemi (Nav2, costmap, yasak bölge, vb.) tek komutla başlatılır.
- `config/keepout_nav2_params.yaml`: Tüm Nav2 parametreleri merkezi olarak burada tutulur.
- `Behavior_tree.xml`: Davranış ağacı (BT) ile görev akışı ve recovery stratejileri tanımlanır.

---

## Önemli Yapılandırma Dosyaları
- `config/keepout_nav2_params.yaml`: Costmap, kontrolcü, planner, yasak bölge ve diğer tüm Nav2 parametreleri.
- `config/ekf.yaml`: Sensör füzyonu ve lokalizasyon için EKF ayarları.
- `maps/teknofestObstacle.yaml`: Harita ve yasak bölge (keepout) alanlarının tanımı.
- `Behavior_tree.xml`: Navigasyon davranış ağacı ve recovery mantığı.

---

## Kodun Genişletilmesi ve Katkı
- Yeni bir kontrolcü, davranış ağacı veya costmap eklentisi eklemek için ilgili dosya ve dizinlere yeni script veya parametre ekleyin.
- `CMakeLists.txt` ve `package.xml` dosyalarında yeni script veya bağımlılıkları tanımlamayı unutmayın.
- Kodunuzu test etmek için kendi launch dosyanızı oluşturabilir veya mevcutları düzenleyebilirsiniz.

---

## Temel Özellikler
- **Yasak (Forbidden) Bölgeler:**
  - Nav2'nin `KeepoutFilter` özelliği ile robotun yasak alanlara kesinlikle girmesi engellenir.
  - Yasak bölgeler haritada tanımlanır ve hem lokal hem global costmap'te uygulanır.
  - Planlayıcıda `allow_unknown: false` ile bilinmeyen ve yasak alanlar engel olarak kabul edilir.
- **Costmap & Kontrolcü Optimizasyonu:**
  - Daha güvenli sınır davranışı için büyük `inflation_radius` ve azaltılmış `footprint`/`padding`.
  - Güvenli ve agresif yol takibi için ayarlanmış `RegulatedPurePursuitController`.
  - Çarpışma algılama aktif; hız ve lookahead parametreleri yarışma performansı için optimize edildi.
- **Performans & Güvenlik:**
  - Yüksek frekanslı kontrolcü ve costmap güncellemeleri ile hızlı tepki.
  - Hedef ve ilerleme kontrolcülerinde muhafazakâr ayarlar ile taşma veya sıkışma önlenir.
- **Kapsamlı Parametre Dokümantasyonu:**
  - Tüm ana parametreler `config/keepout_nav2_params.yaml` dosyasında (Türkçe ve İngilizce açıklamalı) belgelenmiştir.

---

## Hızlı Başlangıç
1. **Bağımlılıklar:**
   - ROS2 (Humble/Foxy ile test edildi)
   - Nav2 paketi
   - Simülasyon için Gazebo/IGN
2. **Derleme:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select navigation_2025
   source install/setup.bash
   ```
3. **Başlatma Örneği:**
   ```bash
   ros2 launch navigation_2025 keepout_full_nav2.launch.py
   ```
   - Simülasyon için `simulation_2025/launch/` altındaki dosyaları kullanın.
4. **RViz Görselleştirme:**
   - Tavsiye edilen RViz ayarı için `config/rviz2.rviz` dosyasını kullanın.

---

## Konfigürasyonun Öne Çıkanları
- **Yasak Bölgeler:**
  - Harita YAML dosyasında tanımlanır ve her iki costmap'te `keepout_filter` ile uygulanır.
  - Costmap eklentilerinde `filters: ["keepout_filter", "inflation_layer"]` kullanılır.
- **Costmap:**
  - `inflation_radius: 6.0` (geniş güvenlik marjı)
  - `footprint: [[-1.1, -0.6], [1.1, -0.6], [1.1, 0.6], [-1.1, 0.6]]`
  - `footprint_padding: 0.05`
- **Kontrolcü:**
  - `desired_linear_vel: 1.2` (yarışma hızı)
  - `lookahead_dist: 1.5`, `min_lookahead_dist: 0.7`, `max_lookahead_dist: 2.5`
  - `use_collision_detection: true`
  - `allow_reversing: true` (dar manevralar için)
- **Planlayıcı:**
  - `allow_unknown: false` (bilinmeyen/yasak alanları engel olarak kabul eder)
  - `motion_model_for_search: REEDS_SHEPP` (araba tipi kinematik)

---

## Sorun Giderme & En İyi Uygulamalar
- **Robot Yasak Bölgeden Çıkıyor:**
  - Her iki costmap'te `keepout_filter` etkin ve planlayıcıda `allow_unknown: false` olduğundan emin olun.
  - Daha sıkı sınır takibi için `inflation_radius` artırın ve `footprint`/`padding` azaltın.
- **CMake Hataları (Eksik Scriptler):**
  - Eksik script hatası alırsanız (örn. `anti_teleport_diagnostic.py`), `CMakeLists.txt` ve `package.xml` dosyalarındaki referansları kaldırın veya güncelleyin.
- **Navigasyon Performansı:**
  - Robot ve ortamınıza göre `desired_linear_vel`, `lookahead_dist` ve costmap güncelleme oranlarını ayarlayın.
  - Hata ayıklama için RViz'de costmap ve yasak bölgeleri görselleştirin.

---

## İleri Seviye Kullanım & Geliştirme
- Gelişmiş ROS2, Nav2 ve Gazebo geliştirme ipuçları için `.github/instructions/ros2.instructions.md` dosyasına bakın:
  - Özel davranış ağaçları
  - Çoklu robot koordinasyonu
  - SLAM ve sensör füzyonu
  - Kontrolcü ve eklenti geliştirme
  - Performans kıyaslama

---

## Dokümantasyon & Destek
- Tüm parametreler `config/keepout_nav2_params.yaml` dosyasında (Türkçe açıklamalı) belgelenmiştir.