# Nav2 Parametre Açıklamaları - Türkçe Kılavuz

## Tamamlanan İşlemler

### 1. Dosya Çevirileri
✅ **CONTROLLER_PERFORMANCE_OPTIMIZATION.md** - Kontrolör Performans Optimizasyonu
- Tüm başlıklar ve açıklamalar Türkçe'ye çevrildi
- Teknik terimler için uygun Türkçe karşılıklar kullanıldı
- Kullanım talimatları ve sorun giderme rehberi çevrildi

✅ **README.md** - Paket Ana Bilgileri
- Kısa README'den kapsamlı Türkçe dokümantasyona dönüştürüldü
- Kurulum, kullanım ve özellik bilgileri eklendi
- Performans optimizasyonları ve harita bilgileri dahil edildi

### 2. Parametre Dosyası Açıklamaları
✅ **keepout_nav2_params.yaml** - Ana Navigasyon Parametreleri
Her parametre için detaylı Türkçe açıklamalar eklendi:

#### AMCL (Adaptive Monte Carlo Localization)
- **alpha1-5**: Hareket gürültü parametreleri - robotun hareket belirsizlikleri
- **max_particles/min_particles**: Parçacık sayıları - lokalizasyon hassasiyeti
- **pf_err/pf_z**: Parçacık filtresi ayarları - yakınsama parametreleri
- **z_hit/z_rand/z_max/z_short**: Lazer model parametreleri - sensör güvenilirliği
- **transform_tolerance**: Transform toleransı - koordinat dönüşüm hassasiyeti

#### BT Navigator (Behavior Tree Navigator)
- **bt_loop_duration**: Davranış ağacı döngü süresi
- **plugin_lib_names**: Tüm davranış modülleri tek tek açıklandı
- **default_server_timeout**: Sunucu zaman aşımı parametreleri

#### Controller Server
- **controller_frequency**: Kontrolör çalışma frekansı - performans optimizasyonu
- **FollowPath parametreleri**: 
  - **lookahead_dist**: İleri bakış mesafesi
  - **desired_linear_vel**: İstenen hız değerleri
  - **transform_tolerance**: Transform hassasiyeti
  - **use_cost_regulated_linear_velocity_scaling**: Maliyet tabanlı hız kontrolü

#### Local Costmap (Yerel Maliyet Haritası)
- **update_frequency/publish_frequency**: Güncelleme frekansları - performans ayarları
- **width/height**: Harita boyutları - hesaplama yükü optimizasyonu
- **inflation_radius**: Güvenlik alanı boyutu
- **obstacle_layer**: Engel algılama parametreleri
- **static_layer**: Statik harita ayarları
- **keepout_filter**: Yasak alan filtresi

#### Global Costmap (Global Maliyet Haritası)
- **track_unknown_space**: Bilinmeyen alan takibi
- **resolution**: Harita çözünürlüğü
- Yerel harita ile benzer parametreler, global ölçekte

#### Planner Server (Yol Planlayıcı)
- **SmacPlannerHybrid parametreleri**:
  - **tolerance**: Hedefe ulaşma hassasiyeti
  - **downsample_costmap**: Performans optimizasyonu
  - **max_iterations**: Maksimum hesaplama döngüsü
  - **motion_model_for_search**: Hareket modeli (Reeds-Shepp)
  - **minimum_turning_radius**: Minimum dönüş yarıçapı
  - **angle_quantization_bins**: Açı ayrıştırma sayısı

#### Smoother Server (Yol Yumuşatıcı)
- **path_downsampling_factor**: Yol alt örnekleme - performans
- **minimum_turning_radius**: Robotun fiziksel kısıtları
- **w_curve/w_smooth/w_cost**: Ağırlık parametreleri
- **optimizer**: Optimizasyon algoritması ayarları

#### Behavior Server (Davranış Sunucusu)
- **cycle_frequency**: Davranış döngü frekansı
- **behavior_plugins**: Mevcut davranışlar (spin, backup, wait vb.)
- **transform_timeout**: Transform gecikmesi toleransı

#### Velocity Smoother (Hız Yumuşatıcı)
- **smoothing_frequency**: Yumuşatma frekansı
- **max_velocity/min_velocity**: Hız sınırları
- **max_accel/max_decel**: İvme sınırları
- **deadband_velocity**: Ölü bölge hızları

### 3. Performans Optimizasyonları Açıklandı
Her optimizasyon için neden yapıldığı ve etkisi belirtildi:
- **Frekans azaltmaları**: CPU yükünü azaltmak için
- **Tolerans gevşetmeleri**: Gerçek zamanlı performans için
- **İterasyon azaltmaları**: Hesaplama süresini kısaltmak için
- **Boyut optimizasyonları**: Bellek kullanımını azaltmak için

### 4. Teknik Terimler Sözlüğü
Her teknik terim için Türkçe karşılık ve açıklama:
- **Costmap**: Maliyet Haritası
- **Inflation**: Şişirme (güvenlik alanı)
- **Raytrace**: Işın İzleme
- **Footprint**: Ayak İzi (robot şekli)
- **Lookahead**: İleri Bakış
- **Downsampling**: Alt Örnekleme
- **Tolerance**: Tolerans/Hoşgörü
- **Threshold**: Eşik Değeri

## Kullanım Faydaları

### 1. Eğitim ve Öğrenme
- Yeni geliştiriciler parametreleri kolayca anlayabilir
- Her parametrenin robota etkisi açıklanmış
- Performans optimizasyonlarının mantığı anlaşılabilir

### 2. Hızlı Konfigürasyon
- Hangi parametrenin ne işe yaradığı belli
- Performans sorunlarında hangi değerlerin değiştirileceği açık
- Sistem gereksinimlerine göre ayar yapma rehberi mevcut

### 3. Hata Ayıklama
- Problemli parametreler kolayca tespit edilebilir
- Performans sorunlarının kaynağı anlaşılabilir
- İzleme araçları ile gerçek zamanlı takip mümkün

### 4. Sistem Optimizasyonu
- Donanım kapasitesine göre ayar yapma rehberi
- CPU ve bellek kullanımını optimize etme yöntemleri
- Farklı robot türleri için adaptasyon klavuzu

Bu kapsamlı Türkçe dokümantasyon sayesinde ROS2 Nav2 sistemi daha anlaşılır ve kullanılabilir hale gelmiştir.
