# OTAGG-01 SIMULATION PACKAGE

## GAZEBO HARMONIC KURULUMU

Ubuntu üzerinde bir terminal açın ve aşağıdaki kodları çalıştırın.

Birkaç eklenti kuracağız:
```bash
sudo apt-get update
sudo apt-get install curl lsb-release gnupg
```

Şimdi Gazebo Harmonic'i kuralım:
```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
```

İndirmeler bittiğinde Gazebo Harmonic kurulmuş olacak.

## ROS_GZ

Gazebo Harmonic ile ROS2 Humble arasındaki bağlantının sağlanabilmesi için ros_gz paketinin kurulması gerekmektedir. ROS2 Humble ile son sürüm Gazebo sürümleri arasından resmi olarak Ignition Fortress desteklenmektedir. Bu sebepten dolayı apt kullanılarak ros_gz paketi kurulduğunda Fortress destekli olarak kurulmaktadır. Bu kurulumu Gazebo Harmonic ile uyumlu hale getirmek için kaynaktan kurulum (build from source) yapmak gerekmektedir.


Kaynaktan kurulum yaparken Gazebo sürümünün belirtilmesi gerekmektedir:
```bash
export GZ_VERSION=harmonic
```

Kaynaktan kurulum yapılmasından dolayı paketin çalışma alanında izole olması önerilmektedir. Bundan dolayı home dizini içerisine ros2_ws'den farklı bir dizin oluşturmak gerekmektedir. Bu yapıldıktan sonra dizin içerisindeki src'ye ros_gz deposunun humble branch'ı klonlanmalıdır:
```bash
cd ~
mkdir -p ~/pkgs_ws/src
cd ~/pkgs_ws/src
git clone https://github.com/gazebosim/ros_gz.git -b humble
```

Kurulumu yapabilmek için ~/pkgs_ws dizini içerisine gelip rosdep ile bağımlılıkları kurmak ve colcon build ile kurulumu başlatmak gerekmektedir:

***BU ADIMI YAPARKEN SİSTEMDE HERHANGİ BAŞKA İŞLEMIN/PROGRAMIN AÇIK OLMAMASI GEREKMEKTEDİR***
```bash
cd ~/pkgs_ws
rosdep install -r --from-paths src -i -y --rosdistro humble
colcon build
```

Bütün bu işlemler bittikten sonra pkgs_ws'nin bashrc'den source yapılması gerekmektedir:
```bash
echo "source ~/pkgs_ws/install/setup.bash" >> ~/.bashrc
```

İşlemler sonucunda ros_gz paketi kurulumu tamamlanmış demektir.

