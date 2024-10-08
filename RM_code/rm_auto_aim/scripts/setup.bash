#!/bin/bash

echo ">>> APT update" && sleep 1
sudo apt update

echo -e "\n\n>>> wget eigen3 ceres pip" && sleep 1
sudo apt install wget libeigen3-dev libceres-dev python3-pip -y

echo -e "\n\n>>> MindVision SDK" && sleep 1
if [ -d /usr/include/mindvision/ ]; then
    echo "mindvision-sdk already installed"
else
    echo ">>> start install mindvision-sdk"
    mkdir mindvision-sdk
    wget https://www.mindvision.com.cn/wp-content/uploads/2023/08/linuxSDK_V2.1.0.37.tar.gz -O mindvision-sdk/SDK.tar.gz
    tar -zxvf mindvision-sdk/SDK.tar.gz --directory=mindvision-sdk
    sed -i 's/usr\/include/usr\/include\/mindvision/g' mindvision-sdk/install.sh
    sed -i 17a\\"mkdir -p /usr/include/mindvision" mindvision-sdk/install.sh
    cd mindvision-sdk && sudo bash ./install.sh && cd ..
    echo ">>> successfully install mindvision-sdk"
    rm -r mindvision-sdk
fi

echo -e "\n\n>>> OpenVINO" && sleep 1
wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB -O /tmp/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
if [ -f /tmp/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB ]; then
    echo "successfully wget GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB"
else
    echo "wget GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB failed"
    exit -1
fi
sudo apt-key add /tmp/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB 
echo "deb https://apt.repos.intel.com/openvino/2023 ubuntu22 main" | sudo tee /etc/apt/sources.list.d/intel-openvino-2023.list # ubuntu22
sudo apt update && sudo apt install openvino -y

echo -e "\n\n>>> ROS 2" && sleep 1
if ! command -v ros2 &> /dev/null
then
    echo "ROS is not installed, start install ROS2"
    wget http://fishros.com/install -O fishros && . fishros
else
    echo "ROS installed, skip install ROS2"
fi

echo -e "\n\n>>> rosdep" && sleep 1
sudo pip3 install rosdepc
sudo rosdepc init # 小鱼的 rosdepc init 会自动 update
rosdep install --from-paths ./ --ignore-src -r -y

cd "$(dirname "$0")"
cd ..
git submodule init
git submodule update
