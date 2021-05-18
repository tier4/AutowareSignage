# aptで必要なパッケージをインストール
sudo apt update -y
sudo apt -y install python3-pyqt5.qtquick qml-module-qtquick-controls2 qml-module-qtquick-shapes

# signageのbuild
colcon build

