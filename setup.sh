# aptで必要なパッケージをインストール
sudo apt update -y
sudo apt -y install python-pyqt5.qtquick qml-module-qtquick-controls2 qml-module-qtquick-controls

# signageのbuild
colcon build

