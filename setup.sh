# aptで必要なパッケージをインストール
sudo apt update -y
sudo apt -y install python3-pyqt5.qtquick qml-module-qtquick-controls qml-module-qtquick-controls2 qml-module-qtquick-shapes python3-pyqt5.qtmultimedia libasound2-dev
sudo pip3 install simpleaudio

# signageのbuild
colcon build

