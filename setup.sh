# aptで必要なパッケージをインストール
sudo apt update -y
sudo apt -y install python3-pyside2.qtquick qml-module-qtquick-controls qml-module-qtquick-controls2 qml-module-qtquick-shapes libasound2-dev qml-module-qtmultimedia
sudo pip3 install simpleaudio

# signageのbuild
colcon build

