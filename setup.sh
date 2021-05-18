# aptで必要なパッケージをインストール
sudo apt update -y
sudo apt -y install qml-module-qtmultimedia qml-module-qtquick-controls qml-module-qtquick-controls2 qml-module-qtgraphicaleffects qml-module-qtquick-dialogs libqt5multimedia5-plugins qml-module-qtquick-shapes

# signageのbuild
colcon build

