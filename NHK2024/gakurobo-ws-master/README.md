# gakurobo_ws

## Environment

- Ubuntu 22.04
- ROS 2 Humble

## Setup

本リポジトリをクローンする.
```sh
git clone https://github.com/ti-robot-robocon/gakurobo-ws.git gakurobo_ws --recursive
cd gakurobo_ws
```

依存パッケージをインストールする.
```sh
sudo apt install ros-humble-pcl-*
rosdep update && rosdep install -i -y --from-paths src
```

ビルドする.
```sh
colcon build --symlink-install
source install/setup.bash
```
