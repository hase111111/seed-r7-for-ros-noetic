# seed-r7-for-ros-noetic
Ubuntu20.04，ROS Noetic用に内容を調整したseed r7のレポジトリです．
以下の内容を含みます。

- seed_r7のパッケージ
- TORK moveit-tutorial
- seed smart actuator

## 環境構築
実行のためには前提条件として、ROS Noeticのインストールと、catkinワークスペースを準備する必要があります。
すでに準備ができている場合は、以下の工程を無視しても構いません。


### ROS Noeticのインストール
ROS Noeticのインストールは以下のコマンドで行うことができます。
```bash
$ sudo apt update

$ sudo apt install ros-noetic-desktop-full
```

### catkinワークスペースの準備
catkinワークスペースを作成するためには、以下のコマンドを実行します。
```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```
