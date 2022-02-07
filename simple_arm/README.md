# 簡単な2自由度ロボットアームのモデル

升谷 保博  
2022年2月  

## 概要

- ロボットアームがどのようなものかを知るためのもの
- URDFの見本
- RVizでロボットモデルを表示する見本

## インストール
- このパッケージを含むリポジトリの入手

- パッケージのビルド
  ```
  sudo apt install ros-foxy-joint-state-publisher-gui
  cd ~/airobot_ws
  colcon build --packages-select simple_arm_description
  source install/setup.bash
  ```

## 使い方

- 端末で以下を実行
  ```
  ros2 launch simple_arm_description display.launch.py
  ```
- `joint_state_publisher_gui`のウインドウのスライダを操作．