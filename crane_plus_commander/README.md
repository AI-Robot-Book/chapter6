# CRANE+用のROS2ノード群を利用する簡単なノード

升谷 保博  
2022年2月  

## 概要

- アールティ社が公開している同社のロボットアーム[CRANE+用のROS2ノード群 crane_plus](https://github.com/rt-net/crane_plus)を利用するノード．
- ノードのプログラムは，Pythonで記述．
- MoveIt2は使わずに，各関節へ指令値を送る．
- [crane_plus_gazebo](https://github.com/rt-net/crane_plus/tree/master/crane_plus_gazebo)を使うことによってGazebo内のCRANE+を同じように動かすこともできる．
- Ubuntu 20.04, ROS Foxyで作成・確認

## インストール

- [crane_plusのREADME](https://github.com/rt-net/crane_plus/blob/master/README.md)に沿って作業する．

- ROSのワークスペースを`~/airobot_ws`とする．

- このパッケージを含むリポジトリを入手
- サービスのインタフェースを定義しているパッケージを含むリポジトリを入手
- パッケージをビルド
  ```
  cd ~/airobot_ws
  colcon build --packages-select ai_robot_book_interfaces crane_plus_commander
  ```

## 使い方

- [crane_plus_controlのREADME](https://github.com/rt-net/crane_plus/blob/master/crane_plus_control/README.md)に沿って実機の設定を行う．

- 端末1
  - オーバレイヤの設定
    ```
    cd ~/airobot_ws
    source install/setup.bash
    ```

  - 実機の場合（robot_state_publisher付き）
    ```
    ros2 launch crane_plus_commander crane_plus_control_rsp.launch.py
    ```
  - 実機の代わりGazeboを使う場合
    ```
    ros2 launch crane_plus_commander crane_plus_gazebo_no_moveit.launch.py 
    ```

- 端末2
  - オーバレイヤの設定
    ```
    cd ~/airobot_ws
    source install/setup.bash
    ```
  - キー操作で関節値を変化させる場合
    ```
    ros2 run crane_plus_commander commander1
    ```

  - キー操作で手先位置も変化させる場合
    ```
    ros2 run crane_plus_commander commander2
    ```

  - キー操作で関節値を変化させつつ，関節の状態を表示する場合
    ```
    ros2 run crane_plus_commander commander3
    ```

  - 同期的な（結果を待つ）アクションクライアントの場合
    ```
    ros2 run crane_plus_commander commander4
    ```

  - tfを利用して順運動学の計算をする場合
    ```
    ros2 run crane_plus_commander commander5
    ```

  - アクションクライアント＋サービスサーバとして使う場合
    ```
    ros2 run crane_plus_commander commander6
    ```

- 端末3（サービスサーバをテストする場合）
  - テスト用のクライアント
    ```
    cd ~/airobot_ws
    source install/setup.bash
    ros2 run crane_plus_commander test_client
    ```

