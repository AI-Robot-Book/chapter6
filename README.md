# 第6章 マニピュレーション

## 概要

ROS2とPythonで作って学ぶAIロボット入門（出村・萩原・升谷・タン著，講談社）第6章のサンプルプログラムと補足情報などを掲載しています．

## ディレクトリ構成

- [crane_plus_commander](crane_plus_commander)： CRANE+ V2用のROS2ノード群を利用する簡単なノード

- [simple_arm/simple_arm_description](simple_arm/simple_arm_description)： 簡単な2自由度ロボットアームのモデル

## サンプルプログラム一覧
- プログラムリスト6.1 [kinematics.pyの一部](crane_plus_commander/crane_plus_commander/kinematics.py#L59-L68)
- プログラムリスト6.2 [kinematics.pyの一部](crane_plus_commander/crane_plus_commander/kinematics.py#L71-L95)
- プログラムリスト6.2 [simple_arm.urdf](simple_arm/simple_arm_description/urdf/simple_arm.urdf)
- 6.5.5 関節を動かすプログラム [commander1.py](crane_plus_commander/crane_plus_commander/commander1.py)
- 6.5.6 手先を動かすプログラム [commander2.py](crane_plus_commander/crane_plus_commander/commander2.py)
- 6.5.7 ロボットの状態を受け取るプログラム [commander3.py](crane_plus_commander/crane_plus_commander/commander3.py)
- 6.5.8 ROS2のアクション通信を利用するプログラム [commander4.py](crane_plus_commander/crane_plus_commander/commander4.py)
- 6.6.3 tfを使ったプログラム [commander5.py](crane_plus_commander/crane_plus_commander/commander5.py)
- 6.8 他のノードから指令を受けて動作するプログラム [commander6.py](crane_plus_commander/crane_plus_commander/commander6.py)

## 補足情報

今のところありません．
