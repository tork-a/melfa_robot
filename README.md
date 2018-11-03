# melfa_ros [![Build Status](https://travis-ci.com/tork-a/melfa_ros.svg?token=Eg7EHKJ8kwE5VZs6TwDp&branch=master)](https://travis-ci.com/tork-a/melfa_ros)

三菱電機製産業ロボットアーム"RV-7FL"をROSから制御するためのパッケージ
群です．

## Loopback nodeについて

`melfa_driver/melfa_driver_node`がコントローラ本体です．このコントロー
ラは，TCP/IPの通信により，ロボット実機のコントローラ，もしくはWindows
で動くシミュレータ"RT Toolbox3"と通信して動作します．

しかし，普通は両方共持ってないので，ロボット実機のコントローラの動作を
模した，ループバックノード(ダミー)を作成しています．それが，
`melfa_driver/melfa_loopback_node`です．これは，TCP/IP経由で与えられた
関節角度の指令値をそのまま現在値として返すだけのノードです．

## Quick start with the loopback node

ループバックノードを使うようにコントローラを立ち上げるには，以下のよう
にします．

```
$ roslaunch melfa_driver melfa_driver.launch loopback:=true
```

rvizでロボットモデルを表示するのは以下のようにします．

```
$ roslaunch melfa_description rviz.launch 
```

rqtのJointTrajctoryControllerプラグインを使うと，スライダでロボットの
各軸を操作できます．

```
$ rqt -s rqt_joint_trajectory_controller/JointTrajectoryController
```
