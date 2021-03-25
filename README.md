# robot-localization-using-external-camera-2020-fukushima
The ROS packages for the robot localization using external cameras.

## Description
This repository contains the ROS packages for estimating the robot-localization using the robot's images observed by external cameras.
These softwares were developed by a subsidized project of the robot industries in Fukushima Prefecture.

このリポジトリには、外部カメラで観測されたロボットの画像を用いて、ロボットの位置を推定するためのROSパッケージが格納されています。
これらのソフトウェアは、福島県の令和２年度ロボット関連産業基盤強化事業によって開発されました。

## ROS packages
* [ROS client](/amqp-ros-client/)
    * A ROS package that consumes messages from a management system and publishes them to the autonomous mobile robot, and that subscribes messages from the robot and produces them to the management system.
* [Camera Controller](/camera-controller)
    * A ROS package that launches external camera system.

## License
[Apache License 2.0](/LICENSE)

## Copyright
Copyright (c) 2021 [TIS Inc.](https://www.tis.co.jp/)

