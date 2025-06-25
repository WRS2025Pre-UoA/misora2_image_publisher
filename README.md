# misora2_image_publisher
## 内容
 - 各タスクのプログラム検証時にsensor_msgs::msg::Image型で画像を受け取ることが可能になる
 - 疑似的にmisoraからの未加工データを送信する。
## 実行コード
 - ワークスペース下で
~~~bash!
colcon build
source install/setup.bash
ros2 run misora2_imiage_publisher image_publisher --ros-args -p path:=<publishしたい画像のpath> -p topic:=<トピック名の指定> -p format:=<色のフォーマット指定>
~~~
 - pathのデフォルトはsrc/misora2_image_publisher/photo1.png
 - topicのデフォルトは /image_raw
 - colorのデフォルトは color
 - 色のフォーマット
    - color:IMREAD_COLOR
    - mono:IMREAD_GRAYSCALE
    - black:CV_8UC1 黒画像 