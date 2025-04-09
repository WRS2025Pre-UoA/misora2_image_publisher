# misora2_image_publisher
## 実装目的
 - 各タスクのプログラム検証時にsensor_msgs::msg::Image型で画像を受け取ることが可能になる
## 実行方法
 - ワークスペース下で
~~~bash!
colcon build
source install/setup.bash
ros2 run misora2_imiage publisher image_publisher --ros-args -p path:=<publishしたい画像のpath> -p topic:=<トピック名の指定> -p color:=<色のフォーマット指定>
~~~
 - pathは必須
 - topicのデフォルトは /image
 - colorのデフォルトは mono