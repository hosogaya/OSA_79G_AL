# OSA-79G-AL
アカサカテックさんの前方ミリ波センサとの通信を行うROS2ノードです．
通信規格はUART通信を採用しています．

[商品情報](https://www.akasakatec.com/products/hardware/mm-wave-radar-module/)

# Message
* `front_milli_wave_sensor_msg::msg::TrackerArray`
   * trakcer型の配列で前方ミリ波センサが取得した障害物の情報を複数含んでいる
* `front_milli_wave_sensor_msg::msg::Tracker`
  * `int32 id`: 障害物のid
  * `float32 x`: 障害物の座標
  * `float32 y`:
  * `float32 vx`: 障害物の速度
  * `float32 vy`:
  * `float32 ax`: 障害物の加速度
  * `float32 ay`:


# Topic
|name|type|description|
|---|---|---|
|`/front_milli_wave_sensor_trackers`|`front_milli_wave_sensor_msg::msg::TrackerArray`|センサが検知した障害物|
|`/front_milli_wave_sensor_marker_array`|`visualization_msgs::msg::MarkerArray`|可視化のためのマーカー|

# Prameters
|name|type|description|
|---|---|---|
|`device_name`|string|センサの繋がっているポート名|
|`x_range`|string (double)|センサが検知する横方向の範囲|
|`y_range`|string (double)|センサが検知する縦方向の範囲|
|`frame`|string|センサ情報を表示する座標系|
|`x`|double|`frame`におけるセンサのx座標|
|`y`|double|`frame`におけるセンサのy座標|
|`z`|double|`frame`におけるセンサのz座標|


# Error 
```sh
[front_milli_wave_sensor_connector-1] terminate called after throwing an instance of 'std::invalid_argument'
[front_milli_wave_sensor_connector-1]   what():  stoi
[ERROR] [front_milli_wave_sensor_connector-1]: process has died [pid 25426, exit code -6, cmd '/home/autoware/ros2_ws/install/front_milli_wave_sensor_connector/lib/front_milli_wave_sensor_connector/front_milli_wave_sensor_connector --ros-args -r __node:=front_milli_wave_sensor --params-file /tmp/launch_params_bqksph1i'].
```
例外処理を入れることで回避しました．