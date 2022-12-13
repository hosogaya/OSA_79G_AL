# OSA-79G-AL
アカサカテックさんの前方ミリ波センサとの通信を行うROS2ノードです．
通信規格はUART通信を採用しています．

[商品情報](https://www.akasakatec.com/products/hardware/mm-wave-radar-module/)

# To do list
1. 本体との接続確認
2. ~~launch ファイルの作成~~
3. センサ情報のパブリッシュの仕方を決定する．（標準のメッセージ型を使うか，独自メッセージ型を使うか）
   1. ->独自メッセージを作成しました。

# TrackerArray msg
trakcer型の配列. 
## Tracker型　
```Tracker.msg
int32 id # tracker id
float32 x # position 
float32 y
float32 vx # velocity
float32 vy
float32 ax # acceleration
float32 ay
```

# Error 
```sh
[front_milli_wave_sensor_connector-1] terminate called after throwing an instance of 'std::invalid_argument'
[front_milli_wave_sensor_connector-1]   what():  stoi
[ERROR] [front_milli_wave_sensor_connector-1]: process has died [pid 25426, exit code -6, cmd '/home/autoware/ros2_ws/install/front_milli_wave_sensor_connector/lib/front_milli_wave_sensor_connector/front_milli_wave_sensor_connector --ros-args -r __node:=front_milli_wave_sensor --params-file /tmp/launch_params_bqksph1i'].
```