# ROS2 野球　投球管理・警告システム
![test](https://github.com/peke0157/mkpkg/actions/workflows/test.yml/badge.svg)
![License](https://img.shields.io/github/license/peke0157/mypkg)

本パッケージはROS2で動作する、投手の投球数管理パッケージです。
複数の投手の投球数を個別に管理することができ、設定された投球数を超えると警告を発し、投手交代を促します。

## ノードの説明
### pitch_server.py
このシステムの中核となる管理ノードです。野球で言えば「公式記録員」の役割を果たします。
- 役割
    - データの保持：現在登板している投手の名前と、各投手の投球数をメモリ上で管理します。
    - 判定と警告：投球されるたびに投球カウントを増やし、設定された制限数（パラメータpitch_limit）を超えていないか判定します。デフォルトは100球です。
    - 指令の受付：外部からの「投手交代」の指令を受け取り、カウント対象を切り替えます。
- 動作
    - サービス（/count_pitch）が呼ばれると、現在選択されている投手の球数を+1します。
    - 球数が制限内であればSuccess: Trueを返しますが、100球を超えるとSuccess: Falseを返し、警告をクライアントに発します。
 
### pitch_client.py
このシステムは動作テスト用ノードです。野球で言えば投手そのものの役割を果たします。
- 役割
    - 投球の実演：定期的（1秒ごと）にサーバーに対して「投げた」という合図を送ります。
    - 結果の確認：サーバーから帰ってきた判定（「OK」または「制限超過」）を受け取り、ログに表示します。

- 動作
    - 起動するとサーバーが立ち上がるのを待機します。
    - 接続後は、実行終了（Ctrl+C）するまで無限ループで/count_pitchサービスをリクエストし続けます。
    - サーバーから「制限超過（Limited Exceed）」のエラーが帰ってきてもテストのため、リクエストを送り続けます。警告を無くすには投手を変えるか、球数をリセットするか実行を終了してください。

## 実行方法
- このリポジトリをターミナルで下記のようにクローンしてください。
```
$ git clone https://github.com/peke0157/mypkg.git
```

- ros2_wsのディレクトリに移動し、ビルドし、sourceを通して設定ファイルの内容を反映してください。
```
$ cd ~/ros2_ws
$ colcon build
$ source ~/.bashrc
```
- 投球数などをまとめて見たい場合はローンチファイルを実行します
```
$ ros2 launch mypkg pitchserver_client.launch.py
[INFO] [launch]: All log files can be found below /home/hiroto117/.ros/log/2025-12-29-11-27-18-133397-Hirotovivobook-4735
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [pitch_server-1]: process started with pid [4738]
[INFO] [pitch_client-2]: process started with pid [4739]
[pitch_server-1] [INFO] [1766975238.805480984] [pitch_server]: Pitch Server Ready
[pitch_server-1] [INFO] [1766975238.805883719] [pitch_server]: Start Pitcher is set to: Collie
[pitch_server-1] [INFO] [1766975240.046978720] [pitch_server]: Pitch_Count: 1/100
[pitch_client-2] [INFO] [1766975240.070582204] [pitch_client]: Server Response: Success=True, Message="OK"
[pitch_server-1] [INFO] [1766975241.045120372] [pitch_server]: Pitch_Count: 2/100
[pitch_client-2] [INFO] [1766975241.046256821] [pitch_client]: Server Response: Success=True, Message="OK"
[pitch_server-1] [INFO] [1766975242.046736823] [pitch_server]: Pitch_Count: 3/100
[pitch_client-2] [INFO] [1766975242.049194256] [pitch_client]: Server Response: Success=True, Message="OK"
[pitch_server-1] [INFO] [1766975243.045052369] [pitch_server]: Pitch_Count: 4/100
[pitch_client-2] [INFO] [1766975243.046015261] [pitch_client]: Server Response: Success=True, Message="OK"
[pitch_server-1] [INFO] [1766975244.045011990] [pitch_server]: Pitch_Count: 5/100
[pitch_client-2] [INFO] [1766975244.046121197] [pitch_client]: Server Response: Success=True, Message="OK"
[pitch_server-1] [INFO] [1766975245.044867214] [pitch_server]: Pitch_Count: 6/100
[pitch_client-2] [INFO] [1766975245.045783787] [pitch_client]: Server Response: Success=True, Message="OK"
[pitch_server-1] [INFO] [1766975246.045250131] [pitch_server]: Pitch_Count: 7/100
[pitch_client-2] [INFO] [1766975246.046111908] [pitch_client]: Server Response: Success=True, Message="OK"
[pitch_server-1] [INFO] [1766975247.047093506] [pitch_server]: Pitch_Count: 8/100
[pitch_client-2] [INFO] [1766975247.049297479] [pitch_client]: Server Response: Success=True, Message="OK"
[pitch_server-1] [INFO] [1766975248.045448100] [pitch_server]: Pitch_Count: 9/100
[pitch_client-2] [INFO] [1766975248.046837508] [pitch_client]: Server Response: Success=True, Message="OK"
[pitch_server-1] [INFO] [1766975249.045014300] [pitch_server]: Pitch_Count: 10/100
[pitch_client-2] [INFO] [1766975249.046009530] [pitch_client]: Server Response: Success=True, Message="OK"
```
上記のように投球数が1秒ごとに1球増えます。
100球を超えると下記の通りに警告が出ます。
```
[pitch_server-1] [INFO] [1766975340.045420619] [pitch_server]: Pitch_Count: 100/100
[pitch_client-2] [INFO] [1766975340.047072171] [pitch_client]: Server Response: Success=True, Message="OK"
[pitch_server-1] [INFO] [1766975341.047293878] [pitch_server]: Pitch_Count: 101/100
[pitch_client-2] [INFO] [1766975341.049963749] [pitch_client]: Server Response: Success=False, Message="Limited Exceed! count = (101)"
[pitch_server-1] [INFO] [1766975342.044995672] [pitch_server]: Pitch_Count: 102/100
[pitch_client-2] [INFO] [1766975342.046227572] [pitch_client]: Server Response: Success=False, Message="Limited Exceed! count = (102)"
[pitch_server-1] [INFO] [1766975343.044804709] [pitch_server]: Pitch_Count: 103/100
[pitch_client-2] [INFO] [1766975343.045899831] [pitch_client]: Server Response: Success=False, Message="Limited Exceed! count = (103)"
[pitch_server-1] [INFO] [1766975344.045187991] [pitch_server]: Pitch_Count: 104/100
[pitch_client-2] [INFO] [1766975344.046523121] [pitch_client]: Server Response: Success=False, Message="Limited Exceed! count = (104)"
[pitch_server-1] [INFO] [1766975345.045917778] [pitch_server]: Pitch_Count: 105/100
[pitch_client-2] [INFO] [1766975345.047600676] [pitch_client]: Server Response: Success=False, Message="Limited Exceed! count = (105)"
```
投手を変更したいまたは投球数をリセットしたいときは下記のコマンドを打ちます。
    - 投手を変更したいとき
```
ros2 param set /pitch_server start_pitcher "peke"
```

    - 投球数をリセットしたいとき
```
ros2 service call /reset_pitch std_srvs/srv/Trigger
```


## 必要なソフトウェア
- ROS2
- Python

## テスト環境
- Ubuntu 24.04.1 LTS
- ROS2 Jazzy
- Python 3.13.5
  
## ライセンス
- このソフトウェアパッケージは，3条項BSDライセンスの下，再頒布および使用が許可されます。
- © 2025 Hiroto Miura
