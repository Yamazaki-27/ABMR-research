# 経由点での座標取り込み方法
## 操作
- プログラムはオリジナルコンテナ内で常時走っている
- タッチパネルを操作
- Node-Redを通じてのWeb-API発行(http://localhost:5000/record/waypoint)は一緒には受け取れない
- そもそも、APIはGETで自分から情報を取りに行く必要があるため面倒な手順になる
- よってオリジナルのAPI(http://localhost:5002/rec)を立ててNode-RedからPOSTしてもらう
- http://localhost:5002/recの中には経由点番号が含まれている
- APIを受信したら、上記経由点番号とともにロボットの現在位置座標X,Yも追記してjsonファイルに書いてから保存
- 現在位置はROSの atmobi/robot_pose トピックから受け取る
- atmobi/robot_poseのデータ型は、nav_msgs/Odometry型
- 保存したいのはコンテナの外にあるディレクトリ(~/map/0014/)
- ファイル名は waypoint
- ファイル(json)が無ければ新規作成
- ファイルがあればデータを追加する