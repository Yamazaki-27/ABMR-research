# -*- coding: utf-8 -*-
import urllib2
import json

def post_waypoint_command():
    # 1. @mobiのAPIから現在の状態を取得
    # ポート5000の /state エンドポイントを使用します
    mobi_state_url = "http://localhost:5000/state"
    
    try:
        # GETリクエストの送信
        response = urllib2.urlopen(mobi_state_url)
        data = json.loads(response.read())
        
        # 現在向かっている経由点番号を抽出
        # 自律走行中でない場合は -1 が返る可能性があります
        cur_wp = data.get("cur_waypoint")
        
        # 2. 送信用データの準備
        # 経由点番号と文字列 "up" をセットにします
        payload = {
            "waypoint_no": cur_wp,
            "status": "up"
        }
        
        # 3. 独自のソフトウェアへPOST送信
        target_url = "http://localhost:5002/turn"
        
        # リクエストオブジェクトを作成し、ヘッダーにJSON形式であることを指定
        req = urllib2.Request(target_url)
        req.add_header('Content-Type', 'application/json')
        
        try:
            # POSTリクエストの送信 (データを渡すと自動的にPOSTになります)
            post_res = urllib2.urlopen(req, json.dumps(payload))
            print("成功: 経由点 {} と 'up' を送信しました。".format(cur_wp))
        except urllib2.HTTPError as e:
            print("送信失敗: HTTPエラーコード {}".format(e.code))
        except urllib2.URLError as e:
            print("送信失敗: 通信エラー {}".format(e.reason))
            
    except urllib2.HTTPError as e:
        print("@mobi 状態取得失敗: HTTPエラーコード {}".format(e.code))
    except urllib2.URLError as e:
        print("@mobi 状態取得失敗: 通信エラー {}".format(e.reason))
    except Exception as e:
        print("エラーが発生しました: {}".format(e))

if __name__ == "__main__":
    post_waypoint_command()