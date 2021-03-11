# Mission.msg
# ミッション列のメッセージ定義（ヘッダ）
#   [topic]
#     /command/mission
#   [msg]
#     header:
#       seq: xxx # 一意な単調増加番号（自動採番）
#       stamp:
#         secs: xxxxxxxxxx # unix timeの秒
#         nsecs: xxxxxxxx # ナノ秒（精度はOS依存）
#       frame_id: mission # mission固定
#     details:
#       - command: 1 # waypoint  通過すべきwaypointを指定する（latとlongの指定が必須）
#                  2 # delay     指定した時間待つ（param1の指定が必須）
#                  3 # yaw       指定された向きに超信地旋回する（param1とparam2の指定が必須）
#         lat: xx.xxxxxxxxxx   # command == 1 の場合
#                              #   緯度
#                              # command != 1 の場合
#                              #   無視
#         long: xxx.xxxxxxxxxx # command == 1 の場合
#                              #   経度
#                              # command != 1 の場合
#                              #   無視
#         param1: x.x          # command == 2 の場合
#                              #   遅延時間（秒）
#                              # command == 3 の場合
#                              #   ロボットの角度（度）
#                              # command != 2 && command != 3 の場合
#                              #   無視
#         param2: x.x          # command == 3 の場合
#                              #   0.0 # absolute 北を0度とし、時計回りでparam1の角度を向く
#                              #       # 北は0度、東は90度、南は180度、西は270度
#                              #   1.0 # relative 回転前のロボットの向きを0度とし、時計回りでparam1の角度を向く
#                              # command != 3 の場合
#                              #   無視
#         param3: 0.0          # 無視
#         param4: 0.0          # 無視

Header header #ヘッダ
Detail[] details # ミッションの詳細（Detail.msgの配列）
