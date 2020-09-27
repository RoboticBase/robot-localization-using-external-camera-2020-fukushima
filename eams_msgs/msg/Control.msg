# Control.msg
# ミッションの開始・終了・中断のメッセージ定義
#   [topic]
#     /command/control
#   [msg]
#     header:
#       seq: xxx # 一意な単調増加番号（自動採番）
#       stamp:
#         secs: xxxxxxxxxx # unix timeの秒
#         nsecs: xxxxxxxx # ナノ秒（精度はOS依存）
#       frame_id: stop | start | suspend # コマンドの種類
#     command: 0 # stop    ミッションを停止し、メモリ上のミッション列を削除する
#              1 # start   メモリ上のミッション列を開始する
#              2 # suspend ミッションを中断するが、メモリ上のミッション列は保持する

Header header #ヘッダ
int8 command #ミッションコマンド
