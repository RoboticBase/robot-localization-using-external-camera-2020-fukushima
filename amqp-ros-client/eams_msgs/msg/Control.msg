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
#       frame_id: stop | start | suspend | resume # コマンドの種類
#     command: 0 # stop    ミッションを停止し、メモリ上のミッション列を削除する
#              1 # start   メモリ上のミッション列を開始する
#              2 # suspend ミッションを中断するが、メモリ上のミッション列は保持する
#              3 # resume  中断したミッションを再開する
#
# * ミッションを一時中断し、後に再開する
#   suspend -> resume
# * ミッションを途中中断し、最初からやり直す
#   suspend -> start
# * ミッションを破棄し、別のミッションを開始する
#   stop -> (新しいミッションを送信) -> start

Header header #ヘッダ
int8 command #ミッションコマンド
