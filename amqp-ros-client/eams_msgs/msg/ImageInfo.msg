# ImageInfo.msg
# 360度カメラ画像のCloud Storageのパスとハッシュ値のメッセージ定義
#   [topic]
#     /camera/image_info
#   [msg]
#     header:
#       seq: xxx # 一意な単調増加番号（自動採番）
#       stamp:
#         secs: xxxxxxxxxx # unix timeの秒
#         nsecs: xxxxxxxx # ナノ秒（精度はOS依存）
#       frame_id: image_info
#     time: xxxxxxxxxxxx ハッシュ値計算に用いた画像撮影日時の文字列
#     lat: xx.xxxxxxxxxx   # ハッシュ値計算に用いた画像撮影緯度の文字列
#     lng: xxx.xxxxxxxxxx  # ハッシュ値計算に用いた画像撮影経度の文字列
#     yaw: xxx.xxx  # ハッシュ値計算に用いた画像撮影方向（コンパスの値）の文字列
#     hash: xxxxxxxxxx # ロボットごとのSALT+画像撮影日時+画像を撮影した緯度・経度・方向+画像のバイト列 から計算したハッシュ値
#     path: xxxxxxxxxx # 画像をアップロードしたCloud Storageのパス

Header header #ヘッダ
string time #画像撮影日時
string lat #緯度
string lng #経度
string yaw #方向
string hash #ハッシュ値
string path #パス
