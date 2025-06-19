# 必要なライブラリをインポート
from pydualsense import pydualsense
import struct
import time
import serial

# --- 設定 ---
BAUD   = 115200       # ボーレート
PORT   = "COM23"        # 使用するシリアルポート (環境に合わせて変更)
PERIOD = 0.01           # 送信周期 (秒) / 0.01s = 100Hz

def main():
    """
    DualSenseコントローラーの入力を読み取り、シリアルポートに送信するメイン関数
    """
    ds = None
    ser = None
    try:
        # DualSenseコントローラーを初期化
        ds = pydualsense()
        ds.init()
        print("DualSenseコントローラーの待機中...")

        # コントローラーが接続されるまで待機
        while not ds.connected:
            time.sleep(1.0)
        print("コントローラーが接続されました。")
        # 状態が安定するまで少し待つ
        time.sleep(0.5)

        # シリアルポートを開く
        # write_timeoutを設定し、書き込みがブロックされるのを防ぐ
        ser = serial.Serial(PORT, BAUD, write_timeout=0.1)
        print(f"シリアルポート {PORT} を {BAUD} bps で開きました。")

        # メインループ
        while ds.connected:
            # スティックの値を-128~127から0~255の範囲に変換
            # スティックの中央(0)が128になる
            lx = ds.state.LX + 128
            ly = -(ds.state.LY - 127)
            rx = ds.state.RX + 128
            ry = -(ds.state.RY - 127)

            # デバッグ用に値を出力 (この行のコメントを外して確認できます)
            print(f"LX: {lx}, LY: {ly}, RX: {rx}, RY: {ry}")

            # 4つの値を符号なしバイト(B)としてパックし、シリアル送信
            # '<'はリトルエンディアンを指定
            packet = struct.pack("<BBBB", lx, ly, rx, ry)
            ser.write(packet)

            # 指定した周期で待機
            time.sleep(PERIOD)

    except serial.SerialTimeoutException:
        print(f"エラー: シリアル書き込みがタイムアウトしました。")
        print(f"ポート {PORT} に接続された受信側デバイスがデータを受け取っていない可能性があります。")
        print("受信側のプログラムや配線を確認してください。")
    except serial.SerialException as e:
        print(f"シリアルポートエラー: {e}")
        print(f"ポート {PORT} が利用可能か、他のプログラム(Arduinoのシリアルモニタ等)で使用中でないか確認してください。")
    except KeyboardInterrupt:
        # Ctrl+Cが押されたらループを抜ける
        print("\nプログラムを終了します。")
    finally:
        # プログラム終了時にリソースを解放
        if ser and ser.is_open:
            ser.close()
            print("シリアルポートを閉じました。")
        if ds and hasattr(ds, 'connected') and ds.connected:
            ds.close()
            print("コントローラー接続を閉じました。")


if __name__ == "__main__":
    main()
