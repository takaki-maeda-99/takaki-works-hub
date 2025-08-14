#!/usr/bin/env python3
"""
利用可能なCOMポートを確認するスクリプト
"""

import serial.tools.list_ports

def list_serial_ports():
    """利用可能なシリアルポートを一覧表示"""
    ports = serial.tools.list_ports.comports()
    
    if not ports:
        print("利用可能なCOMポートが見つかりません")
        return
    
    print("利用可能なCOMポート:")
    print("-" * 60)
    for port in ports:
        print(f"ポート: {port.device}")
        print(f"説明:   {port.description}")
        print(f"ハードウェア ID: {port.hwid}")
        print("-" * 60)
    
    # Teensyらしきポートを特定
    teensy_ports = []
    for port in ports:
        if any(keyword in port.description.lower() for keyword in ['teensy', 'usb serial']):
            teensy_ports.append(port.device)
    
    if teensy_ports:
        print(f"\nTeensyの可能性があるポート: {teensy_ports}")
    else:
        print("\nTeensyらしきポートが見つかりません")

if __name__ == "__main__":
    list_serial_ports()