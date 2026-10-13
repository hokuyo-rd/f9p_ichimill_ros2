#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ROS を介さず F9P の生の状態を確認する単体ツール。

ドライバのパース結果を疑わずに済むよう、受信機から直接読んで素の値を表示する。
ファームウェア版数・アンテナ状態・衛星ごとの C/N0 が一度に分かる。

    ros2 run f9p_ichimill gnss_check [/dev/ttyACM0]

ノードがシリアルポートを掴んでいると開けないため、先に停止すること。
"""

import struct
import sys
import time

import serial

GNSS = {0: 'GPS', 1: 'SBAS', 2: 'GAL', 3: 'BDS', 4: 'IMES', 5: 'QZSS', 6: 'GLO', 7: 'NavIC'}
ANT_STATUS = {0: 'INIT', 1: 'DONTKNOW', 2: 'OK', 3: 'SHORT', 4: 'OPEN'}
ANT_POWER = {0: 'OFF', 1: 'ON', 2: 'DONTKNOW'}

# (クラス, ID, 表示名, 内部キー)
POLLS = (
    (0x0A, 0x04, 'MON-VER', 'ver'),
    (0x0A, 0x09, 'MON-HW', 'hw'),
    (0x0A, 0x38, 'MON-RF', 'rf'),
    (0x01, 0x35, 'NAV-SAT', 'sat'),
)


def frame(msg_class, msg_id, payload=b''):
    """UBX フレームを組み立てる (ペイロード長 0 は問い合わせになる)。"""
    body = struct.pack('<BBH', msg_class, msg_id, len(payload)) + payload
    ck_a = ck_b = 0
    for byte in body:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return b'\xb5\x62' + body + bytes((ck_a, ck_b))


def show_mon_ver(payload):
    print("=== MON-VER (ファームウェア) ===")
    print("  sw:", payload[0:30].split(b'\0')[0].decode(errors='ignore'))
    print("  hw:", payload[30:40].split(b'\0')[0].decode(errors='ignore'))
    for index in range(40, len(payload), 30):
        text = payload[index:index + 30].split(b'\0')[0].decode(errors='ignore')
        if text:
            print("     ", text)


def show_mon_hw(payload):
    print(f"\n=== MON-HW (アンテナ, payload={len(payload)}B) ===")
    if len(payload) < 46:
        print("  ペイロードが短く解釈できない")
        return
    print(f"  aStatus={ANT_STATUS.get(payload[20], payload[20])}"
          f"  aPower={ANT_POWER.get(payload[21], payload[21])}"
          f"  noise={struct.unpack('<H', payload[16:18])[0]}"
          f"  agc={struct.unpack('<H', payload[18:20])[0]}"
          f"  jamInd={payload[45]}")


def show_mon_rf(payload):
    print(f"\n=== MON-RF (payload={len(payload)}B, blocks={payload[1]}) ===")
    for index in range(payload[1]):
        offset = 4 + index * 24
        if offset + 24 > len(payload):
            break
        block = payload[offset:offset + 24]
        print(f"  block{block[0]}: antStatus={ANT_STATUS.get(block[2], block[2])}"
              f" antPower={ANT_POWER.get(block[3], block[3])}"
              f" noise={struct.unpack('<H', block[12:14])[0]}"
              f" agc={struct.unpack('<H', block[14:16])[0]}"
              f" jam={block[16]}")


def show_nav_sat(payload):
    num_svs = payload[5]
    print(f"\n=== NAV-SAT (payload={len(payload)}B, numSvs={num_svs}, "
          f"期待長={8 + num_svs * 12}B) ===")
    rows = []
    for index in range(num_svs):
        offset = 8 + index * 12
        if offset + 12 > len(payload):
            break
        gnss_id, sv_id, cno = payload[offset], payload[offset + 1], payload[offset + 2]
        elev = struct.unpack('<b', payload[offset + 3:offset + 4])[0]
        azim = struct.unpack('<h', payload[offset + 4:offset + 6])[0]
        quality = struct.unpack('<I', payload[offset + 8:offset + 12])[0] & 0x7
        rows.append((cno, gnss_id, sv_id, elev, azim, quality))
    if not rows:
        print("  衛星情報なし")
        return
    rows.sort(reverse=True)
    print("  C/N0  系   SV  仰角  方位  qual")
    for cno, gnss_id, sv_id, elev, azim, quality in rows[:16]:
        print(f"  {cno:4d}  {GNSS.get(gnss_id, gnss_id):4s} {sv_id:3d} "
              f"{elev:5d} {azim:5d}  {quality}")
    tracked = [r for r in rows if r[0] > 0]
    strong = [r for r in rows if r[0] >= 35]
    print(f"\n  信号あり {len(tracked)}/{len(rows)} 機, 最大 {rows[0][0]} dBHz, "
          f"35dBHz以上 {len(strong)} 機")
    print(f"  仰角の範囲 {min(r[3] for r in rows)}..{max(r[3] for r in rows)} "
          f"(-90..90 なら解釈は正しい)")
    if len(strong) < 4:
        print("  → 測位には 35 dBHz 以上が 4 機以上必要。アンテナと設置場所を確認")


HANDLERS = {
    (0x0A, 0x04): show_mon_ver,
    (0x0A, 0x09): show_mon_hw,
    (0x0A, 0x38): show_mon_rf,
    (0x01, 0x35): show_nav_sat,
}


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    try:
        receiver = serial.Serial(port, 230400, timeout=0.2, write_timeout=2.0)
    except serial.SerialException as ex:
        print(f"シリアルポートを開けない: {ex}")
        print("ノードが掴んでいないか確認: pgrep -af f9p_ichimill")
        return 1

    print(f"opened {port}\n")
    for msg_class, msg_id, _name, _key in POLLS:
        receiver.write(frame(msg_class, msg_id))
        time.sleep(0.15)

    buffer = b''
    seen = set()
    deadline = time.time() + 8
    while time.time() < deadline and len(seen) < len(POLLS):
        buffer += receiver.read(max(1, receiver.in_waiting))
        while True:
            start = buffer.find(b'\xb5\x62')
            if start < 0 or len(buffer) - start < 8:
                break
            length = struct.unpack('<H', buffer[start + 4:start + 6])[0]
            if len(buffer) - start < 6 + length + 2:
                break
            body = buffer[start + 2:start + 6 + length]
            ck_a = ck_b = 0
            for byte in body:
                ck_a = (ck_a + byte) & 0xFF
                ck_b = (ck_b + ck_a) & 0xFF
            if (ck_a, ck_b) != (buffer[start + 6 + length], buffer[start + 7 + length]):
                buffer = buffer[start + 2:]
                continue
            key = (buffer[start + 2], buffer[start + 3])
            payload = buffer[start + 6:start + 6 + length]
            buffer = buffer[start + 6 + length + 2:]
            if key in HANDLERS and key not in seen:
                seen.add(key)
                HANDLERS[key](payload)

    for msg_class, msg_id, name, _key in POLLS:
        if (msg_class, msg_id) not in seen:
            print(f"\n[応答なし] {name}")
    receiver.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
