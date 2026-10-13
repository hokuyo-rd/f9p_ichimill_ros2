#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ZED-F9P 向け UBX プロトコルのユーティリティ。

PointPerfect (SPARTN) を利用するには受信機側で
  * SPARTN 入力プロトコルの有効化
  * CFG-SPARTN-USE_SOURCE = IP
  * UBX-RXM-SPARTNKEY (動的キー) の投入
が必要になるため、UBX-CFG-VALSET の生成と UBX フレームの解析をここにまとめる。

キーID・メッセージIDは HPG 1.32 Interface description (UBX-22008968) に準拠。
"""

import struct

UBX_SYNC = b"\xb5\x62"

# --- メッセージクラス / ID ---------------------------------------------------
UBX_CLASS_NAV = 0x01
UBX_CLASS_RXM = 0x02
UBX_CLASS_ACK = 0x05
UBX_CLASS_CFG = 0x06
UBX_CLASS_MON = 0x0A

UBX_ACK_NAK = 0x00
UBX_ACK_ACK = 0x01
UBX_CFG_VALSET = 0x8A
UBX_CFG_VALGET = 0x8B
# UBX-RXM-SPARTNKEY は CFG クラス (0x06 0x36) に定義されている。
# PointPerfect の MQTT トピック "/pp/ubx/0236/ip" の 0236 はこのクラス/IDを表す。
UBX_RXM_SPARTNKEY = 0x36
UBX_RXM_SPARTN = 0x33
UBX_NAV_SAT = 0x35
UBX_MON_RF = 0x38

# --- 設定レイヤ --------------------------------------------------------------
LAYER_RAM = 0x01
LAYER_BBR = 0x02
LAYER_FLASH = 0x04

# --- 設定キーID --------------------------------------------------------------
# 入力プロトコル有効化 (L: 1bit)
INPROT_KEYS = {
    # インタフェース名: (UBX, NMEA, RTCM3X, SPARTN)
    "i2c":   (0x10710001, 0x10710002, 0x10710004, 0x10710005),
    "uart1": (0x10730001, 0x10730002, 0x10730004, 0x10730005),
    "uart2": (0x10750001, 0x10750002, 0x10750004, 0x10750005),
    "usb":   (0x10770001, 0x10770002, 0x10770004, 0x10770005),
    "spi":   (0x10790001, 0x10790002, 0x10790004, 0x10790005),
}

# 出力プロトコル有効化 (L: 1bit)
OUTPROT_KEYS = {
    # インタフェース名: (UBX, NMEA, RTCM3X)
    "i2c":   (0x10720001, 0x10720002, 0x10720004),
    "uart1": (0x10740001, 0x10740002, 0x10740004),
    "uart2": (0x10760001, 0x10760002, 0x10760004),
    "usb":   (0x10780001, 0x10780002, 0x10780004),
    "spi":   (0x107a0001, 0x107a0002, 0x107a0004),
}

# NMEA 出力レート (U1: 航法周期あたりの出力回数)
# CFG-MSGOUT-NMEA_ID_<talker>_<port>
MSGOUT_NMEA_KEYS = {
    # センテンス: {インタフェース: キーID}
    "gga": {"i2c": 0x209100BA, "uart1": 0x209100BB, "uart2": 0x209100BC,
            "usb": 0x209100BD, "spi": 0x209100BE},
    "rmc": {"i2c": 0x209100AB, "uart1": 0x209100AC, "uart2": 0x209100AD,
            "usb": 0x209100AE, "spi": 0x209100AF},
    "zda": {"i2c": 0x209100D8, "uart1": 0x209100D9, "uart2": 0x209100DA,
            "usb": 0x209100DB, "spi": 0x209100DC},
    "gsv": {"i2c": 0x209100C4, "uart1": 0x209100C5, "uart2": 0x209100C6,
            "usb": 0x209100C7, "spi": 0x209100C8},
    "gsa": {"i2c": 0x209100BF, "uart1": 0x209100C0, "uart2": 0x209100C1,
            "usb": 0x209100C2, "spi": 0x209100C3},
    "gll": {"i2c": 0x209100C9, "uart1": 0x209100CA, "uart2": 0x209100CB,
            "usb": 0x209100CC, "spi": 0x209100CD},
    "vtg": {"i2c": 0x209100B0, "uart1": 0x209100B1, "uart2": 0x209100B2,
            "usb": 0x209100B3, "spi": 0x209100B4},
}

# 航法レート
CFG_RATE_MEAS = 0x30210001   # U2 [ms] 測位周期
CFG_RATE_NAV = 0x30210002    # U2 測位周期あたりの航法解算出回数

# SPARTN 補正源 (E1) 0 = IP, 1 = L-band
CFG_SPARTN_USE_SOURCE = 0x20A70001
SPARTN_SOURCE_IP = 0
SPARTN_SOURCE_LBAND = 1


def checksum(payload: bytes):
    """UBX の 8bit Fletcher チェックサムを計算する。"""
    ck_a = 0
    ck_b = 0
    for byte in payload:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def build_frame(msg_class: int, msg_id: int, payload: bytes = b"") -> bytes:
    """UBX フレーム (同期文字・長さ・チェックサム付き) を組み立てる。"""
    body = struct.pack("<BBH", msg_class, msg_id, len(payload)) + payload
    ck_a, ck_b = checksum(body)
    return UBX_SYNC + body + bytes((ck_a, ck_b))


def key_value_size(key: int) -> int:
    """設定キーIDのストレージサイズ種別から値のバイト数を求める。"""
    storage_size = (key >> 28) & 0x07
    sizes = {0x1: 1, 0x2: 1, 0x3: 2, 0x4: 4, 0x5: 8}
    if storage_size not in sizes:
        raise ValueError(f"Unknown storage size in key 0x{key:08x}")
    return sizes[storage_size]


def encode_key_value(key: int, value: int) -> bytes:
    """設定キーIDと値を CFG-VALSET の cfgData 形式にエンコードする。"""
    size = key_value_size(key)
    fmt = {1: "<B", 2: "<H", 4: "<I", 8: "<Q"}[size]
    if (key >> 28) & 0x07 == 0x1:  # L 型は 0/1 に丸める
        value = 1 if value else 0
    return struct.pack("<I", key) + struct.pack(fmt, value)


def build_valset(key_values, layers: int = LAYER_RAM) -> bytes:
    """UBX-CFG-VALSET フレームを生成する。

    key_values: (キーID, 値) のシーケンス
    layers    : LAYER_RAM / LAYER_BBR / LAYER_FLASH の OR
    """
    payload = struct.pack("<BBBB", 0x00, layers & 0xFF, 0x00, 0x00)
    for key, value in key_values:
        payload += encode_key_value(key, value)
    return build_frame(UBX_CLASS_CFG, UBX_CFG_VALSET, payload)


def split_valset(key_values, layers: int = LAYER_RAM, max_keys: int = 64):
    """キー数が多い場合に備えて CFG-VALSET を分割して生成する。"""
    key_values = list(key_values)
    return [build_valset(key_values[i:i + max_keys], layers)
            for i in range(0, len(key_values), max_keys)]


def parse_frame(buffer: bytes):
    """バッファ先頭の UBX フレームを取り出す。

    戻り値は (msg_class, msg_id, payload, 消費バイト数)。
    フレームが未完成の場合は (None, None, None, 0) を返す。
    チェックサム不一致の場合は (None, None, None, 消費バイト数) を返し、
    呼び出し側が読み飛ばせるようにする。
    """
    if len(buffer) < 8:
        return None, None, None, 0
    if not buffer.startswith(UBX_SYNC):
        return None, None, None, 0

    length = struct.unpack("<H", buffer[4:6])[0]
    total = 6 + length + 2
    if len(buffer) < total:
        return None, None, None, 0

    body = buffer[2:6 + length]
    ck_a, ck_b = checksum(body)
    if (ck_a, ck_b) != (buffer[6 + length], buffer[7 + length]):
        # 同期文字らしきものを読み飛ばして再同期させる
        return None, None, None, 2

    return buffer[2], buffer[3], buffer[6:6 + length], total


def is_ubx_frame(data: bytes) -> bool:
    """データ列が UBX フレームとして妥当かを判定する。"""
    if len(data) < 8 or not data.startswith(UBX_SYNC):
        return False
    length = struct.unpack("<H", data[4:6])[0]
    if len(data) < 6 + length + 2:
        return False
    ck_a, ck_b = checksum(data[2:6 + length])
    return (ck_a, ck_b) == (data[6 + length], data[7 + length])


# --- 設定キーの名前（ログ表示用） -------------------------------------------
def key_name(key: int) -> str:
    """設定キーIDから分かりやすい名前を引く。未知なら 16 進で返す。"""
    for iface, keys in INPROT_KEYS.items():
        for proto, k in zip(("UBX", "NMEA", "RTCM3X", "SPARTN"), keys):
            if k == key:
                return f"CFG-{iface.upper()}INPROT-{proto}"
    for sentence, ports in MSGOUT_NMEA_KEYS.items():
        for iface, k in ports.items():
            if k == key:
                return f"CFG-MSGOUT-NMEA_ID_{sentence.upper()}_{iface.upper()}"
    return {
        CFG_SPARTN_USE_SOURCE: "CFG-SPARTN-USE_SOURCE",
        CFG_RATE_MEAS: "CFG-RATE-MEAS",
        CFG_RATE_NAV: "CFG-RATE-NAV",
    }.get(key, f"0x{key:08x}")


def build_valget(keys, layer: int = 0, position: int = 0) -> bytes:
    """UBX-CFG-VALGET フレームを生成する。

    layer: 0 = RAM, 1 = BBR, 2 = Flash, 7 = Default
    """
    payload = struct.pack("<BBH", 0x00, layer, position)
    for key in keys:
        payload += struct.pack("<I", key)
    return build_frame(UBX_CLASS_CFG, UBX_CFG_VALGET, payload)


def parse_valget(payload: bytes):
    """UBX-CFG-VALGET 応答から (キーID, 値) の一覧を取り出す。"""
    results = []
    if len(payload) < 4:
        return results
    index = 4  # version, layer, position をスキップ
    while index + 4 <= len(payload):
        key = struct.unpack("<I", payload[index:index + 4])[0]
        index += 4
        try:
            size = key_value_size(key)
        except ValueError:
            break
        if index + size > len(payload):
            break
        fmt = {1: "<B", 2: "<H", 4: "<I", 8: "<Q"}[size]
        results.append((key, struct.unpack(fmt, payload[index:index + size])[0]))
        index += size
    return results


# --- 診断用メッセージの出力設定 ---------------------------------------------
# アンテナ状態と衛星の信号強度を受信機から直接読むために使う。
MSGOUT_UBX_KEYS = {
    "nav_sat": {"i2c": 0x20910015, "uart1": 0x20910016, "uart2": 0x20910017,
                "usb": 0x20910018, "spi": 0x20910019},
    "mon_rf":  {"i2c": 0x20910359, "uart1": 0x2091035A, "uart2": 0x2091035B,
                "usb": 0x2091035C, "spi": 0x2091035D},
}

ANT_STATUS = {0: "INIT", 1: "DONTKNOW", 2: "OK", 3: "SHORT", 4: "OPEN"}
ANT_POWER = {0: "OFF", 1: "ON", 2: "DONTKNOW"}


def parse_mon_rf(payload: bytes):
    """UBX-MON-RF から RF ブロックごとの状態を取り出す。

    antStatus が OPEN ならアンテナが接続されていない、SHORT なら短絡。
    agcCnt と jamInd は受信環境の指標。
    """
    if len(payload) < 4:
        return []
    blocks = []
    for index in range(payload[1]):
        offset = 4 + index * 24
        if offset + 24 > len(payload):
            break
        block = payload[offset:offset + 24]
        blocks.append({
            "id": block[0],
            "ant_status": ANT_STATUS.get(block[2], str(block[2])),
            "ant_power": ANT_POWER.get(block[3], str(block[3])),
            "noise": struct.unpack("<H", block[12:14])[0],
            "agc": struct.unpack("<H", block[14:16])[0],
            "jamming": block[16],
        })
    return blocks


def parse_nav_sat(payload: bytes):
    """UBX-NAV-SAT から衛星数と C/N0 の一覧を取り出す。"""
    if len(payload) < 8:
        return 0, []
    num_svs = payload[5]
    cnos = []
    for index in range(num_svs):
        offset = 8 + index * 12
        if offset + 12 > len(payload):
            break
        cnos.append(payload[offset + 2])
    return num_svs, cnos
