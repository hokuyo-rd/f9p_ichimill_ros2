import struct

import pytest

from scripts.ubx_nav_pvt import UbxNmeaParser, ubx_checksum, unpack_nav_pvt


def make_frame(message_class, message_id, payload):
    body = bytes((message_class, message_id)) + len(payload).to_bytes(2, "little") + payload
    return b"\xb5\x62" + body + ubx_checksum(body)


def test_parser_handles_fragmented_interleaved_stream():
    payload = bytes(range(92))
    frame = make_frame(1, 7, payload)
    parser = UbxNmeaParser()

    nmea, ubx = parser.feed(b"noise$GNGGA,one*00\r\n" + frame[:19])
    assert nmea == ["$GNGGA,one*00"]
    assert ubx == []

    nmea, ubx = parser.feed(frame[19:] + b"$GNRMC,two*00\n")
    assert nmea == ["$GNRMC,two*00"]
    assert ubx == [(1, 7, payload)]


def test_parser_rejects_bad_checksum_and_recovers():
    bad_frame = bytearray(make_frame(1, 7, bytes(92)))
    bad_frame[-1] ^= 0xFF
    good_frame = make_frame(1, 7, bytes(92))

    assert UbxNmeaParser().feed(bad_frame + good_frame)[1] == [(1, 7, bytes(92))]


def test_unpack_nav_pvt_maps_every_field():
    values = (
        123456, 2026, 8, 21, 12, 34, 56, 15, 42, -100, 3, 129, 224, 18,
        139123456, 351234567, 12345, 12000, 500, 800, 100, -200, 30, 225,
        9000000, 50, 10000, 125, b"\x01\x02\x03\x04\x05\x06", 9100000, -123, 45,
    )
    payload = struct.pack("<IHBBBBBBIiBBBBiiiiIIiiiiiIIH6sihH", *values)

    decoded = unpack_nav_pvt(payload)

    assert decoded["i_tow"] == 123456
    assert decoded["nano"] == -100
    assert decoded["lon"] == 139123456
    assert decoded["reserved1"] == [1, 2, 3, 4, 5, 6]
    assert decoded["head_veh"] == 9100000
    assert decoded["mag_dec"] == -123


def test_unpack_nav_pvt_rejects_legacy_short_payload():
    with pytest.raises(ValueError, match="at least 92 bytes"):
        unpack_nav_pvt(bytes(84))
