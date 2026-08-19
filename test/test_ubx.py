import struct
import unittest

from scripts.ubx import GnssStreamParser, decode_nav_pvt, ubx_checksum


def make_ubx(message_class, message_id, payload):
    body = bytes((message_class, message_id)) + struct.pack("<H", len(payload)) + payload
    return b"\xb5\x62" + body + ubx_checksum(body)


class TestGnssStreamParser(unittest.TestCase):
    def test_extracts_fragmented_nmea_and_nav_pvt(self):
        payload = bytearray(92)
        payload[20] = 3
        payload[21] = 1
        struct.pack_into("<iiii", payload, 24, 1391234567, 351234567, 12345, 10000)
        struct.pack_into("<II", payload, 40, 1200, 2300)
        nav_pvt = make_ubx(0x01, 0x07, payload)
        stream = b"noise$GNGGA,example*00\r\n" + nav_pvt + b"$GNRMC,example*00\n"

        parser = GnssStreamParser()
        messages = []
        for chunk in (stream[:5], stream[5:31], stream[31:70], stream[70:]):
            messages.extend(parser.feed(chunk))

        self.assertEqual([item[0] for item in messages], ["nmea", "ubx", "nmea"])
        self.assertEqual(messages[0][1], b"$GNGGA,example*00")
        pvt = decode_nav_pvt(messages[1][1])
        self.assertAlmostEqual(pvt["latitude"], 35.1234567)
        self.assertAlmostEqual(pvt["longitude"], 139.1234567)
        self.assertAlmostEqual(pvt["height"], 12.345)
        self.assertEqual(pvt["fix_type"], 3)
        self.assertTrue(pvt["gnss_fix_ok"])

    def test_rejects_bad_checksum_and_recovers(self):
        bad_frame = bytearray(make_ubx(0x01, 0x07, bytes(92)))
        bad_frame[-1] ^= 0xFF
        parser = GnssStreamParser()

        messages = parser.feed(bytes(bad_frame) + b"$GNZDA,example*00\r\n")

        self.assertEqual(messages, [("nmea", b"$GNZDA,example*00")])

    def test_rejects_other_messages_as_nav_pvt(self):
        frame = make_ubx(0x01, 0x02, bytes(92))
        with self.assertRaises(ValueError):
            decode_nav_pvt(frame)


if __name__ == "__main__":
    unittest.main()
