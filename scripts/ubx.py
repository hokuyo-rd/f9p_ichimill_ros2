"""Utilities for extracting and decoding u-blox UBX messages."""

import struct


UBX_SYNC = b"\xb5\x62"
NAV_PVT_CLASS = 0x01
NAV_PVT_ID = 0x07
NAV_PVT_PAYLOAD_LENGTH = 92


def ubx_checksum(data):
    """Return the two-byte Fletcher checksum used by UBX."""
    ck_a = 0
    ck_b = 0
    for value in data:
        ck_a = (ck_a + value) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes((ck_a, ck_b))


class GnssStreamParser:
    """Split a mixed serial byte stream into NMEA sentences and UBX frames."""

    def __init__(self, max_payload_length=4096):
        self.buffer = bytearray()
        self.max_payload_length = max_payload_length

    def feed(self, data):
        self.buffer.extend(data)
        messages = []

        while self.buffer:
            if self.buffer.startswith(UBX_SYNC):
                if len(self.buffer) < 6:
                    break
                payload_length = int.from_bytes(self.buffer[4:6], "little")
                if payload_length > self.max_payload_length:
                    del self.buffer[0]
                    continue
                frame_length = payload_length + 8
                if len(self.buffer) < frame_length:
                    break
                frame = bytes(self.buffer[:frame_length])
                del self.buffer[:frame_length]
                if ubx_checksum(frame[2:-2]) == frame[-2:]:
                    messages.append(("ubx", frame))
                continue

            if self.buffer[0] == ord('$'):
                newline = self.buffer.find(b'\n')
                if newline < 0:
                    # A UBX sync indicates that the preceding NMEA fragment is
                    # corrupt; discard it rather than blocking binary parsing.
                    sync = self.buffer.find(UBX_SYNC, 1)
                    if sync >= 0:
                        del self.buffer[:sync]
                        continue
                    break
                sentence = bytes(self.buffer[:newline + 1])
                del self.buffer[:newline + 1]
                messages.append(("nmea", sentence.rstrip(b'\r\n')))
                continue

            # Skip noise, but retain a possible first UBX sync byte.
            if self.buffer[0] == UBX_SYNC[0] and len(self.buffer) == 1:
                break
            del self.buffer[0]

        return messages


def decode_nav_pvt(frame):
    """Decode fields required for a NavSatFix from a UBX-NAV-PVT frame."""
    if (len(frame) != NAV_PVT_PAYLOAD_LENGTH + 8
            or frame[2] != NAV_PVT_CLASS
            or frame[3] != NAV_PVT_ID
            or ubx_checksum(frame[2:-2]) != frame[-2:]):
        raise ValueError("not a valid UBX-NAV-PVT frame")

    payload = frame[6:-2]
    fix_type = payload[20]
    flags = payload[21]
    longitude, latitude, height, mean_sea_level = struct.unpack_from(
        "<iiii", payload, 24)
    horizontal_accuracy, vertical_accuracy = struct.unpack_from("<II", payload, 40)
    return {
        "fix_type": fix_type,
        "gnss_fix_ok": bool(flags & 0x01),
        "longitude": longitude * 1e-7,
        "latitude": latitude * 1e-7,
        "height": height * 1e-3,
        "mean_sea_level": mean_sea_level * 1e-3,
        "horizontal_accuracy": horizontal_accuracy * 1e-3,
        "vertical_accuracy": vertical_accuracy * 1e-3,
    }
