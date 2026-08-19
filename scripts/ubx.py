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
    """Decode a UBX-NAV-PVT frame into raw fields and SI conveniences.

    The raw values retain the units used by ``ublox_msgs/msg/NavPVT``.  The
    existing converted keys are kept for the accompanying ``NavSatFix``.
    """
    if (len(frame) != NAV_PVT_PAYLOAD_LENGTH + 8
            or frame[2] != NAV_PVT_CLASS
            or frame[3] != NAV_PVT_ID
            or ubx_checksum(frame[2:-2]) != frame[-2:]):
        raise ValueError("not a valid UBX-NAV-PVT frame")

    payload = frame[6:-2]
    fields = struct.unpack_from("<IHBBBBBBIiBBBBiiiiIIiiiiiIIH6sihH", payload)
    names = (
        "i_tow", "year", "month", "day", "hour", "min", "sec", "valid",
        "t_acc", "nano", "fix_type", "flags", "flags2", "num_sv", "lon",
        "lat", "height_raw", "h_msl", "h_acc", "v_acc", "vel_n", "vel_e",
        "vel_d", "g_speed", "head_mot", "s_acc", "head_acc", "p_dop",
        "reserved1", "head_veh", "mag_dec", "mag_acc")
    result = dict(zip(names, fields))
    result["reserved1"] = list(result["reserved1"])
    result.update({
        "gnss_fix_ok": bool(result["flags"] & 0x01),
        "longitude": result["lon"] * 1e-7,
        "latitude": result["lat"] * 1e-7,
        "height": result["height_raw"] * 1e-3,
        "mean_sea_level": result["h_msl"] * 1e-3,
        "horizontal_accuracy": result["h_acc"] * 1e-3,
        "vertical_accuracy": result["v_acc"] * 1e-3,
    })
    return result
