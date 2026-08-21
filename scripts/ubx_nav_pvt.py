"""Helpers for extracting UBX frames from the F9P serial byte stream."""

import struct


UBX_SYNC = b"\xb5\x62"
NAV_PVT_CLASS = 0x01
NAV_PVT_ID = 0x07
NAV_PVT_LENGTH = 92


def ubx_checksum(data):
    """Return the two-byte Fletcher checksum used by UBX."""
    ck_a = 0
    ck_b = 0
    for value in data:
        ck_a = (ck_a + value) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes((ck_a, ck_b))


class UbxNmeaParser:
    """Incrementally split an interleaved UBX/NMEA serial stream."""

    def __init__(self):
        self._buffer = bytearray()

    def feed(self, data):
        """Return ``(nmea_sentences, ubx_frames)`` parsed from *data*.

        UBX frames are returned as ``(message_class, message_id, payload)``.
        Invalid data and frames with a bad checksum are discarded.
        """
        self._buffer.extend(data)
        nmea_sentences = []
        ubx_frames = []

        while self._buffer:
            nmea_start = self._buffer.find(b"$")
            ubx_start = self._buffer.find(UBX_SYNC)
            starts = [offset for offset in (nmea_start, ubx_start) if offset >= 0]
            if not starts:
                # Retain a possible first UBX sync byte across reads.
                self._buffer[:] = self._buffer[-1:] if self._buffer[-1:] == b"\xb5" else b""
                break

            start = min(starts)
            if start:
                del self._buffer[:start]

            if self._buffer.startswith(b"$"):
                newline = self._buffer.find(b"\n")
                if newline < 0:
                    break
                raw_sentence = bytes(self._buffer[:newline + 1]).rstrip(b"\r\n")
                del self._buffer[:newline + 1]
                try:
                    nmea_sentences.append(raw_sentence.decode("ascii"))
                except UnicodeDecodeError:
                    pass
                continue

            if len(self._buffer) < 6:
                break
            payload_length = int.from_bytes(self._buffer[4:6], "little")
            frame_length = payload_length + 8
            if len(self._buffer) < frame_length:
                break

            frame = bytes(self._buffer[:frame_length])
            if ubx_checksum(frame[2:-2]) == frame[-2:]:
                ubx_frames.append((frame[2], frame[3], frame[6:-2]))
                del self._buffer[:frame_length]
            else:
                # Move past the first sync byte and search for the next frame.
                del self._buffer[0]

        return nmea_sentences, ubx_frames


def unpack_nav_pvt(payload):
    """Decode a protocol-version 14+ UBX-NAV-PVT payload into field values."""
    if len(payload) < NAV_PVT_LENGTH:
        raise ValueError(
            f"UBX-NAV-PVT payload must be at least {NAV_PVT_LENGTH} bytes; "
            f"received {len(payload)}"
        )

    values = struct.unpack_from("<IHBBBBBBIiBBBBiiiiIIiiiiiIIH6sihH", payload)
    names = (
        "i_tow", "year", "month", "day", "hour", "min", "sec", "valid",
        "t_acc", "nano", "fix_type", "flags", "flags2", "num_sv", "lon",
        "lat", "height", "h_msl", "h_acc", "v_acc", "vel_n", "vel_e",
        "vel_d", "g_speed", "heading", "s_acc", "head_acc", "p_dop",
        "reserved1", "head_veh", "mag_dec", "mag_acc",
    )
    result = dict(zip(names, values))
    result["reserved1"] = list(result["reserved1"])
    return result
