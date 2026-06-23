"""Unit tests for RTCM3 streaming parser."""

from __future__ import annotations

import time

import pytest

from rtcm3_parser import (
    Rtcm3StreamParser,
    build_rtcm3_frame,
    parse_allowed_message_types,
    rtcm3_crc24q,
    rtcm3_message_type,
)


def _payload_for_type(msg_type: int, extra: bytes = b"\x00") -> bytes:
    return bytes([(msg_type >> 4) & 0xFF, ((msg_type & 0x0F) << 4) | 0x0F]) + extra


def test_crc24q_known_vector():
    # Independently constructed: payload for RTCM type 1005 (12 zero bytes).
    payload = _payload_for_type(1005, b"\x00" * 10)
    frame = build_rtcm3_frame(payload)
    assert rtcm3_crc24q(frame, len(frame) - 3) == int.from_bytes(frame[-3:], "big")
    assert rtcm3_message_type(frame) == 1005


def test_partial_chunk_reassembly():
    payload = _payload_for_type(1074, b"\x01\x02\x03")
    frame = build_rtcm3_frame(payload)
    parser = Rtcm3StreamParser()
    first = parser.feed(frame[:4])
    assert first == []
    second = parser.feed(frame[4:])
    assert second == [frame]
    assert parser.snapshot_stats().valid_frames == 1


def test_multiple_frames_one_chunk():
    frames_in = [build_rtcm3_frame(_payload_for_type(t)) for t in (1005, 1074, 1084)]
    blob = b"".join(frames_in)
    parser = Rtcm3StreamParser()
    out = parser.feed(blob)
    assert out == frames_in
    assert parser.snapshot_stats().valid_frames == 3


def test_bad_crc_not_published():
    payload = _payload_for_type(1005, b"\xab\xcd")
    frame = bytearray(build_rtcm3_frame(payload))
    frame[-1] ^= 0xFF
    parser = Rtcm3StreamParser()
    out = parser.feed(bytes(frame))
    assert out == []
    stats = parser.snapshot_stats()
    assert stats.crc_errors == 1
    assert stats.dropped_frames == 1


def test_bad_preamble_and_oversized_dropped():
    parser = Rtcm3StreamParser(max_frame_size=20)
    garbage = b"\x00\x01\xff" + bytes([0xD3, 0xFF, 0xFF])  # length 1023
    out = parser.feed(garbage)
    assert out == []
    stats = parser.snapshot_stats()
    assert stats.invalid_frames >= 1
    assert stats.dropped_frames >= 1


def test_resync_after_garbage():
    payload = _payload_for_type(1230, b"\x11\x22")
    frame = build_rtcm3_frame(payload)
    parser = Rtcm3StreamParser()
    out = parser.feed(b"\x00\x00\x00" + frame)
    assert out == [frame]


def test_message_type_filter():
    allowed = parse_allowed_message_types("1005,1074")
    parser = Rtcm3StreamParser(allowed_message_types=allowed)
    good = build_rtcm3_frame(_payload_for_type(1005))
    bad = build_rtcm3_frame(_payload_for_type(999))
    out = parser.feed(good + bad)
    assert out == [good]
    assert parser.snapshot_stats().dropped_frames == 1


def test_rate_limit_drops_excess(monkeypatch):
    parser = Rtcm3StreamParser(max_frames_per_sec=2.0, max_bytes_per_sec=100000)
    frame = build_rtcm3_frame(_payload_for_type(1005, b"\x00" * 4))
    t = 1000.0
    monkeypatch.setattr(time, "monotonic", lambda: t)
    first = parser.feed(frame + frame)
    assert len(first) == 2
    third = parser.feed(frame)
    assert third == []
    assert parser.snapshot_stats().dropped_frames == 1


def test_bounded_buffer_drops_overflow():
    parser = Rtcm3StreamParser(max_buffer_size=16)
    parser.feed(b"\x00" * 32)
    assert parser.snapshot_stats().dropped_frames >= 1
    assert len(parser._buf) <= 16


def test_parser_has_no_injected_byte_counter():
    frame = build_rtcm3_frame(_payload_for_type(1005))
    parser = Rtcm3StreamParser()
    parser.feed(frame)
    stats = parser.snapshot_stats()
    assert stats.valid_frames == 1
    assert "bytes_injected" not in stats.__dict__