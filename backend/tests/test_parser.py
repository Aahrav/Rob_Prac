"""Tests for backend.parser."""

from backend.parser import parse_sensor_line


def test_parse_valid_json():
    d = parse_sensor_line('{"t":10,"r":1,"p":2,"y":3}')
    assert d is not None
    assert d["t"] == 10
    assert d["r"] == 1.0


def test_parse_error_payload():
    d = parse_sensor_line('{"error":"sensor_init_failed"}')
    assert d == {"_error": "sensor_init_failed"}


def test_parse_garbage_returns_none():
    assert parse_sensor_line("not json") is None
    assert parse_sensor_line("") is None
