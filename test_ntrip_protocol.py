from ntrip_protocol import gga_position_is_usable, response_is_success


def test_accepts_standard_ntrip_success_lines():
    assert response_is_success("ICY 200 OK")
    assert response_is_success("HTTP/1.0 200 OK\r\nServer: caster")
    assert response_is_success("HTTP/1.1 200\r\n")


def test_rejects_lookalike_or_sourcetable_responses():
    assert not response_is_success("HTTP/1.1 401 Unauthorized\r\nX-Reason: 200")
    assert not response_is_success("HTTP/1.1 2000 Strange")
    assert not response_is_success("SOURCETABLE 200 OK")
    assert not response_is_success("")


def test_gga_requires_fresh_finite_in_range_position():
    assert gga_position_is_usable(13.0, 80.0, 20.0, 0.2)
    assert not gga_position_is_usable(float("nan"), 80.0, 20.0, 0.2)
    assert not gga_position_is_usable(91.0, 80.0, 20.0, 0.2)
    assert not gga_position_is_usable(13.0, 181.0, 20.0, 0.2)
    assert not gga_position_is_usable(13.0, 80.0, 20.0, 5.1)
