"""S4 — upload size caps on the two point-CSV parse routes.

/parse-point-csv and /parse-point-gps-csv did an unbounded ``await
file.read()`` while every other upload route reads ``MAX_UPLOAD_BYTES + 1``
and raises 413. These tests pin the now-shared behaviour on all four routes:
oversize → 413 before any parsing; under-cap → the cap does not fire (the
parser sees the content).

Follows the server-test convention: call the async route coroutines directly.
"""
import io
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

import pytest
from fastapi import HTTPException
from starlette.datastructures import UploadFile

import routes.path as path_route

pytestmark = pytest.mark.anyio


@pytest.fixture
def anyio_backend():
    return "asyncio"


def _upload(content: bytes, name: str) -> UploadFile:
    return UploadFile(file=io.BytesIO(content), filename=name)


_OVERSIZE = b"x" * (path_route.MAX_UPLOAD_BYTES + 1)


async def test_parse_point_csv_rejects_oversize():
    with pytest.raises(HTTPException) as ei:
        await path_route.parse_point_csv(file=_upload(_OVERSIZE, "pts.csv"))
    assert ei.value.status_code == 413


async def test_parse_point_gps_csv_rejects_oversize():
    with pytest.raises(HTTPException) as ei:
        await path_route.parse_point_gps_csv(file=_upload(_OVERSIZE, "pts_gps.csv"))
    assert ei.value.status_code == 413


async def test_parse_point_csv_under_cap_still_parses():
    body = b"north,east\n1.0,2.0\n"
    result = await path_route.parse_point_csv(file=_upload(body, "pts.csv"))
    assert result["num_points"] == 1


async def test_parse_point_csv_under_cap_invalid_content_is_422_not_413():
    # The cap must not swallow parse errors: garbage under the cap reaches the
    # parser and comes back 422.
    with pytest.raises(HTTPException) as ei:
        await path_route.parse_point_csv(file=_upload(b"not,a header\nx,y\n", "bad.csv"))
    assert ei.value.status_code == 422


async def test_upload_rejects_oversize():
    # The pre-existing capped route, pinned so the four routes stay consistent.
    with pytest.raises(HTTPException) as ei:
        await path_route.upload_path(file=_upload(_OVERSIZE, "big.dxf"))
    assert ei.value.status_code == 413
