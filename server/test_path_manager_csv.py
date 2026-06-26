import os
import tempfile

import pytest

from path_manager import read_ned_csv


def test_server_csv_reader_rejects_missing_east_column():
    with tempfile.NamedTemporaryFile(mode="w", suffix=".csv", delete=False) as f:
        f.write("1.0\n")
        f.flush()
        path = f.name
    try:
        with pytest.raises(ValueError, match="expected north_m,east_m"):
            read_ned_csv(path)
    finally:
        os.unlink(path)


def test_server_csv_reader_rejects_non_finite_coordinates():
    with tempfile.NamedTemporaryFile(mode="w", suffix=".csv", delete=False) as f:
        f.write("1.0,inf\n")
        f.flush()
        path = f.name
    try:
        with pytest.raises(ValueError, match="finite"):
            read_ned_csv(path)
    finally:
        os.unlink(path)
