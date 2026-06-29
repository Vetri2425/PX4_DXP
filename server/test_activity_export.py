import asyncio
import csv
import io

import auth
import main
from routes import system


def setup_function():
    auth.reset_for_tests()
    main.activity_log.clear()


def test_activity_csv_route_is_registered():
    paths = {f"/api{getattr(route, 'path', '')}" for route in system.router.routes}

    assert "/api/activity.csv" in paths


def test_activity_csv_exports_existing_bounded_records():
    main.activity_log.append(
        {"timestamp": "2026-06-29T10:00:00Z", "level": "info", "message": "ready"}
    )
    main.activity_log.append(
        {
            "timestamp": "2026-06-29T10:00:01Z",
            "level": "warning",
            "message": "comma, newline\nquoted",
        }
    )

    response = asyncio.run(system.activity_csv())

    assert response.status_code == 200
    assert response.media_type.startswith("text/csv")
    rows = list(csv.DictReader(io.StringIO(response.body.decode("utf-8"))))
    assert rows == [
        {
            "timestamp": "2026-06-29T10:00:00Z",
            "level": "info",
            "message": "ready",
        },
        {
            "timestamp": "2026-06-29T10:00:01Z",
            "level": "warning",
            "message": "comma, newline\nquoted",
        },
    ]
