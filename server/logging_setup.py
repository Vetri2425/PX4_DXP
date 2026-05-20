"""Structured logging configuration.

Replaces the bare `print()` calls scattered through the prototype with a
single configured root logger. ISO-8601 timestamps, level, name, message.
"""
from __future__ import annotations

import logging
import logging.config
import sys


def configure_logging(level: str = "INFO") -> None:
    logging.config.dictConfig({
        "version": 1,
        "disable_existing_loggers": False,
        "formatters": {
            "default": {
                "format": "%(asctime)s.%(msecs)03d [%(levelname)s] %(name)s — %(message)s",
                "datefmt": "%Y-%m-%dT%H:%M:%S",
            },
        },
        "handlers": {
            "stderr": {
                "class": "logging.StreamHandler",
                "stream": sys.stderr,
                "formatter": "default",
            },
        },
        "root": {
            "handlers": ["stderr"],
            "level": level,
        },
        "loggers": {
            "uvicorn":       {"handlers": ["stderr"], "level": level, "propagate": False},
            "uvicorn.error": {"handlers": ["stderr"], "level": level, "propagate": False},
            "uvicorn.access":{"handlers": ["stderr"], "level": "WARNING", "propagate": False},
        },
    })


def get_logger(name: str) -> logging.Logger:
    return logging.getLogger(name)
