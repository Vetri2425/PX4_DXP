#!/usr/bin/env python3
"""Local Jetson CLI for rover authentication setup.

Usage:
  python3 server/rover_auth_cli.py setup
  python3 server/rover_auth_cli.py reset-password
  python3 server/rover_auth_cli.py create-machine-token --name bag-autorecord
"""
from __future__ import annotations

import argparse
import getpass
import sys
from pathlib import Path

from auth import create_machine_token, write_password_hash
from config import AUTH_MACHINE_TOKENS_FILE, AUTH_PASSWORD_FILE


def _read_new_password() -> str:
    password = getpass.getpass("New rover operator password: ")
    confirm = getpass.getpass("Confirm password: ")
    if password != confirm:
        raise SystemExit("passwords did not match")
    if len(password) < 8:
        raise SystemExit("password must be at least 8 characters")
    return password


def cmd_setup(args: argparse.Namespace) -> None:
    path = Path(args.password_file)
    if path.exists() and not args.force:
        raise SystemExit(f"password file already exists: {path}")
    write_password_hash(_read_new_password(), path=path)
    print(f"operator password configured at {path}")


def cmd_reset_password(args: argparse.Namespace) -> None:
    path = Path(args.password_file)
    write_password_hash(_read_new_password(), path=path)
    print(f"operator password reset at {path}")


def cmd_create_machine_token(args: argparse.Namespace) -> None:
    token = create_machine_token(args.name, path=Path(args.machine_tokens_file))
    print("machine token created; copy it into the bag-autorecord service env")
    print(token)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Rover local authentication CLI")
    parser.set_defaults(func=None)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--password-file", default=AUTH_PASSWORD_FILE)
    machine = argparse.ArgumentParser(add_help=False)
    machine.add_argument("--machine-tokens-file", default=AUTH_MACHINE_TOKENS_FILE)

    setup = parser.add_subparsers(dest="command", required=True)
    p_setup = setup.add_parser("setup", parents=[common])
    p_setup.add_argument("--force", action="store_true")
    p_setup.set_defaults(func=cmd_setup)

    p_reset = setup.add_parser("reset-password", parents=[common])
    p_reset.set_defaults(func=cmd_reset_password)

    p_machine = setup.add_parser("create-machine-token", parents=[machine])
    p_machine.add_argument("--name", default="bag-autorecord")
    p_machine.set_defaults(func=cmd_create_machine_token)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    args.func(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
