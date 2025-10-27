"""CAN interface helper for Raspberry Pi

Provides utilities to check and bring up a SocketCAN interface (e.g. can0)
programmatically. This is a convenience wrapper around the "ip" command and
will try to run it directly. Note: creating/configuring kernel CAN interfaces
requires elevated privileges (CAP_NET_ADMIN). See README for options (sudo or
setcap).

Usage:
    from utils.can_setup import ensure_can_interface
    ensure_can_interface('can0', bitrate=500000)

The function will attempt a best-effort configuration and raises a
RuntimeError with a helpful message if it fails.

This module intentionally avoids forcing sudo; prefer granting capabilities to
python (setcap) or running the script with sudo/systemd.
"""

from __future__ import annotations
import subprocess
import shutil
from typing import Tuple


def _run(cmd: list[str], check: bool = True) -> Tuple[int, str, str]:
    """Run a command, return (rc, stdout, stderr)"""
    try:
        proc = subprocess.run(cmd, check=check, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        return proc.returncode, proc.stdout.strip(), proc.stderr.strip()
    except subprocess.CalledProcessError as e:
        return e.returncode, e.stdout.strip() if e.stdout else "", e.stderr.strip() if e.stderr else ""


def interface_exists(ifname: str) -> bool:
    """Return True if interface exists (ip link show dev <ifname> succeeds)."""
    ip_cmd = shutil.which('ip')
    if not ip_cmd:
        raise RuntimeError("`ip` command not found on system")
    rc, out, err = _run([ip_cmd, 'link', 'show', 'dev', ifname], check=False)
    return rc == 0


def interface_state(ifname: str) -> str:
    """Return the administrative state of the interface (UP/DOWN) or 'missing'."""
    if not interface_exists(ifname):
        return 'missing'
    ip_cmd = shutil.which('ip')
    rc, out, err = _run([ip_cmd, 'link', 'show', 'dev', ifname], check=False)
    # sample output contains: "state UP" or "state DOWN"
    for token in out.split():
        if token == 'state':
            # next token
            parts = out.split()
            idx = parts.index('state')
            if idx + 1 < len(parts):
                return parts[idx + 1]
    return 'unknown'


def ensure_can_interface(ifname: str = 'can0', bitrate: int = 500000, use_sudo: bool = False) -> None:
    """Ensure a SocketCAN interface exists and is up with the requested bitrate.

    This will:
    - check that `ip` exists
    - bring the interface down, set its type to `can` and requested bitrate,
      then bring it up.

    Args:
        ifname: interface name (default: can0)
        bitrate: bitrate in bits per second (default: 500000)
        use_sudo: if True, prefix commands with `sudo` when necessary. If False,
                  commands will be attempted without sudo and will fail if not
                  permitted.

    Raises:
        RuntimeError: if the operation fails. The error message will include the
                      command stderr to help debugging.
    """
    ip_cmd = shutil.which('ip')
    if not ip_cmd:
        raise RuntimeError("`ip` command not found on this system. Install iproute2.")

    prefix = ['sudo'] if use_sudo else []

    # Bring the interface down if it exists
    if interface_exists(ifname):
        rc, out, err = _run(prefix + [ip_cmd, 'link', 'set', ifname, 'down'], check=False)
        # ignore rc, we will attempt to configure next

    # Set interface type to can and bitrate
    # Note: this will fail if the interface does not exist or the driver isn't
    # loaded; the caller should ensure overlays/drivers are present.
    cmd = prefix + [ip_cmd, 'link', 'set', ifname, 'type', 'can', 'bitrate', str(bitrate)]
    rc, out, err = _run(cmd, check=False)
    if rc != 0:
        raise RuntimeError(f"Failed to set {ifname} type=can bitrate={bitrate}: {err}")

    # Bring the interface up
    cmd_up = prefix + [ip_cmd, 'link', 'set', ifname, 'up']
    rc, out, err = _run(cmd_up, check=False)
    if rc != 0:
        raise RuntimeError(f"Failed to bring {ifname} up: {err}")

    # Confirm it is UP
    state = interface_state(ifname)
    if state.upper() != 'UP':
        raise RuntimeError(f"Interface {ifname} not UP after configuration (state={state})")


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Bring up a SocketCAN interface programmatically')
    parser.add_argument('--ifname', default='can0')
    parser.add_argument('--bitrate', type=int, default=500000)
    parser.add_argument('--use-sudo', action='store_true', default=False, help='Prefix commands with sudo')
    args = parser.parse_args()

    try:
        ensure_can_interface(args.ifname, bitrate=args.bitrate, use_sudo=args.use_sudo)
        print(f"Interface {args.ifname} configured and up @ {args.bitrate}")
    except Exception as e:
        print(f"Error: {e}")
