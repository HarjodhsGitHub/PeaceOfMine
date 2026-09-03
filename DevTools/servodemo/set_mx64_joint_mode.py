#!/usr/bin/env python3
"""Inspect or explicitly change an MX-64 from wheel mode to joint mode.

Opening the ArbotiX FTDI link resets its controller, so the program waits for
the ROS firmware to answer before accessing the DYNAMIXEL bus.

By default this script only reports the current configuration.  Supplying
--apply writes the selected CCW angle limit to EEPROM with torque disabled.
It never re-enables torque or commands motion.
"""

from __future__ import annotations

import argparse
import sys
import time

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler


PORT = "/dev/cu.usbserial-AI049UTL"
HOST_BAUD = 115_200
SERVO_ID = 1
PROTOCOL_VERSION = 1.0
GATEWAY_ID = 253
GATEWAY_READY_TIMEOUT_SECONDS = 5

CW_ANGLE_LIMIT = 6
CCW_ANGLE_LIMIT = 8
TORQUE_ENABLE = 24


BUS_BAUD_VALUES = {1_000_000: 1, 57_600: 34}


def set_bus_baud(packet_handler: PacketHandler, port_handler: PortHandler, baud: int) -> None:
    comm_result, error = packet_handler.write1ByteTxRx(port_handler, GATEWAY_ID, 4, BUS_BAUD_VALUES[baud])
    if comm_result != COMM_SUCCESS or error:
        raise RuntimeError(f"Could not set ArbotiX DYNAMIXEL bus to {baud:,} baud.")


def scan_ids(packet_handler: PacketHandler, port_handler: PortHandler) -> list[int]:
    """Return every actuator whose ID register answers through the ROS sketch."""
    found = []
    for servo_id in range(253):
        value, comm_result, error = packet_handler.read1ByteTxRx(port_handler, servo_id, 3)
        if comm_result == COMM_SUCCESS and not error and value == servo_id:
            found.append(servo_id)
    return found


def wait_for_gateway(packet_handler: PacketHandler, port_handler: PortHandler) -> float:
    """Wait until the ArbotiX controller answers a Protocol 1.0 PING."""
    start = time.monotonic()
    deadline = start + GATEWAY_READY_TIMEOUT_SECONDS
    while time.monotonic() < deadline:
        value, comm_result, error = packet_handler.read1ByteTxRx(port_handler, GATEWAY_ID, 0)
        if comm_result == COMM_SUCCESS and not error and value == 44:
            return time.monotonic() - start
        time.sleep(0.05)
    raise RuntimeError(f"ArbotiX did not answer ID {GATEWAY_ID} within {GATEWAY_READY_TIMEOUT_SECONDS} seconds.")


def checked_read_word(packet_handler: PacketHandler, port_handler: PortHandler, servo_id: int, address: int) -> int:
    value, comm_result, error = packet_handler.read2ByteTxRx(port_handler, servo_id, address)
    if comm_result != COMM_SUCCESS:
        raise RuntimeError(packet_handler.getTxRxResult(comm_result))
    if error:
        raise RuntimeError(packet_handler.getRxPacketError(error))
    return value


def checked_write(packet_handler: PacketHandler, port_handler: PortHandler, servo_id: int, size: int, address: int, value: int) -> None:
    if size == 1:
        comm_result, error = packet_handler.write1ByteTxRx(port_handler, servo_id, address, value)
    else:
        comm_result, error = packet_handler.write2ByteTxRx(port_handler, servo_id, address, value)
    if comm_result != COMM_SUCCESS:
        raise RuntimeError(packet_handler.getTxRxResult(comm_result))
    if error:
        raise RuntimeError(packet_handler.getRxPacketError(error))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--ccw-limit",
        type=int,
        default=4095,
        choices=range(1, 4096),
        metavar="1..4095",
        help="joint-mode upper limit in DYNAMIXEL position units (default: 4095)",
    )
    parser.add_argument("--apply", action="store_true", help="disable torque and persist the selected CCW limit")
    parser.add_argument("--id", type=int, default=SERVO_ID, choices=range(253), metavar="0..252", help="target ID (default: 1)")
    parser.add_argument("--scan-only", action="store_true", help="list responding Protocol 1.0 IDs, then exit")
    args = parser.parse_args()

    port_handler = PortHandler(PORT)
    packet_handler = PacketHandler(PROTOCOL_VERSION)

    try:
        if not port_handler.setBaudRate(HOST_BAUD):
            raise RuntimeError(f"Could not open {PORT} at {HOST_BAUD} baud")
        print("Waiting for the ArbotiX gateway...", flush=True)
        print(f"ArbotiX ready after {wait_for_gateway(packet_handler, port_handler):.1f} seconds.", flush=True)

        found_by_baud = {}
        for bus_baud in BUS_BAUD_VALUES:
            set_bus_baud(packet_handler, port_handler, bus_baud)
            for servo_id in scan_ids(packet_handler, port_handler):
                found_by_baud[servo_id] = bus_baud
        found = sorted(found_by_baud)
        if found:
            print("Detected Protocol 1.0 actuators: " + ", ".join(f"ID {servo_id} at {found_by_baud[servo_id]:,} baud" for servo_id in found))
        else:
            raise RuntimeError("No DYNAMIXEL IDs 0–252 replied during the Protocol 1.0 scan.")
        if args.scan_only:
            set_bus_baud(packet_handler, port_handler, 1_000_000)
            return
        if args.id not in found:
            raise RuntimeError(f"Target ID {args.id} did not reply. Re-run with --id <one of the detected IDs>.")
        set_bus_baud(packet_handler, port_handler, found_by_baud[args.id])

        cw = checked_read_word(packet_handler, port_handler, args.id, CW_ANGLE_LIMIT)
        ccw = checked_read_word(packet_handler, port_handler, args.id, CCW_ANGLE_LIMIT)
        print(f"Current limits: CW={cw}, CCW={ccw}")
        if not args.apply:
            print("No changes made. Re-run with --apply to set joint mode.")
            return

        checked_write(packet_handler, port_handler, args.id, 1, TORQUE_ENABLE, 0)
        checked_write(packet_handler, port_handler, args.id, 2, CCW_ANGLE_LIMIT, args.ccw_limit)
        cw = checked_read_word(packet_handler, port_handler, args.id, CW_ANGLE_LIMIT)
        ccw = checked_read_word(packet_handler, port_handler, args.id, CCW_ANGLE_LIMIT)
        if cw == 0 and ccw == 0:
            raise RuntimeError("Verification failed: the MX-64 is still in wheel mode.")
        if ccw != args.ccw_limit:
            raise RuntimeError(f"Verification failed: expected CCW={args.ccw_limit}, got {ccw}.")
        print(f"Joint mode saved: CW={cw}, CCW={ccw}. Torque remains OFF.")
    finally:
        port_handler.closePort()


if __name__ == "__main__":
    try:
        main()
    except RuntimeError as error:
        print(f"Error: {error}", file=sys.stderr)
        raise SystemExit(1)
