# ArbotiX-M firmware

`arbotix_ros/` contains the exact firmware source flashed to the connected
ArbotiX-M on 2026-08-31.

## Provenance

- Upstream: <https://github.com/trossenrobotics/arbotix>
- Upstream revision: `76e6665f09bb5c7f3703eea4c4da6b147fc139a4`
- Sketch: `arbotix_ros/ros/ros.ino`
- Target: ArbotiX-M, ATmega644P, Arduino bootloader at 38,400 baud

The upstream source uses the BSD license text in the ROS sketch header.

## Local compatibility change

Modern AVR libc no longer defines `prog_char`. In
`arbotix_ros/hardware/arbotix/cores/arbotix/Print.cpp`, the type was changed
to `char`. This is the only source change from the upstream revision and is
required to compile with Arduino CLI 1.5.1 / the installed AVR toolchain.

## Verified runtime configuration

- Mac-to-ArbotiX host serial: 115,200 baud
- DYNAMIXEL bus on startup: 1,000,000 baud
- Protocol: DYNAMIXEL Protocol 1.0
- Discovered actuators: IDs 1 and 2, both at 1,000,000 baud

The firmware must remain paired with the Python app's ArbotiX ROS transport;
it is not a transparent USB-to-DYNAMIXEL adapter.
