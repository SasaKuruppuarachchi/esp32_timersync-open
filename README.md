# ESP32 Timer Sync Bridge

Free-running ESP32 firmware for the robot sensor stack timing board replacement. The DFRobot FireBeetle 2 ESP32-S3 generates a 10 Hz camera trigger, a 1 Hz sync pulse, and a synthetic GPRMC NMEA stream for downstream consumers.

## System Overview

This board replaces the STM32F103 timing board used in the sensor stack. In Phase 1 it runs without any external time source and produces three outputs:

| Output | Consumer | Purpose |
|---|---|---|
| 10 Hz PWM trigger | Hikrobot MVS camera, Line0 OPTO_IN | Starts image capture on a fixed cadence |
| 1 Hz sync pulse | TTL-to-RS485 module to Livox AVIA LiDAR M12 pin 11/12 | Provides a PPS-like timing reference |
| GPRMC NMEA sentence at 9600 baud on UART1 | Host `/dev/ttyUSBx` to `livox_ros_driver` | Supplies the LiDAR driver with a GPS-like time feed |

## Current Phase

Phase 1: Board bring-up — COMPLETE (verified on bench)

Phase 2: MAVLink link bring-up — ACTIVE (requires PX4 hardware)

Phases 3–5 blocked on Phase 2 completion

## Phase 1 Hardware

Required:

- DFRobot FireBeetle 2 ESP32-S3
- USB-serial adapter for monitoring GPRMC on GPIO 17

Recommended:

- oscilloscope or logic analyzer to verify the 10 Hz and 1 Hz outputs

## Phase 1 Wiring

| Function | GPIO | Destination | Phase 1 note |
|---|---:|---|---|
| 10 Hz camera trigger out | 4 | Hikrobot camera Line0 OPTO_IN | Optional in Phase 1; verify with scope or logic analyzer |
| 1 Hz PPS-like sync out | 5 | TTL-to-RS485 module | Optional in Phase 1; verify with scope or logic analyzer |
| GPRMC UART1 TX | 17 | USB-serial adapter RX | Required for serial verification |
| GPRMC UART1 RX | 18 | Unused | Reserved for future phases |
| Status LED | 2 | On-board LED | Toggled at 0.5 Hz for liveness |

## Phase 2 Wiring

| Function | GPIO | Destination | Note |
|---|---:|---|---|
| MAVLink UART2 TX | 21 | PX4 UART RX (e.g. TELEM2) | 3.3V logic, cross TX→RX |
| MAVLink UART2 RX | 20 | PX4 UART TX | 3.3V logic, cross RX→TX |
| GND | GND | PX4 GND | common ground required |

Note: baud rate is 57600 by default (constant MAVLINK_BAUD in main.cpp). Check PX4 parameter SER_TELx_BAUD for the target port; change the constant if needed (OQ2).

## Build And Flash

```bash
pio run -t upload
pio device monitor -b 115200
```

## Expected USB-CDC Console Output

The actual firmware prints the following USB-CDC banner during setup():

```text
=== ESP32-S3 Phase 1 Timing Firmware ===
Board: DFRobot FireBeetle2 ESP32-S3
10Hz PWM trigger: GPIO 4 (LEDC low-speed timer0, 50% duty)
1Hz sync pulse:   GPIO 5 (esp_timer 500ms toggle, 50% duty)
GPRMC UART1:      TX GPIO 17, RX GPIO 18 @ 9600 baud
Invariant: on each 1Hz rising edge -> LEDC timer reset + monotonic time increment + GPRMC emit
```

Expected 1 Hz log line format:

```text
[1Hz] 00:00:01 - GPRMC emitted, 10Hz phase reset
```

Expected 10 second heartbeat format:

```text
[HB] uptime=10s time=00:00:10
```

## Phase 2 Expected Console Output

- Startup banner now says "Phase 2"
- Heartbeat log unchanged
- New MAVLink log lines:

```text
[MAV] HEARTBEAT sysid=1 compid=1 type=2 autopilot=12 base_mode=0x89 state=4
[MAV] SYSTEM_TIME unix_us=1748476800000000 boot_ms=12345
[MAV] TIMESYNC sent ts1=12345678000 ns
[MAV] TIMESYNC RTT=4500 us  offset_est=-200 us
[MAV] GPS_RAW_INT fix=3 sats=12 time_us=1748476800000000
```

## Expected GPRMC Output On GPIO 17

The UART1 output is a valid NMEA `$GPRMC` sentence at 9600 baud. It uses the Beijing position, a static date, and free-running time starting at 00:00:01.

Example format:

```text
$GPRMC,000001.00,A,2237.496474,N,11356.089515,E,0.0,225.5,230520,2.3,W,A*XX
```

## Phase 1 Verification Checklist

- confirm the board boots and the USB-CDC console opens at 115200 baud
- confirm the status LED toggles at 0.5 Hz
- confirm GPIO 4 produces a 10 Hz square wave with roughly 50% duty cycle
- confirm GPIO 5 produces a 1 Hz square wave with roughly 50% duty cycle
- confirm GPIO 17 emits GPRMC sentences at 9600 baud
- confirm the GPRMC sentence is parseable by the host-side receiver

## Phase 2 Verification Checklist

- Wire ESP32 UART2 (GPIO 20/21) to PX4 telemetry UART with crossed TX/RX and shared GND
- Flash and open USB-CDC monitor at 115200
- Confirm `[MAV] HEARTBEAT` lines appear (proves bidirectional link)
- Confirm `[MAV] SYSTEM_TIME` is received with a non-zero unix_us (proves PX4 has GPS lock)
- Confirm `[MAV] TIMESYNC RTT` shows a round-trip time < 50 ms consistently
- Confirm `[MAV] GPS_RAW_INT fix=3` or higher (3D fix)
- Confirm Phase 1 outputs (1 Hz log, 10 Hz scope) are still working during MAVLink traffic
- Report: HEARTBEAT received Y/N, SYSTEM_TIME unix_us value, typical RTT, GPS fix type

## Open Items

- OQ1: confirm the GPIO pin mapping against the schematic before wiring the final harness
- OQ2: PX4 telemetry baud alignment (default 57600, change MAVLINK_BAUD constant if PX4 port is configured differently)
- OQ5: confirm the host serial device path once the USB-serial adapter is connected, since it may enumerate as `/dev/ttyUSBx` or `/dev/ttyACMx`
