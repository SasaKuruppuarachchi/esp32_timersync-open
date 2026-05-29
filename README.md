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

Phase 1 is board bring-up only:

- free-running operation
- no MAVLink clock discipline
- no PX4 integration yet
- only local verification of GPIO timing and GPRMC output

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

## Open Items

- OQ1: confirm the GPIO pin mapping against the schematic before wiring the final harness
- OQ5: confirm the host serial device path once the USB-serial adapter is connected, since it may enumerate as `/dev/ttyUSBx` or `/dev/ttyACMx`
