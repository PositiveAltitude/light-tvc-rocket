# Project context

This repository contains the main software stack for a thrust-vector-controlled
(TVC) model rocket. The current code is a prototype migrated from the Light
Robot Core project.

The stack is split into three Rust crates:

- `light-robot-core-embedded`: ESP-IDF firmware for the rocket's ESP32-S3 main
  controller. It manages Wi-Fi, the embedded web server, sensors, LEDs, pyro
  outputs, and CAN communication with the TVC servos.
- `light-robot-core-frontend`: a Yew/WebAssembly control panel. Its compressed
  build output is packaged into the embedded firmware.
- `light-robot-core-api`: shared HTTP state and command schemas used by the
  firmware and frontend.

The custom motor controller is maintained separately in the sibling
`../BLDC-servo-v1` repository. It is an STM32G431-based servo/ESC used by the
rocket's TVC mechanism. Changes to the CAN protocol must be kept compatible
between `light-robot-core-embedded/src/servo_api.rs` here and
`../BLDC-servo-v1/src/can_api.rs`.

This is experimental hardware-control software. Treat motor and pyro behavior
as safety-critical, and do not assume code paths are safe to exercise on
energized hardware without explicit bench-test precautions.
