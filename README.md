# esp32 Robot Management Unit (RMU)

On-robot embedded micro that will swap the wheels controlled by the PWM signals based on an accelerometer's signal
to determine if the robot is flipped upside down. Based on an ESP32 to also give us future extendability to 
pull off additional sensor readings to aid in competition.

## Tasks
1. Identify ESP32 target board (assuming ESP32-C3 for now for toolchain simplicity)
2. Identify voltage levels of motor control levels for all major components:
    a. Flysky FS-X6B Reciever
    b. $BRAND motor controller
    c. $BRAND IMU
    d. The ESP32 board itself
3. Determine the most feasable way to swap the reciever signal:
    a. ESP32 injests all control signals and then re-emits the signals in the correct direction. (more flexability in later control algorithms)
    b. ESP32 controls an external transister that switches the PWM signals. (more reliable / predictable failure mode)

## Env Setup Steps
Following guides from [esp-rs.org](https://docs.esp-rs.org/book). Using `no_std` for a bare-metal application.

## Build / Run Commands

* `make build`: Builds the project
* `make run`: Deploy binary to ESP32 board
* `make sim`: Deploy binary to Wokwi simulation
