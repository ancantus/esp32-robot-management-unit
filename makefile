
build:
	cargo build

sim:
	wokwi-server --chip=esp32c3 target/riscv32imc-unknown-none-elf/debug/esp32-robot-management-unit

run:
	cargo run

clean:
	cargo clean
