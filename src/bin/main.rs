#![no_std]
#![no_main]

use bbqueue::BBBuffer;
use core::cell::RefCell;
use critical_section::Mutex;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::interrupt::InterruptConfigurable;
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::timer::{AnyTimer, Timer};
use esp_hal::{
    gpio::{Event, Input, Io, Level, Pull},
    handler, main,
};
use log::info;

extern crate alloc;

static GLOBAL_TIMER: Mutex<RefCell<Option<AnyTimer>>> = Mutex::new(RefCell::new(None));

static LEFT_DRIVE_INPUT: Mutex<RefCell<Option<Input>>> = Mutex::new(RefCell::new(None));
static RIGHT_DRIVE_INPUT: Mutex<RefCell<Option<Input>>> = Mutex::new(RefCell::new(None));

static BB: BBBuffer<6> = BBBuffer::new();

#[main]
fn main() -> ! {
    // generator version: 0.2.2

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_println::logger::init_logger_from_env();

    // Initalize the WiFi subsystem
    esp_alloc::heap_allocator!(72 * 1024);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let _init = esp_wifi::init(
        timg0.timer0,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    let mut gpio_io = Io::new(peripherals.IO_MUX);

    let (mut prod, mut cons) = BB.try_split().unwrap();

    // Initalize the PWM Input subsystem
    let sys_clock = SystemTimer::new(peripherals.SYSTIMER);
    let mut gpio9 = Input::new(peripherals.GPIO9, Pull::None);
    let mut gpio8 = Input::new(peripherals.GPIO8, Pull::None);
    critical_section::with(|cs| {
        sys_clock.alarm0.start();
        GLOBAL_TIMER
            .borrow_ref_mut(cs)
            .replace(AnyTimer::from(sys_clock.alarm0));

        gpio9.listen(Event::AnyEdge);
        LEFT_DRIVE_INPUT.borrow_ref_mut(cs).replace(gpio9);

        gpio8.listen(Event::AnyEdge);
        RIGHT_DRIVE_INPUT.borrow_ref_mut(cs).replace(gpio8);
    });

    // bind the interrupt handler so GPIO interrupts are captured
    gpio_io.set_interrupt_handler(gpio_handler);

    let delay = Delay::new();
    loop {
        info!("Hello world!");
        delay.delay_millis(500);
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/v0.23.1/examples/src/bin
}

enum PinEvent {
    RisingEdge(Instant<u64, 1, 1000000>),
    FallingEdge(Instant<u64, 1, 1000000>),
}

fn read_pin_event(pin: &Input, timestamp: &Instant<u64, 1, 1000000>) -> Option<PinEvent> {
    let level = match pin.is_interrupt_set() {
        false => return None,
        true => pin.level(),
    };
    return match level {
        Level::Low => Some(PinEvent::FallingEdge(timestamp)),
        Level::High => Some(PinEvent::RisingEdge(timestamp)),
    };
}

#[handler]
fn gpio_handler() {
    let mut left_input: Option<PinEvent> = None;
    let mut right_input: Option<PinEvent> = None;

    // save off the left & right pin inputs within the critical section
    // this section should also take care to reset the interrupt of any set pins
    critical_section::with(|cs| {
        let timestamp = GLOBAL_TIMER.borrow_ref_mut(cs).as_mut().unwrap().now();

        left_input = match LEFT_DRIVE_INPUT.borrow_ref_mut(cs).as_mut() {
            Some(pin) => match read_pin_event(pin, timestamp) {
                None => None,
                Some(event) => {
                    pin.clear_interrupt();
                    Some(event)
                }
            },
            None => None,
        };
        right_input = match RIGHT_DRIVE_INPUT.borrow_ref_mut(cs).as_mut() {
            Some(pin) => match read_pin_event(pin, timestamp) {
                None => None,
                Some(event) => {
                    pin.clear_interrupt();
                    Some(event)
                }
            },
            None => None,
        };
    });
    // TODO: store left & right PWM event into a queue with the timestamp for the main loop to
    // pop and convert to calculate the pulse width / duty cycle.
    // Both pins need to be read because the handler is triggered for either pin.
}
