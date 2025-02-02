#![no_std]
#![no_main]

use bbqueue::BBBuffer;
use core::cell::RefCell;
use critical_section::Mutex;
use esp32_robot_management_unit::{PinEvent, PwmInput, PwmOutput};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::interrupt::InterruptConfigurable;
use esp_hal::time::Instant;
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

const PWM_QUEUE_SIZE: usize = PinEvent::PIN_EVENT_SIZE * 10;
static LEFT_PWM_QUEUE: BBBuffer<PWM_QUEUE_SIZE> = BBBuffer::new();
static LEFT_PWM_INPUT_QUEUE: Mutex<RefCell<Option<PwmInput<'static, PWM_QUEUE_SIZE>>>> =
    Mutex::new(RefCell::new(None));

static RIGHT_PWM_QUEUE: BBBuffer<PWM_QUEUE_SIZE> = BBBuffer::new();
static RIGHT_PWM_INPUT_QUEUE: Mutex<RefCell<Option<PwmInput<'static, PWM_QUEUE_SIZE>>>> =
    Mutex::new(RefCell::new(None));

fn register_pwm_input_queues() -> (
    PwmOutput<'static, PWM_QUEUE_SIZE>,
    PwmOutput<'static, PWM_QUEUE_SIZE>,
) {
    // split the bbqueues into producer & consumers
    // SAFETY: unwrap ok here as panic is the only valid response to calling this regisration twice
    // unit / bench tests should catch any obvious mistake.
    let (left_prod, left_cons) = LEFT_PWM_QUEUE.try_split().unwrap();
    let (right_prod, right_cons) = RIGHT_PWM_QUEUE.try_split().unwrap();

    // register the producer side of the queue up into static variables visable to the ISR
    critical_section::with(|cs| {
        LEFT_PWM_INPUT_QUEUE
            .borrow_ref_mut(cs)
            .replace(PwmInput::new(left_prod));
        RIGHT_PWM_INPUT_QUEUE
            .borrow_ref_mut(cs)
            .replace(PwmInput::new(right_prod));
    });

    return (PwmOutput::new(left_cons), PwmOutput::new(right_cons));
}

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

    let (mut left_pwm_events, mut right_pwm_events) = register_pwm_input_queues();

    // bind the interrupt handler so GPIO interrupts are captured
    gpio_io.set_interrupt_handler(gpio_handler);

    let delay = Delay::new();
    loop {
        match left_pwm_events.pop() {
            Ok(event) => info!("Left Input: {}", event),
            Err(_) => (),
        };
        match right_pwm_events.pop() {
            Ok(event) => info!("Right Input: {}", event),
            Err(_) => (),
        };
        // info!("Hello world!");
        delay.delay_millis(1);
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/v0.23.1/examples/src/bin
}

#[inline]
fn read_pin_event(pin: &Input, timestamp: &Instant) -> Option<PinEvent> {
    let level = match pin.is_interrupt_set() {
        false => return None,
        true => pin.level(),
    };
    return match level {
        Level::Low => Some(PinEvent::FallingEdge(timestamp.clone())),
        Level::High => Some(PinEvent::RisingEdge(timestamp.clone())),
    };
}

#[inline]
fn irq_queue_gpio_event<const N: usize>(
    queue: &mut PwmInput<'_, N>,
    pin: &mut Input<'_>,
    timestamp: &Instant,
) {
    // read the pin's event and clear interrupts if applicable
    let event = match read_pin_event(pin, timestamp) {
        None => return (),
        Some(event) => {
            pin.clear_interrupt();
            event
        }
    };

    // attempt to push to the queue three times before throwing out the event.
    // TODO: test on real hardware to make sure this is a practical & safe upper limit
    for _ in 0..3 {
        match queue.push(event.clone()) {
            Ok(_) => return (),
            Err(_) => (),
        };
    }
}

#[handler]
fn gpio_handler() {
    // save off the left & right pin inputs within the critical section
    // this section should also take care to reset the interrupt of any set pins
    critical_section::with(|cs| {
        // SAFETY: timer is globally registered before interrupt is active
        // but a 0 interval is used if something's been set up wrong instead of panicing in an ISR
        let timestamp = match GLOBAL_TIMER.borrow_ref_mut(cs).as_mut() {
            Some(timer) => timer.now(),
            None => Instant::from_ticks(0u64),
        };
        match (
            LEFT_DRIVE_INPUT.borrow_ref_mut(cs).as_mut(),
            LEFT_PWM_INPUT_QUEUE.borrow_ref_mut(cs).as_mut(),
        ) {
            (Some(pin), Some(queue)) => irq_queue_gpio_event(queue, pin, &timestamp),
            (Some(pin), None) => pin.clear_interrupt(), // make sure we don't latch interupts
            _ => (),
        };
        match (
            RIGHT_DRIVE_INPUT.borrow_ref_mut(cs).as_mut(),
            RIGHT_PWM_INPUT_QUEUE.borrow_ref_mut(cs).as_mut(),
        ) {
            (Some(pin), Some(queue)) => irq_queue_gpio_event(queue, pin, &timestamp),
            (Some(pin), None) => pin.clear_interrupt(), // make sure we don't latch interupts
            _ => (),
        };
        // TODO fix up the sharing / ownership model of the queues so the push doesn't happen in a
        // critical section. The queues do not need locks but the lazy loading mechanism
        // 'technically' does.
    });
}
