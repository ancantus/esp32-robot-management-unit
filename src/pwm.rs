use bbqueue::{Consumer, Error, Producer};
use core::fmt;
use esp_hal::time::Instant;

#[derive(Clone, Copy)]
pub enum PinEvent {
    RisingEdge(Instant),
    FallingEdge(Instant),
}

impl PinEvent {
    pub const PIN_EVENT_SIZE: usize = 9; // 1 byte for the variant flag & 8 bytes for the timestamp

    const fn descriminant(&self) -> u8 {
        match self {
            Self::RisingEdge(_) => 0u8,
            Self::FallingEdge(_) => 1u8,
        }
    }
}
impl fmt::Display for PinEvent {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::RisingEdge(timestamp) => write!(f, "RisingEdge({})", timestamp),
            Self::FallingEdge(timestamp) => write!(f, "FallingEdge({})", timestamp),
        }
    }
}

pub struct PwmInput<'a, const N: usize> {
    input_queue: Producer<'a, N>,
}

impl<'a, const N: usize> PwmInput<'a, N> {
    pub fn new(prod: Producer<'a, N>) -> PwmInput<'a, N> {
        return PwmInput { input_queue: prod };
    }

    pub fn push(&mut self, event: PinEvent) -> Result<(), Error> {
        let ticks = match event {
            PinEvent::RisingEdge(ts) => ts.ticks().to_le_bytes(),
            PinEvent::FallingEdge(ts) => ts.ticks().to_le_bytes(),
        };
        let mut grant = self.input_queue.grant_exact(PinEvent::PIN_EVENT_SIZE)?;
        grant[0] = event.descriminant();
        grant[1..9].copy_from_slice(&ticks);
        grant.commit(PinEvent::PIN_EVENT_SIZE);
        return Ok(());
    }
}

pub struct PwmOutput<'a, const N: usize> {
    output_queue: Consumer<'a, N>,
}

impl<'a, const N: usize> PwmOutput<'a, N> {
    pub fn new(cons: Consumer<'a, N>) -> PwmOutput<'a, N> {
        return PwmOutput { output_queue: cons };
    }

    pub fn pop(&mut self) -> Result<PinEvent, Error> {
        let grant = self.output_queue.read()?;
        if grant.len() < PinEvent::PIN_EVENT_SIZE {
            return Err(Error::InsufficientSize);
        }
        let event_id = grant[0];
        let timestamp = Instant::from_ticks(u64::from_le_bytes(grant[1..9].try_into().unwrap()));
        grant.release(PinEvent::PIN_EVENT_SIZE);
        return match event_id {
            id if id == PinEvent::RisingEdge(timestamp).descriminant() => {
                Ok(PinEvent::RisingEdge(timestamp))
            }
            id if id == PinEvent::FallingEdge(timestamp).descriminant() => {
                Ok(PinEvent::FallingEdge(timestamp))
            }
            id => panic!("Unknown pin event descriminant {:?}", id),
        };
    }
}
