//! Production timer-alarm sleep. No virtual-platform-specific firmware path.
use core::sync::atomic::{AtomicBool, Ordering};
use rp2040_hal::{
    pac,
    pac::interrupt,
    timer::{Alarm, Alarm0, Timer},
};

static COMPLETE: AtomicBool = AtomicBool::new(false);

pub struct PeriodWait(Alarm0);

impl PeriodWait {
    pub fn new(timer: &mut Timer) -> Self {
        let mut alarm = timer.alarm_0().expect("control period owns alarm 0");
        alarm.clear_interrupt();
        alarm.enable_interrupt();
        // Safety: handler clears only alarm 0's interrupt and publishes COMPLETE.
        unsafe { cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0) };
        Self(alarm)
    }

    pub fn wait(&mut self, micros: u32) {
        COMPLETE.store(false, Ordering::Release);
        self.0
            .schedule(fugit::MicrosDurationU32::micros(micros))
            .expect("control period fits the hardware alarm horizon");
        while !COMPLETE.load(Ordering::Acquire) {
            // SEV in the ISR closes the check-to-sleep race. Unrelated events
            // are harmless: always recheck the completion flag after waking.
            cortex_m::asm::wfe();
        }
    }
}

#[interrupt]
fn TIMER_IRQ_0() {
    // Safety: W1C touches only alarm 0's pending flag. Alarm scheduling remains
    // owned by PeriodWait in thread mode; no shared mutable Rust reference.
    unsafe {
        let timer = &*pac::TIMER::ptr();
        // HAL forces INTF if the deadline passes during scheduling. The
        // RP2040 +0x3000 atomic-clear alias touches only alarm 0's bit.
        core::ptr::write_volatile(timer.intf().as_ptr().byte_add(0x3000), 1);
        timer.intr().write(|w| w.alarm_0().clear_bit_by_one());
    };
    COMPLETE.store(true, Ordering::Release);
    cortex_m::asm::sev();
}
