//! Running a closure on a larger stack without leaving the thread.
//!
//! Building the aircraft moves a few hundred kilobytes of systems through
//! several stack frames, more than X-Plane's main thread leaves a plugin
//! (it crashed with a stack overflow, 0xC00000FD). The XPLM calls made while
//! building must stay on that thread, so the closure runs on a Windows fiber:
//! the same thread, with a stack of its own.

use std::ffi::c_void;

/// Stack for the fiber: reserved address space, committed only as used.
pub const STACK_BYTES: usize = 64 * 1024 * 1024;

#[cfg(windows)]
mod win {
    use std::ffi::c_void;

    pub type FiberProc = unsafe extern "system" fn(*mut c_void);

    #[link(name = "kernel32")]
    extern "system" {
        pub fn ConvertThreadToFiber(parameter: *mut c_void) -> *mut c_void;
        pub fn ConvertFiberToThread() -> i32;
        pub fn CreateFiber(stack_size: usize, start: FiberProc, parameter: *mut c_void) -> *mut c_void;
        pub fn DeleteFiber(fiber: *mut c_void);
        pub fn SwitchToFiber(fiber: *mut c_void);
        pub fn IsThreadAFiber() -> i32;
    }

    // GetCurrentFiber is a macro over the TEB in the Windows headers: the
    // fiber data pointer sits at gs:[0x20] on x64.
    #[cfg(target_arch = "x86_64")]
    pub unsafe fn current_fiber() -> *mut c_void {
        let teb: *const *const c_void;
        std::arch::asm!("mov {}, gs:[0x30]", out(reg) teb, options(nostack, readonly, preserves_flags));
        *(teb as *const *mut c_void).add(4)
    }
}

struct Job<F, R> {
    work: Option<F>,
    result: Option<std::thread::Result<R>>,
    back: *mut c_void,
}

/// Runs `work` on a stack of [`STACK_BYTES`] and returns what it returns; a
/// panic inside comes back as `Err`, as with `catch_unwind`. Where a fiber
/// cannot be made it runs on the current stack.
pub fn run<R, F: FnOnce() -> R>(work: F) -> std::thread::Result<R> {
    #[cfg(all(windows, target_arch = "x86_64"))]
    unsafe {
        use win::*;
        unsafe extern "system" fn start<F: FnOnce() -> R, R>(parameter: *mut c_void) {
            let job = &mut *(parameter as *mut Job<F, R>);
            if let Some(work) = job.work.take() {
                job.result = Some(std::panic::catch_unwind(std::panic::AssertUnwindSafe(work)));
            }
            // A fiber must never return from its start routine.
            loop {
                SwitchToFiber(job.back);
            }
        }

        let converted = IsThreadAFiber() == 0;
        let back = if converted { ConvertThreadToFiber(std::ptr::null_mut()) } else { current_fiber() };
        if back.is_null() {
            return std::panic::catch_unwind(std::panic::AssertUnwindSafe(work));
        }
        let mut job = Job { work: Some(work), result: None, back };
        let fiber = CreateFiber(STACK_BYTES, start::<F, R>, &mut job as *mut Job<F, R> as *mut c_void);
        if fiber.is_null() {
            if converted {
                ConvertFiberToThread();
            }
            let work = job.work.take().expect("not run yet");
            return std::panic::catch_unwind(std::panic::AssertUnwindSafe(work));
        }
        SwitchToFiber(fiber);
        DeleteFiber(fiber);
        if converted {
            ConvertFiberToThread();
        }
        job.result.unwrap_or_else(|| Err(Box::new("the fiber did not run")))
    }
    #[cfg(not(all(windows, target_arch = "x86_64")))]
    {
        std::panic::catch_unwind(std::panic::AssertUnwindSafe(work))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn work_that_overflows_a_small_stack_runs_on_the_fiber() {
        // A thread with a 256 KB stack, as tight as a host's main thread can
        // be, building 2 MB on its stack.
        let out = std::thread::Builder::new()
            .stack_size(256 * 1024)
            .spawn(|| {
                run(|| {
                    #[inline(never)]
                    fn big() -> u64 {
                        let block = std::hint::black_box([7u8; 2 * 1024 * 1024]);
                        block.iter().map(|&b| b as u64).sum()
                    }
                    (big(), std::thread::current().id())
                })
                .map(|(sum, id)| (sum, id == std::thread::current().id()))
            })
            .unwrap()
            .join()
            .unwrap();
        let (sum, same_thread) = out.expect("no panic");
        assert_eq!(sum, 7 * 2 * 1024 * 1024);
        assert!(same_thread, "the work stays on the calling thread");
    }

    #[test]
    fn a_panic_comes_back_as_err_and_the_thread_carries_on() {
        let r: std::thread::Result<()> = run(|| panic!("inside"));
        assert!(r.is_err());
        assert_eq!(run(|| 5).ok(), Some(5));
    }
}
