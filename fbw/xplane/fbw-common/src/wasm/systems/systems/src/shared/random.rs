#[cfg(any(target_arch = "wasm32", doc))]
pub use wasm::*;

#[cfg(any(target_arch = "wasm32", doc))]
mod wasm {
    use rand::rngs::SmallRng;
    use rand::{Rng, SeedableRng};
    use rand_distr::{Distribution, Normal};
    use std::mem::MaybeUninit;
    use std::sync::Once;

    static RAND_INIT: Once = Once::new();
    static mut RAND: MaybeUninit<SmallRng> = MaybeUninit::uninit();

    pub fn random_number() -> u8 {
        // SAFETY: WASM is single-threaded, and we're not passing references to `RAND` around.
        RAND_INIT.call_once(|| unsafe {
            RAND = MaybeUninit::new(SmallRng::from_os_rng());
        });

        // SAFETY: `RAND` was initialized above.
        unsafe { (*RAND.as_mut_ptr()).random() }
    }

    pub fn random_from_range(from: f64, to: f64) -> f64 {
        // SAFETY: WASM is single-threaded, and we're not passing references to `RAND` around.
        RAND_INIT.call_once(|| unsafe {
            RAND = MaybeUninit::new(SmallRng::from_os_rng());
        });

        // SAFETY: `RAND` was initialized above.
        unsafe { (*RAND.as_mut_ptr()).random_range(from..to) }
    }

    // Generates a random number based on normal distribution
    pub fn random_from_normal_distribution(mean: f64, std_dev: f64) -> f64 {
        // SAFETY: WASM is single-threaded, and we're not passing references to `RAND` around.
        RAND_INIT.call_once(|| unsafe {
            RAND = MaybeUninit::new(SmallRng::from_os_rng());
        });

        let normal = Normal::new(mean, std_dev).unwrap();
        let limit_offset = 4. * std_dev;

        // SAFETY: `RAND` was initialized above.
        unsafe {
            normal
                .sample(&mut *RAND.as_mut_ptr())
                .max(mean - limit_offset)
                .min(mean + limit_offset)
        }
    }
}

#[cfg(not(any(target_arch = "wasm32", doc)))]
pub use not_wasm::*;

#[cfg(not(any(target_arch = "wasm32", doc)))]
mod not_wasm {
    use rand::rngs::SmallRng;
    use rand::{Rng, SeedableRng};
    use rand_distr::{Distribution, Normal};
    use std::cell::RefCell;

    // One seeded generator per thread: a simulation is reproducible from its
    // seed, and its generator state can be saved and restored with the rest
    // of it (`random_state`/`set_random_state`).
    thread_local! {
        static RNG: RefCell<SmallRng> = RefCell::new(SmallRng::seed_from_u64(0x5EED_A380));
    }

    /// Restart this thread's random numbers from `seed`.
    pub fn seed_random(seed: u64) {
        RNG.with(|r| *r.borrow_mut() = SmallRng::seed_from_u64(seed));
    }

    /// This thread's generator, to restore later with [`set_random_state`].
    pub fn random_state() -> SmallRng {
        RNG.with(|r| r.borrow().clone())
    }

    pub fn set_random_state(state: SmallRng) {
        RNG.with(|r| *r.borrow_mut() = state);
    }

    pub fn random_number() -> u8 {
        RNG.with(|r| r.borrow_mut().random())
    }

    pub fn random_from_range(from: f64, to: f64) -> f64 {
        RNG.with(|r| r.borrow_mut().random_range(from..to))
    }

    /// Random value from normal distribution. Output limited to -4 / +4 sigma
    pub fn random_from_normal_distribution(mean: f64, std_dev: f64) -> f64 {
        let normal = Normal::new(mean, std_dev).unwrap();
        let limit_offset = 4. * std_dev;
        RNG.with(|r| normal.sample(&mut *r.borrow_mut()))
            .max(mean - limit_offset)
            .min(mean + limit_offset)
    }
}
