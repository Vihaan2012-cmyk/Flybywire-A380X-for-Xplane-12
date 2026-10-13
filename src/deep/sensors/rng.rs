//! A tiny, self-contained, deterministic pseudo-random source for this
//! module's sensor noise (multipath, receiver jitter, resolver drift). Not
//! cryptographic; only needs to be repeatable per session so a given fault
//! magnitude always produces the same statistical character.
//!
//! splitmix64 (Vigna, public-domain reference algorithm, `<https://prng.di.unimi.it/splitmix64.c>`)
//! for the bit generator, Box-Muller (Box & Muller, "A Note on the Generation
//! of Random Normal Deviates", Annals of Mathematical Statistics, 1958) for
//! Gaussian noise. Independent, self-contained copy for this directory (see
//! `docs/deep/BRIEF.md` hard rule 1: no dependency on other modules'
//! internals); `src/physics/adirs.rs` has its own equivalent for the same
//! documented reason.

#[derive(Clone, Debug)]
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        // Avoid the all-zero state, which splitmix64 would otherwise get
        // stuck at.
        Self(seed ^ 0x9E37_79B9_7F4A_7C15)
    }

    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in `[0, 1)`.
    pub fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }

    /// Standard normal (mean 0, sigma 1), Box-Muller transform.
    pub fn gaussian(&mut self) -> f64 {
        let u1 = self.next_f64().max(1e-12);
        let u2 = self.next_f64();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_seed_reproduces_the_same_sequence() {
        let mut a = Rng::new(42);
        let mut b = Rng::new(42);
        for _ in 0..10 {
            assert_eq!(a.next_u64(), b.next_u64());
        }
    }

    #[test]
    fn different_seeds_diverge() {
        let mut a = Rng::new(1);
        let mut b = Rng::new(2);
        assert_ne!(a.next_u64(), b.next_u64());
    }

    #[test]
    fn gaussian_draws_are_finite_and_roughly_zero_mean() {
        let mut rng = Rng::new(7);
        let n = 20_000;
        let mut sum = 0.0;
        for _ in 0..n {
            let g = rng.gaussian();
            assert!(g.is_finite());
            sum += g;
        }
        // 1-sigma of the mean of n=20000 unit-variance draws is ~1/sqrt(n)
        // =~0.007; 0.1 is a generous bound against test flakiness.
        assert!((sum / n as f64).abs() < 0.1, "mean {}", sum / n as f64);
    }

    #[test]
    fn uniform_draws_stay_in_zero_one() {
        let mut rng = Rng::new(99);
        for _ in 0..1000 {
            let u = rng.next_f64();
            assert!((0.0..1.0).contains(&u));
        }
    }
}
