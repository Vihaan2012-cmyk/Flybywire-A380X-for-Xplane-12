//! A tiny deterministic PRNG (xorshift64* — Marsaglia 2003, multiplier from
//! Vigna 2016's "splitmix"-style finishers) used only to pick strike
//! targets, bird/hail/lightning classes and random-mode timing in this
//! directory's models. Not cryptographic. `std` has no PRNG at all and the
//! brief forbids external crates (no `rand`), so this directory carries its
//! own; callers own the seed (e.g. from a frame counter) so runs are
//! reproducible for tests.

#[derive(Clone, Copy, Debug)]
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        // xorshift's state must never be zero (it is a fixed point).
        Self(if seed == 0 { 0x9E3779B97F4A7C15 } else { seed })
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// Uniform in `[0, 1)`.
    pub fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }

    /// Uniform integer in `[0, n)`. Returns 0 for `n == 0`.
    pub fn below(&mut self, n: usize) -> usize {
        if n == 0 {
            0
        } else {
            ((self.unit() * n as f64) as usize).min(n - 1)
        }
    }

    /// `true` with probability `p` (`p` clamped to `[0, 1]`).
    pub fn chance(&mut self, p: f64) -> bool {
        self.unit() < p.clamp(0.0, 1.0)
    }

    /// Standard normal sample (Box-Muller), for the turbulence/gust models.
    pub fn normal(&mut self) -> f64 {
        let u1 = self.unit().max(1e-12);
        let u2 = self.unit();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stays_in_unit_range_and_is_deterministic_from_seed() {
        let mut a = Rng::new(42);
        let mut b = Rng::new(42);
        for _ in 0..1000 {
            let (x, y) = (a.unit(), b.unit());
            assert!((0.0..1.0).contains(&x));
            assert_eq!(x, y);
        }
    }

    #[test]
    fn zero_seed_does_not_lock_up() {
        let mut r = Rng::new(0);
        assert!(r.unit() >= 0.0);
        assert!(r.below(5) < 5);
    }

    #[test]
    fn chance_zero_and_one_are_never_and_always() {
        let mut r = Rng::new(7);
        for _ in 0..100 {
            assert!(!r.chance(0.0));
        }
        for _ in 0..100 {
            assert!(r.chance(1.0));
        }
    }

    #[test]
    fn normal_samples_have_roughly_unit_variance() {
        let mut r = Rng::new(123);
        let n = 20_000;
        let sum: f64 = (0..n).map(|_| r.normal()).sum();
        let mean = sum / n as f64;
        assert!(mean.abs() < 0.05, "mean {mean}");
    }
}
