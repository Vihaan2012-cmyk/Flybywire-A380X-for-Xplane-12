pub struct Progress {
    pub systems: bool,
    pub screens_up: usize,
    pub screens: usize,
}

impl Progress {
    pub fn complete(&self) -> bool {
        self.systems && self.screens_up >= self.screens
    }

    pub fn percent(&self) -> f32 {
        let screens = if self.screens == 0 { 1. } else { self.screens_up as f32 / self.screens as f32 };
        f32::from(u8::from(self.systems)) * 10. + screens * 80. + if self.complete() { 10. } else { 0. }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn progress_reaches_a_hundred_only_when_every_screen_is_up() {
        let p = Progress { systems: true, screens_up: 9, screens: 18 };
        assert!(!p.complete());
        assert_eq!(p.percent(), 50.);
        let p = Progress { systems: true, screens_up: 18, screens: 18 };
        assert!(p.complete());
        assert_eq!(p.percent(), 100.);
        assert_eq!(Progress { systems: false, screens_up: 0, screens: 18 }.percent(), 0.);
    }
}
