pub mod crew_calls;
pub mod doors_slides;
pub mod galley;
pub mod ife;
pub mod registry;
pub mod waste;
pub mod water;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Zone {
    Fwd,
    Mid,
    Aft,
}

impl Zone {
    pub const ALL: [Zone; 3] = [Zone::Fwd, Zone::Mid, Zone::Aft];
    pub const COUNT: usize = 3;

    pub fn index(self) -> usize {
        match self {
            Zone::Fwd => 0,
            Zone::Mid => 1,
            Zone::Aft => 2,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Zone::Fwd => "FWD",
            Zone::Mid => "MID",
            Zone::Aft => "AFT",
        }
    }
}

impl Default for Zone {
    fn default() -> Self {
        Zone::Fwd
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zone_index_is_stable_and_matches_all() {
        for (i, z) in Zone::ALL.iter().enumerate() {
            assert_eq!(z.index(), i);
        }
        assert_eq!(Zone::ALL.len(), Zone::COUNT);
    }
}

pub mod live;
