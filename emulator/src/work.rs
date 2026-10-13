//! A work queue shared by a battery's worker processes: every worker is
//! given the whole list and claims it a small chunk at a time, by
//! atomically creating a marker file for the chunk (`create_new` fails if
//! another worker got there first). Whoever finishes a chunk takes the next
//! free one, so no worker sits idle while another still has a long tail.

use std::path::{Path, PathBuf};

pub struct Claims {
    dir: PathBuf,
    items: Vec<usize>,
    chunk: usize,
    next_chunk: usize,
    current: std::ops::Range<usize>,
}

impl Claims {
    /// Claim `items` (case indices) from `dir`'s markers, `chunk` at a time.
    pub fn new(dir: &Path, items: Vec<usize>, chunk: usize) -> Self {
        let _ = std::fs::create_dir_all(dir);
        Self { dir: dir.to_owned(), items, chunk: chunk.max(1), next_chunk: 0, current: 0..0 }
    }

    /// A chunk size that splits `total` items into about 50 chunks per
    /// worker: small enough to balance, large enough to keep claims cheap.
    pub fn chunk_for(total: usize, workers: usize) -> usize {
        (total / (workers.max(1) * 50)).clamp(1, 256)
    }
}

impl Iterator for Claims {
    type Item = usize;

    fn next(&mut self) -> Option<usize> {
        loop {
            if let Some(i) = self.current.next() {
                return Some(self.items[i]);
            }
            let start = self.next_chunk * self.chunk;
            if start >= self.items.len() {
                return None;
            }
            let marker = self.dir.join(format!("{}", self.next_chunk));
            self.next_chunk += 1;
            if std::fs::OpenOptions::new().write(true).create_new(true).open(&marker).is_ok() {
                self.current = start..(start + self.chunk).min(self.items.len());
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn two_claimants_share_every_item_exactly_once() {
        let dir = std::env::temp_dir().join(format!("claims_test_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let items: Vec<usize> = (100..1100).collect();
        let mut a = Claims::new(&dir, items.clone(), 7);
        let mut b = Claims::new(&dir, items.clone(), 7);
        let mut seen = Vec::new();
        loop {
            let x = a.next();
            let y = b.next();
            if x.is_none() && y.is_none() {
                break;
            }
            seen.extend(x);
            seen.extend(y);
        }
        seen.sort_unstable();
        assert_eq!(seen, items);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
