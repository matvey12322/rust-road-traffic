//! TEMPORARY per-stage timing instrumentation for the frame loop.
//!
//! Compiled only when the build defines `--cfg perf_stages`, e.g.
//! `RUSTFLAGS="--cfg perf_stages" cargo build --release ...`.
//! Without that cfg every method is a no-op and this module costs nothing.
//! To remove the instrumentation: delete this file, its `mod` line in
//! `lib/mod.rs` and the `perf.*` call sites in `main.rs`.
use std::time::Instant;

const REPORT_EVERY_SECS: f64 = 5.0;

pub struct PerfStages {
    #[cfg(perf_stages)]
    inner: Option<Inner>,
}

#[cfg(perf_stages)]
struct StageAcc {
    total: f64,
    max: f32,
    count: u64,
    samples: Vec<f32>,
}

#[cfg(perf_stages)]
struct Inner {
    stages: std::collections::HashMap<&'static str, StageAcc>,
    order: Vec<&'static str>,
    counters: std::collections::HashMap<&'static str, u64>,
    frame_start: Option<Instant>,
    cur_stage: Option<(&'static str, Instant)>,
    frame_total: f64,
    frame_count: u64,
    frame_samples: Vec<f64>,
    last_report: Instant,
}

impl PerfStages {
    pub fn new() -> Self {
        #[cfg(perf_stages)]
        {
            Self {
                inner: Some(Inner {
                    stages: std::collections::HashMap::new(),
                    order: Vec::new(),
                    counters: std::collections::HashMap::new(),
                    frame_start: None,
                    cur_stage: None,
                    frame_total: 0.0,
                    frame_count: 0,
                    frame_samples: Vec::new(),
                    last_report: Instant::now(),
                }),
            }
        }
        #[cfg(not(perf_stages))]
        {
            Self {}
        }
    }

    /// Called at the very top of each frame loop iteration. Also closes a
    /// frame that ended abnormally (e.g. the loop `continue`d out).
    pub fn begin_frame(&mut self) {
        #[cfg(perf_stages)]
        {
            if let Some(inner) = self.inner.as_mut() {
                // Close a frame that was never ended (loop `continue`).
                inner.end_frame();
                inner.frame_start = Some(Instant::now());
            }
        }
    }

    /// Closes the previous stage and opens `name`.
    pub fn stage(&mut self, name: &'static str) {
        #[cfg(perf_stages)]
        {
            if let Some(inner) = self.inner.as_mut() {
                inner.close_stage();
                inner.cur_stage = Some((name, Instant::now()));
                inner.maybe_report();
            }
        }
    }

    /// Adds `n` to a named counter (e.g. tracks per frame).
    pub fn count(&mut self, name: &'static str, n: u64) {
        #[cfg(perf_stages)]
        {
            if let Some(inner) = self.inner.as_mut() {
                *inner.counters.entry(name).or_insert(0) += n;
            }
        }
    }

    /// Called at the end of a normal frame loop iteration.
    pub fn end_frame(&mut self) {
        #[cfg(perf_stages)]
        {
            if let Some(inner) = self.inner.as_mut() {
                inner.end_frame();
                inner.maybe_report();
            }
        }
    }
}

#[cfg(perf_stages)]
impl Inner {
    fn close_stage(&mut self) {
        if let Some((name, start)) = self.cur_stage.take() {
            let secs = start.elapsed().as_secs_f32();
            let acc = self.stages.entry(name).or_insert_with(|| {
                self.order.push(name);
                StageAcc {
                    total: 0.0,
                    max: 0.0,
                    count: 0,
                    samples: Vec::new(),
                }
            });
            acc.total += secs as f64;
            acc.max = acc.max.max(secs);
            acc.count += 1;
            acc.samples.push(secs);
        }
    }

    fn end_frame(&mut self) {
        self.close_stage();
        if let Some(start) = self.frame_start.take() {
            let secs = start.elapsed().as_secs_f64();
            self.frame_total += secs;
            self.frame_count += 1;
            self.frame_samples.push(secs);
        }
    }

    fn maybe_report(&mut self) {
        if self.last_report.elapsed().as_secs_f64() < REPORT_EVERY_SECS {
            return;
        }
        self.last_report = Instant::now();

        let n = self.frame_count.max(1) as f64;
        let avg_frame_ms = self.frame_total * 1000.0 / n;
        let mut frame_samples = self.frame_samples.clone();
        frame_samples.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let frame_p95_ms = frame_samples
            .get((frame_samples.len().saturating_sub(1)).saturating_mul(95) / 100)
            .copied()
            .unwrap_or(0.0)
            * 1000.0;
        let frame_max_ms = frame_samples.last().copied().unwrap_or(0.0) * 1000.0;

        let mut parts: Vec<String> = Vec::new();
        parts.push(format!(
            "frames={} frame_avg={:.2}ms frame_p95={:.2}ms frame_max={:.2}ms fps={:.1}",
            self.frame_count,
            avg_frame_ms,
            frame_p95_ms,
            frame_max_ms,
            if avg_frame_ms > 0.0 {
                1000.0 / avg_frame_ms
            } else {
                0.0
            }
        ));
        for name in &self.order {
            if let Some(acc) = self.stages.get(name) {
                let avg = acc.total * 1000.0 / acc.count.max(1) as f64;
                let mut samples = acc.samples.clone();
                samples.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
                let p95 = samples
                    .get((samples.len().saturating_sub(1)).saturating_mul(95) / 100)
                    .copied()
                    .unwrap_or(0.0)
                    * 1000.0;
                parts.push(format!(
                    "{name}: avg={avg:.3}ms p95={p95:.3}ms max={:.3}ms",
                    acc.max * 1000.0
                ));
            }
        }
        let mut counters: Vec<(&&str, &u64)> = self.counters.iter().collect();
        counters.sort_by_key(|(name, _)| **name);
        let counter_str = counters
            .iter()
            .map(|(name, value)| format!("{name}={value}"))
            .collect::<Vec<_>>()
            .join(" ");
        tracing::info!(
            scope = crate::lib::logging::SCOPE_PROCESSING,
            "PerfStages {} {}",
            parts.join(" "),
            counter_str
        );

        // Reset the window.
        for acc in self.stages.values_mut() {
            acc.total = 0.0;
            acc.max = 0.0;
            acc.count = 0;
            acc.samples.clear();
        }
        self.frame_total = 0.0;
        self.frame_count = 0;
        self.frame_samples.clear();
        self.counters.clear();
    }
}
