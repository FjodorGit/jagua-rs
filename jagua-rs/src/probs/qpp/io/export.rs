use crate::Instant;
use crate::io::export::export_layout_snapshot;
use crate::probs::qpp::entities::{QPInstance, QPSolution};
use crate::probs::qpp::io::ext_repr::ExtQPSolution;

/// Exports a solution out of the library
pub fn export(instance: &QPInstance, solution: &QPSolution, epoch: Instant) -> ExtQPSolution {
    ExtQPSolution {
        square_side: solution.square.side_length,
        layout: export_layout_snapshot(&solution.layout_snapshot, instance),
        density: solution.density(instance),
        run_time_sec: solution.time_stamp.duration_since(epoch).as_secs(),
    }
}
