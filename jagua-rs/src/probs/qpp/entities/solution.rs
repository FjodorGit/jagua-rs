use crate::Instant;
use crate::entities::LayoutSnapshot;
use crate::probs::qpp::entities::QPInstance;
use crate::probs::qpp::entities::square::Square;

/// Snapshot of [`SPProblem`](crate::probs::spp::entities::SPProblem) at a specific moment. Can be used to restore to a previous state.
#[derive(Debug, Clone)]
pub struct QPSolution {
    pub square: Square,
    pub layout_snapshot: LayoutSnapshot,
    /// Instant the solution was created
    pub time_stamp: Instant,
}

impl QPSolution {
    pub fn density(&self, instance: &QPInstance) -> f32 {
        self.layout_snapshot.density(instance)
    }
    pub fn square_side_length(&self) -> f32 {
        self.square.side_length
    }
}
