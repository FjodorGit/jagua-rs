use jagua_rs::probs::qpp::entities::{QPInstance, QPSolution};

/// Trait for listeners that can receive solutions during the optimization process
pub trait SolutionListener {
    fn report(&mut self, report: ReportType, solution: &QPSolution, instance: &QPInstance);
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReportType {
    /// Report contains a feasible solution reached by the exploration phase.
    ExplFeas,
    /// Report contains an infeasible solution reached by the exploration phase.
    ExplInfeas,
    /// Report contains an intermediate solution from the exploration phase that is closer to feasibility than the previous one.
    ExplImproving,
    /// Report contains a feasible solution from the comparison phase.
    CmprFeas,
    /// Report contains the final solution
    Final
}

/// A dummy implementation of the `SolutionListener` trait that does nothing.
pub struct DummySolListener;

impl SolutionListener for DummySolListener {
    fn report(&mut self, _report: ReportType, _solution: &QPSolution, _instance: &QPInstance) {
        // Do nothing
    }
}
