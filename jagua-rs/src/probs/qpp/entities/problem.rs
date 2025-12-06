use crate::Instant;
use crate::entities::{Instance, Layout, PItemKey};
use crate::geometry::DTransformation;
use crate::probs::qpp::entities::{QPInstance, QPSolution, Square};
use crate::probs::qpp::util::assertions::problem_matches_solution;
use itertools::Itertools;

/// Modifiable counterpart of [`SPInstance`]: items can be placed and removed, strip can be extended or fitted.
#[derive(Clone)]
pub struct QPProblem {
    pub instance: QPInstance,
    pub square: Square,
    pub layout: Layout,
    pub item_demand_qtys: Vec<usize>,
}

impl QPProblem {
    pub fn new(instance: QPInstance) -> Self {
        let item_demand_qtys = instance.items.iter().map(|(_, qty)| *qty).collect_vec();
        let square = instance.base_square;
        let layout = Layout::new(square.into());

        Self {
            instance,
            square,
            layout,
            item_demand_qtys,
        }
    }

    /// Modifies the width of the strip in the back, keeping the front fixed.
    pub fn change_square_side_length(&mut self, new_width: f64) {
        self.square.set_side_length(new_width);
        self.layout.swap_container(self.square.into());
    }

    /// Shrinks the strip to the minimum width that fits all items.
    pub fn fit_square(&mut self) {
        let feasible_before = self.layout.is_feasible();

        //Find the rightmost item in the strip and add some tolerance (avoiding false collision positives)
        let item_x_max = self
            .layout
            .placed_items
            .values()
            .map(|pi| pi.shape.bbox.x_max)
            .max_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap()
            * 1.00001;

        let item_y_max = self
            .layout
            .placed_items
            .values()
            .map(|pi| pi.shape.bbox.y_max)
            .max_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap()
            * 1.00001;

        // add the shape offset if any, the strip needs to be at least `offset` wider than the items
        let fitted_width = item_x_max + self.square.shape_modify_config.offset.unwrap_or(0.0);
        let fitted_height = item_y_max + self.square.shape_modify_config.offset.unwrap_or(0.0);

        self.change_square_side_length(fitted_width.max(fitted_height));
        debug_assert!(feasible_before == self.layout.is_feasible());
    }

    /// Places an item according to the given `SPPlacement` in the problem.
    pub fn place_item(&mut self, placement: QPPlacement) -> PItemKey {
        self.register_included_item(placement.item_id);
        let item = self.instance.item(placement.item_id);

        self.layout.place_item(item, placement.d_transf)
    }

    /// Removes a placed item from the strip. Returns the placement of the item.
    /// Set `commit_instantly` to false if there's a high chance that this modification will be reverted.
    pub fn remove_item(&mut self, pkey: PItemKey) -> QPPlacement {
        let pi = self.layout.remove_item(pkey);
        self.deregister_included_item(pi.item_id);

        QPPlacement {
            item_id: pi.item_id,
            d_transf: pi.d_transf,
        }
    }

    /// Creates a snapshot of the current state of the problem as a [`SPSolution`].
    pub fn save(&self) -> QPSolution {
        let solution = QPSolution {
            layout_snapshot: self.layout.save(),
            square: self.square,
            time_stamp: Instant::now(),
        };

        debug_assert!(problem_matches_solution(self, &solution));

        solution
    }

    /// Restores the state of the problem to the given [`SPSolution`].
    pub fn restore(&mut self, solution: &QPSolution) {
        if self.square == solution.square {
            // the square is the same, restore the layout
            self.layout.restore(&solution.layout_snapshot);
        } else {
            // the square has changed, rebuild the layout
            self.layout = Layout::from_snapshot(&solution.layout_snapshot);
            self.square = solution.square;
        }

        //Restore the item demands
        {
            self.item_demand_qtys
                .iter_mut()
                .enumerate()
                .for_each(|(id, qty)| *qty = self.instance.item_qty(id));

            self.layout
                .placed_items
                .iter()
                .for_each(|(_, pi)| self.item_demand_qtys[pi.item_id] -= 1);
        }
        debug_assert!(problem_matches_solution(self, solution));
    }

    fn register_included_item(&mut self, item_id: usize) {
        self.item_demand_qtys[item_id] -= 1;
    }

    fn deregister_included_item(&mut self, item_id: usize) {
        self.item_demand_qtys[item_id] += 1;
    }

    pub fn density(&self) -> f64 {
        self.layout.density(&self.instance)
    }

    pub fn side_length(&self) -> f64 {
        self.square.side_length
    }
}

/// Represents a placement of an item in the strip packing problem.
#[derive(Debug, Clone, Copy)]
pub struct QPPlacement {
    pub item_id: usize,
    pub d_transf: DTransformation,
}
