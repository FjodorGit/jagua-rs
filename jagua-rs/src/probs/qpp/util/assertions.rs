use crate::entities::Item;
use crate::probs::qpp::entities::{QPProblem, QPSolution};
use crate::util::assertions::layouts_match;

pub fn problem_matches_solution(qpp: &QPProblem, sol: &QPSolution) -> bool {
    let QPSolution {
        square,
        layout_snapshot,
        time_stamp: _,
    } = sol;

    assert_eq!(*square, qpp.square);
    assert_eq!(qpp.density(), sol.density(&qpp.instance));
    assert!(layouts_match(&qpp.layout, layout_snapshot));

    true
}

pub fn instance_item_ids_correct(items: &[(Item, usize)]) -> bool {
    items
        .iter()
        .enumerate()
        .all(|(i, (item, _qty))| item.id == i)
}
