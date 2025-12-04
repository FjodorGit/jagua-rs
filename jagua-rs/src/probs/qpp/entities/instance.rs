use crate::entities::{Container, Instance, Item};
use crate::probs::qpp::entities::square::Square;
use crate::probs::qpp::util::assertions;
use std::iter;

#[derive(Debug, Clone)]
/// Instance of the Strip Packing Problem.
pub struct QPInstance {
    /// The items to be packed and their demands
    pub items: Vec<(Item, usize)>,
    /// The strip in which to pack the items
    pub base_square: Square,
}

impl QPInstance {
    pub fn new(items: Vec<(Item, usize)>, base_square: Square) -> Self {
        assert!(
            assertions::instance_item_ids_correct(&items),
            "All items should have consecutive IDs starting from 0"
        );

        Self { items, base_square }
    }

    pub fn item_area(&self) -> f64 {
        self.items
            .iter()
            .map(|(item, qty)| item.shape_orig.area() * *qty as f64)
            .sum()
    }

    pub fn item_qty(&self, id: usize) -> usize {
        self.items[id].1
    }

    pub fn total_item_qty(&self) -> usize {
        self.items.iter().map(|(_, qty)| *qty).sum()
    }
}

impl Instance for QPInstance {
    fn items(&self) -> impl Iterator<Item = &Item> {
        self.items.iter().map(|(item, _qty)| item)
    }

    fn containers(&self) -> impl Iterator<Item = &Container> {
        iter::empty()
    }

    fn item(&self, id: usize) -> &Item {
        &self.items.get(id).unwrap().0
    }

    fn container(&self, _id: usize) -> &Container {
        panic!("no predefined containers for strip packing instances")
    }
}
