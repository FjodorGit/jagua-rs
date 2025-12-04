use crate::entities::Item;
use crate::geometry::shape_modification::ShapeModifyConfig;
use crate::io::import::Importer;
use crate::probs::qpp::entities::{QPInstance, Square};
use crate::probs::qpp::io::ext_repr::ExtQPInstance;
use anyhow::{Result, ensure};
use itertools::Itertools;
use rayon::prelude::*;

/// Imports an instance into the library
pub fn import(importer: &Importer, ext_instance: &ExtQPInstance) -> Result<QPInstance> {
    let items: Vec<(Item, usize)> = {
        let mut items = ext_instance
            .items
            .par_iter()
            .map(|ext_item| {
                let item = importer.import_item(&ext_item.base)?;
                let demand = ext_item.demand as usize;
                Ok((item, demand))
            })
            .collect::<Result<Vec<(Item, usize)>>>()?;

        items.sort_by_key(|(item, _)| item.id);
        items.retain(|(_, demand)| *demand > 0);

        ensure!(
            items.iter().enumerate().all(|(i, (item, _))| item.id == i),
            "All items should have consecutive IDs starting from 0. IDs: {:?}",
            items.iter().map(|(item, _)| item.id).sorted().collect_vec()
        );
        ensure!(
            !items.is_empty(),
            "ExtSPInstance must have at least one item with positive demand"
        );

        items
    };

    let total_item_area = items
        .iter()
        .map(|(item, demand)| item.area() * *demand as f64)
        .sum::<f64>();

    // Initialize the base width for 100% density
    let side_length = total_item_area.sqrt();

    let base_square = Square::new(
        importer.cde_config,
        ShapeModifyConfig {
            offset: importer.shape_modify_config.offset,
            simplify_tolerance: None,
            narrow_concavity_cutoff_ratio: None,
        },
        side_length,
    )?;

    Ok(QPInstance::new(items, base_square))
}
