use crate::config::LBFConfig;
use jagua_rs::io::ext_repr::ExtLayout;
use jagua_rs::probs::bpp::io::ext_repr::{ExtBPInstance, ExtBPSolution};
use jagua_rs::probs::qpp::io::ext_repr::{ExtQPInstance, ExtQPSolution};
use jagua_rs::probs::spp::io::ext_repr::{ExtSPInstance, ExtSPSolution};
use serde::{Deserialize, Serialize};

/// Scaling factor applied to input coordinates.
/// Coordinates are scaled UP by this factor on import,
/// and scaled DOWN by this factor on export to maintain precision.
pub const COORDINATE_SCALE_FACTOR: f32 = 10_000.0;

#[derive(Serialize, Deserialize, Clone)]
pub struct SPOutput {
    #[serde(flatten)]
    pub instance: ExtSPInstance,
    pub solution: ExtSPSolution,
    pub config: LBFConfig,
}

#[derive(Serialize, Deserialize, Clone)]
pub struct BPOutput {
    #[serde(flatten)]
    pub instance: ExtBPInstance,
    pub solution: ExtBPSolution,
    pub config: LBFConfig,
}

#[derive(Serialize, Deserialize, Clone)]
pub struct QPOutput {
    #[serde(flatten)]
    pub instance: ExtQPInstance,
    pub solution: ExtQPSolution,
    pub config: LBFConfig,
}

#[derive(Serialize, Deserialize, Clone)]
pub struct CSVPlacedItem {
    pub item_id: u64,
    pub reference_point_x: f32,
    pub reference_point_y: f32,
    pub rotation_degrees: f32,
}

#[derive(Serialize, Deserialize, Clone)]
pub struct CombinedCSVItem {
    pub id: String,
    pub x: f32,
    pub y: f32,
    pub deg: f32,
}

pub fn layout_to_csv(layout: &ExtLayout) -> Vec<CSVPlacedItem> {
    layout
        .placed_items
        .iter()
        .map(|pi| {
            // Apply reverse scaling to convert from internal scaled coordinates
            // back to original coordinate space
            let x_unscaled = pi.transformation.translation.0 / COORDINATE_SCALE_FACTOR;
            let y_unscaled = pi.transformation.translation.1 / COORDINATE_SCALE_FACTOR;
            
            CSVPlacedItem {
                item_id: pi.item_id,
                reference_point_x: x_unscaled,
                reference_point_y: y_unscaled,
                rotation_degrees: pi.transformation.rotation.to_degrees(),
            }
        })
        .collect()
}
