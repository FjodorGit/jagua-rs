use crate::collision_detection::CDEConfig;
use crate::entities::Container;
use crate::geometry::primitives::{Rect, SPolygon};
use crate::geometry::shape_modification::{ShapeModifyConfig, ShapeModifyMode};
use crate::geometry::{DTransformation, OriginalShape};
use anyhow::{Result, ensure};

#[derive(Clone, Debug, Copy, PartialEq)]
/// Represents a rectangular container with fixed height and variable width.
pub struct Square {
    pub cde_config: CDEConfig,
    pub shape_modify_config: ShapeModifyConfig,
    pub side_length: f64,
}

impl Square {
    pub fn new(
        cde_config: CDEConfig,
        shape_modify_config: ShapeModifyConfig,
        side_length: f64,
    ) -> Result<Self> {
        ensure!(side_length > 0.0, "strip height must be positive");
        Ok(Self {
            cde_config,
            shape_modify_config,
            side_length,
        })
    }

    pub fn set_side_length(&mut self, side_length: f64) {
        assert!(side_length > 0.0, "strip width must be positive");
        self.side_length = side_length;
    }
}

impl From<Square> for Container {
    fn from(s: Square) -> Container {
        Container::new(
            0,
            OriginalShape {
                shape: SPolygon::from(
                    Rect::try_new(0.0, 0.0, s.side_length, s.side_length).unwrap(),
                ),
                pre_transform: DTransformation::empty(),
                modify_mode: ShapeModifyMode::Deflate,
                modify_config: s.shape_modify_config,
            },
            vec![],
            s.cde_config,
        )
        .unwrap()
    }
}
