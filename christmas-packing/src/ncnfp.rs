use nfp::{NFPConvex, Point};

use crate::tree::Tree;

pub struct NcNfp {
    pub i_piece_idx: usize,
    pub j_piece_idx: usize,
    pub indentical_pieces: bool,
    sub_nfps: Vec<Nfp>,
}
impl NcNfp {
    pub fn new(nfps: Vec<Nfp>, i: usize, j: usize, identical: bool) -> Self {
        Self {
            sub_nfps: nfps,
            indentical_pieces: identical,
            i_piece_idx: i,
            j_piece_idx: j,
        }
    }

    pub fn sub_nfps(&self) -> impl Iterator<Item = &Nfp> {
        self.sub_nfps.iter()
    }

    pub fn as_sub_nfps(self) -> Vec<Nfp> {
        self.sub_nfps
    }
}

#[derive(Debug, Clone)]
pub struct Nfp {
    pub i_piece_idx: usize,
    pub j_piece_idx: usize,
    pub f_conv_piece_idx: usize,
    pub g_conv_piece_idx: usize,
    points: Vec<Point>,
}

impl Nfp {
    pub fn new(mut points: Vec<Point>, i: usize, j: usize, f: usize, g: usize) -> Nfp {
        points.push(points[0]);
        Self {
            points,
            i_piece_idx: i,
            j_piece_idx: j,
            f_conv_piece_idx: f,
            g_conv_piece_idx: g,
        }
    }

    pub fn idx(&self) -> usize {
        4 * self.f_conv_piece_idx + self.g_conv_piece_idx
    }

    pub fn points(&self) -> impl Iterator<Item = &Point> {
        self.points.iter()
    }

    pub fn x_min(&self) -> f64 {
        self.points
            .iter()
            .map(|p| p.x)
            .fold(f64::INFINITY, f64::min)
    }

    pub fn x_max(&self) -> f64 {
        self.points
            .iter()
            .map(|p| p.x)
            .fold(f64::NEG_INFINITY, f64::max)
    }

    pub fn edges(&self) -> impl Iterator<Item = (usize, Point, Point)> {
        self.points
            .windows(2)
            .enumerate()
            .map(|(i, w)| (i, w[0], w[1]))
    }
}

pub fn compute_nfps<S: Tree>(trees: &[S]) -> Vec<NcNfp> {
    let n = trees.len();
    let mut all_ncnfps = Vec::new();

    // Pre-compute convex decompositions for all shapes
    let convex_decomps: Vec<_> = trees.iter().map(|s| s.convex_decomp()).collect();

    // Compute NFPs for all pairs (i, j) where i < j
    for i in 0..n {
        for j in (i + 1)..n {
            let convex_decomp_i = &convex_decomps[i];
            let convex_decomp_j = &convex_decomps[j];

            let mut nfps_for_pair = Vec::new();

            // Compute NFP between all pairs of convex parts
            for (f, poly_a) in convex_decomp_i.iter().enumerate() {
                for (g, poly_b) in convex_decomp_j.iter().enumerate() {
                    let nfp_points = NFPConvex::nfp(poly_a, poly_b).unwrap();
                    let nfp = Nfp::new(nfp_points, i, j, f, g);
                    nfps_for_pair.push(nfp);
                }
            }

            let identical = trees[i].rotation() == trees[j].rotation();
            let ncnfp = NcNfp::new(nfps_for_pair, i, j, identical);
            all_ncnfps.push(ncnfp);
        }
    }

    all_ncnfps
}
