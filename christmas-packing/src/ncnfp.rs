use nfp::{NFPConvex, Point};

use crate::tree::ChristmasTree;

pub struct NcNfp {
    pub i_piece_idx: usize,
    pub j_piece_idx: usize,
    sub_nfps: Vec<Nfp>,
}
impl NcNfp {
    pub fn new(nfps: Vec<Nfp>, i: usize, j: usize) -> Self {
        Self {
            sub_nfps: nfps,
            i_piece_idx: i,
            j_piece_idx: j,
        }
    }

    pub fn sub_nfps(&self) -> impl Iterator<Item = &Nfp> {
        self.sub_nfps.iter()
    }
}

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

pub fn compute_nfps(trees: &[ChristmasTree]) -> Vec<NcNfp> {
    let convex_decomp_1 = trees[0].convex_decomp();
    let convex_decomp_2 = trees[1].convex_decomp();

    let mut all_nfps = Vec::new();
    for (f, poly_a) in convex_decomp_1.iter().enumerate() {
        for (g, poly_b) in convex_decomp_2.iter().enumerate() {
            let nfp_points = NFPConvex::nfp(poly_a, poly_b).unwrap();
            let nfp = Nfp::new(nfp_points, 0, 1, f, g);
            all_nfps.push(nfp);
        }
    }
    let ncnfp = NcNfp::new(all_nfps, 0, 1);
    return vec![ncnfp];
}
