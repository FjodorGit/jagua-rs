use std::collections::HashMap;

use grb::prelude::*;
use nfp::Point;

use crate::{
    ncnfp::{NcNfp, Nfp, compute_nfps},
    tree::{PieceBounds, Tree},
};

#[derive(Copy, Clone)]
struct EdgeVar {
    a: Point,
    b: Point,
    v: Var,
}

#[derive(Copy, Clone)]
struct HalfPlaneVar {
    x_bound: f64,
    v: Var,
}

impl EdgeVar {
    fn is_top(&self) -> bool {
        self.a.x > self.b.x - 1e10
    }

    fn point_is_to_the_right(&self, c: Point) -> bool {
        let cross =
            (self.b.x - self.a.x) * (c.y - self.a.y) - (self.b.y - self.a.y) * (c.x - self.a.x);
        cross < 0.
    }

    fn left_bound(&self) -> f64 {
        self.a.x.min(self.b.x)
    }

    fn right_bound(&self) -> f64 {
        self.a.x.max(self.b.x)
    }

    fn contains(&self, other: &Self) -> bool {
        self.left_bound() > other.left_bound() - 1e-10
            && self.right_bound() < other.right_bound() - 1e-10
            && self.point_is_to_the_right(other.a)
            && self.point_is_to_the_right(other.b)
    }
}

/// Represents a region's geometry for containment checking
enum RegionGeometry {
    /// Half-plane: x < x_bound
    Left(HalfPlaneVar),
    /// Half-plane: x > x_bound
    Right(HalfPlaneVar),
    /// Vertical slice between x_left and x_right (from edge regions)
    Slice(EdgeVar),
}

impl RegionGeometry {
    fn var(&self) -> Var {
        match self {
            RegionGeometry::Left(half_plane_var) => half_plane_var.v,
            RegionGeometry::Right(half_plane_var) => half_plane_var.v,
            RegionGeometry::Slice(edge_var) => edge_var.v,
        }
    }
}

/// Check if inner region is geometrically contained within outer region
fn is_subsumed(inner: &RegionGeometry, outer: &RegionGeometry) -> bool {
    match (inner, outer) {
        // Left ⊂ Left: smaller x_bound means smaller region
        (RegionGeometry::Left(l_in), RegionGeometry::Left(l_out)) => {
            l_in.x_bound < l_out.x_bound - 1e-10
        }
        // Right ⊂ Right: larger x_bound means smaller region
        (RegionGeometry::Right(r_in), RegionGeometry::Right(r_out)) => {
            r_in.x_bound > r_out.x_bound + 1e-10
        }
        // Slice ⊂ Left: slice must be entirely left of x_bound
        (RegionGeometry::Slice(edge), RegionGeometry::Left(l)) => {
            edge.right_bound() < l.x_bound - 1e-10
        }
        // Slice ⊂ Right: slice must be entirely right of x_bound
        (RegionGeometry::Slice(edge), RegionGeometry::Right(r)) => {
            edge.left_bound() > r.x_bound + 1e-10
        }
        // Slice ⊂ Slice: inner must be contained in outer's x-range
        // (RegionGeometry::Slice(edge_in), RegionGeometry::Slice(edge_out)) => {
        //     edge_in.contains(edge_out)
        // }
        // Left/Right cannot be subsumed by Slice (infinite vs finite)
        _ => false,
    }
}

struct NfpVarInfo {
    nfp: Nfp,
    left_var: HalfPlaneVar,
    right_var: HalfPlaneVar,
    edge_vars: Vec<EdgeVar>,
}

impl NfpVarInfo {
    fn i_piece_idx(&self) -> usize {
        self.nfp.i_piece_idx
    }

    fn j_piece_idx(&self) -> usize {
        self.nfp.j_piece_idx
    }

    fn idx(&self) -> usize {
        self.nfp.idx()
    }

    fn edges(&self) -> impl Iterator<Item = &EdgeVar> {
        self.edge_vars.iter()
    }

    fn x_min(&self) -> f64 {
        self.left_var.x_bound
    }

    fn x_max(&self) -> f64 {
        self.right_var.x_bound
    }

    /// Collect all regions (left, right, and edge-based) with their geometry
    fn all_regions(&self) -> Vec<RegionGeometry> {
        let mut regions = Vec::new();

        regions.push(RegionGeometry::Left(self.left_var));
        regions.push(RegionGeometry::Right(self.right_var));

        for edge_var in &self.edge_vars {
            regions.push(RegionGeometry::Slice(*edge_var));
        }

        regions
    }
}

/// Solve the packing problem for n shapes with given rotations
pub fn solve_packing<S: Tree>(trees: &[S]) -> Result<Solution, grb::Error> {
    let n = trees.len();
    let nc_nfps: Vec<NcNfp> = compute_nfps(trees);

    let mut model = Model::new("n_shapes")?;
    model.set_param(param::NumericFocus, 0)?;

    // Compute per-piece geometry bounds (distances from reference point to boundaries)
    let bounds: Vec<PieceBounds> = trees.iter().map(|shape| shape.bounds()).collect();

    // Constraint (21): Lower bound for s
    let s_lb = bounds.iter().map(|b| b.max_diameter()).fold(0.0, f64::max);

    // Constraint (20): Upper bound for s
    // Worst case: all pieces placed end to end
    let max_dimension: f64 = bounds.iter().map(|b| b.max_diameter()).sum();
    let s_ub = max_dimension;

    // Decision variables for s (square side length)
    let s = add_ctsvar!(model, name: "s", bounds: s_lb..s_ub)?;

    // Decision variables for piece positions (30), (31)
    let mut x_vars: Vec<Var> = Vec::with_capacity(n);
    let mut y_vars: Vec<Var> = Vec::with_capacity(n);

    for i in 0..n {
        // Constraint (18): l_min <= x_i
        let x_i = add_ctsvar!(model, name: &format!("x_{}", i), bounds: bounds[i].l_min..)?;
        // Constraint (19): h_min <= y_i
        let y_i = add_ctsvar!(model, name: &format!("y_{}", i), bounds: bounds[i].h_min..)?;

        x_vars.push(x_i);
        y_vars.push(y_i);

        // Constraints (18) upper bounds: x_i <= s - l_max
        model.add_constr(
            &format!("bound_x_{}_upper", i),
            c!(x_i <= s - bounds[i].l_max),
        )?;

        // Constraints (19) upper bounds: y_i <= s - h_max
        model.add_constr(
            &format!("bound_y_{}_upper", i),
            c!(y_i <= s - bounds[i].h_max),
        )?;
    }

    // Process all pairs (i, j) where i < j
    // nfps is indexed as nfps[i][j-i-1] for pair (i, j)
    let mut all_nfp_vars = Vec::new();
    for nc_nfp in nc_nfps {
        let nfp_vars = add_nfp_vars(&mut model, nc_nfp)?;
        all_nfp_vars.extend(nfp_vars);
    }

    for nfp_var in all_nfp_vars.iter() {
        add_nfp_constraints(&mut model, nfp_var, &x_vars, &y_vars, &bounds, s_ub)?;
    }

    add_identical_piece_triplet_constraints(&mut model, &all_nfp_vars, trees)?;

    // Symmetry breaking: order identical pieces by y-coordinate
    // This assumes all trees are identical - adjust if they have different shapes
    for i in 0..(n - 1) {
        if trees[i].rotation() == trees[i + 1].rotation() {
            model.add_constr(
                &format!("symmetry_breaking_{}", i),
                c!(y_vars[i] <= y_vars[i + 1]),
            )?;
        }
    }

    // Objective function (17): minimize square side length
    model.set_objective(s, Minimize)?;

    // Write model for inspection
    model.write("model.lp")?;

    // Optimize
    model.optimize()?;

    // Extract results
    let status = model.status()?;
    println!("Optimization status: {:?}", status);

    if status == Status::Optimal || status == Status::SubOptimal {
        let s_val = model.get_obj_attr(attr::X, &s)?;
        let positions: Vec<Point> = x_vars
            .iter()
            .zip(y_vars.iter())
            .map(|(x, y)| {
                let x_val = model.get_obj_attr(attr::X, x).unwrap();
                let y_val = model.get_obj_attr(attr::X, y).unwrap();
                Point::new(x_val, y_val)
            })
            .collect();

        Ok(Solution {
            s: s_val,
            positions,
            status,
        })
    } else {
        Ok(Solution {
            s: f64::INFINITY,
            positions: vec![],
            status,
        })
    }
}

fn add_nfp_vars(model: &mut Model, nc_nfp: NcNfp) -> Result<Vec<NfpVarInfo>, grb::Error> {
    let i = nc_nfp.i_piece_idx;
    let j = nc_nfp.j_piece_idx;
    let pieces_identical = nc_nfp.indentical_pieces;

    let mut left_region_bounds: Vec<(f64, Var)> = Vec::new();
    let mut right_region_bounds: Vec<(f64, Var)> = Vec::new();
    let mut nfp_var_infos: Vec<NfpVarInfo> = Vec::new();
    for nfp in nc_nfp.as_sub_nfps() {
        let fg_idx = nfp.idx();
        let x_fg_min = nfp.x_min();
        let x_fg_max = nfp.x_max();

        let v_left = if let Some((_, existing_var)) = left_region_bounds
            .iter()
            .filter(|(l, _)| (*l - x_fg_min).abs() < 1e-10)
            .next()
        {
            *existing_var
        } else {
            let new_var = add_binvar!(model, name: &format!("v_l_{}_{}_{}", i, j, fg_idx))?;
            left_region_bounds.push((x_fg_min, new_var));
            new_var
        };
        let v_right = if let Some((_, existing_var)) = right_region_bounds
            .iter()
            .filter(|(r, _)| (*r - x_fg_max).abs() < 1e-10)
            .next()
        {
            *existing_var
        } else {
            let new_var = add_binvar!(model, name: &format!("v_r_{}_{}_{}", i, j, fg_idx))?;
            right_region_bounds.push((x_fg_max, new_var));
            new_var
        };
        let mut edge_vars: Vec<EdgeVar> = Vec::new();

        for (edge_idx, a, b) in nfp.edges() {
            // Skip vertical edges
            if (a.x - b.x).abs() < 1e-10 {
                continue;
            }

            let is_top = a.x > b.x;
            // =======================================================
            // Constraint (34): Skip bottom regions below reference point for identical pieces
            // =======================================================
            if pieces_identical && !is_top && a.y < 0.0 && b.y < 0.0 {
                // This variable would be fixed to 0, so we don't create it at all
                // The constraint (23) sum will simply not include this region
                continue;
            }

            let v = add_binvar!(model, name: &format!("v_{}_{}_{}_{}",i, j, fg_idx, edge_idx))?;
            let edge_var = EdgeVar { a, b, v };
            edge_vars.push(edge_var);
        }
        let left_var = HalfPlaneVar {
            x_bound: x_fg_min,
            v: v_left,
        };
        let right_var = HalfPlaneVar {
            x_bound: x_fg_max,
            v: v_right,
        };
        let nfp_var = NfpVarInfo {
            nfp,
            left_var,
            right_var,
            edge_vars,
        };
        nfp_var_infos.push(nfp_var);
    }
    add_subsumption_constraints(model, &nfp_var_infos)?;
    Ok(nfp_var_infos)
}

/// Add NFP constraints for a pair of pieces (i, j)
fn add_nfp_constraints(
    model: &mut Model,
    nfp: &NfpVarInfo,
    x_vars: &[Var],
    y_vars: &[Var],
    bounds: &[PieceBounds],
    s_ub: f64,
) -> Result<(), grb::Error> {
    let x_i = x_vars[nfp.i_piece_idx()];
    let y_i = y_vars[nfp.i_piece_idx()];
    let x_j = x_vars[nfp.j_piece_idx()];
    let y_j = y_vars[nfp.j_piece_idx()];

    for (edge_idx, EdgeVar { a, b, v }) in nfp.edges().enumerate() {
        // Constraint (22): half-plane constraint
        let c_val = b.y * a.x - b.x * a.y;
        let m_22 = (b.x - a.x).abs() * s_ub + (a.y - b.y).abs() * s_ub + c_val;

        model.add_constr(
            &format!(
                "halfplane_{}_{}_{}_{}",
                nfp.i_piece_idx(),
                nfp.j_piece_idx(),
                nfp.idx(),
                edge_idx
            ),
            c!((b.x - a.x) * (y_j - y_i) + (a.y - b.y) * (x_j - x_i) + c_val <= (1.0 - *v) * m_22),
        )?;

        if a.x > b.x {
            // TOP edge: constraints (24) and (25)
            let m_24 =
                b.x + s_ub - bounds[nfp.i_piece_idx()].l_max - bounds[nfp.j_piece_idx()].l_min;
            model.add_constr(
                &format!(
                    "top_left_{}_{}_{}_{}",
                    nfp.i_piece_idx(),
                    nfp.j_piece_idx(),
                    nfp.idx(),
                    edge_idx
                ),
                c!(b.x + x_i - x_j <= (1.0 - *v) * m_24),
            )?;

            let m_25 =
                s_ub - bounds[nfp.j_piece_idx()].l_max - bounds[nfp.i_piece_idx()].l_min - a.x;
            model.add_constr(
                &format!(
                    "top_right_{}_{}_{}_{}",
                    nfp.i_piece_idx(),
                    nfp.j_piece_idx(),
                    nfp.idx(),
                    edge_idx
                ),
                c!(x_j - x_i - a.x <= (1.0 - *v) * m_25),
            )?;
        } else {
            // BOTTOM edge: constraints (26) and (27)
            let m_26 =
                a.x + s_ub - bounds[nfp.i_piece_idx()].l_max - bounds[nfp.j_piece_idx()].l_min;
            model.add_constr(
                &format!(
                    "bottom_left_{}_{}_{}_{}",
                    nfp.i_piece_idx(),
                    nfp.j_piece_idx(),
                    nfp.idx(),
                    edge_idx
                ),
                c!(a.x + x_i - x_j <= (1.0 - *v) * m_26),
            )?;

            let m_27 =
                s_ub - bounds[nfp.j_piece_idx()].l_max - bounds[nfp.i_piece_idx()].l_min - b.x;
            model.add_constr(
                &format!(
                    "bottom_right_{}_{}_{}_{}",
                    nfp.i_piece_idx(),
                    nfp.j_piece_idx(),
                    nfp.idx(),
                    edge_idx
                ),
                c!(x_j - x_i - b.x <= (1.0 - *v) * m_27),
            )?;
        }
    }

    // Constraint (28): left region constraint
    let m_28 =
        s_ub - bounds[nfp.j_piece_idx()].l_max - bounds[nfp.i_piece_idx()].l_min - nfp.x_min();
    model.add_constr(
        &format!(
            "left_region_{}_{}_{}",
            nfp.i_piece_idx(),
            nfp.j_piece_idx(),
            nfp.idx()
        ),
        c!(x_j - x_i - nfp.x_min() <= (1.0 - nfp.left_var.v) * m_28),
    )?;

    // Constraint (29): right region constraint
    let m_29 =
        bounds[nfp.j_piece_idx()].l_min + bounds[nfp.i_piece_idx()].l_max - s_ub - nfp.x_max();
    model.add_constr(
        &format!(
            "right_region_{}_{}_{}",
            nfp.i_piece_idx(),
            nfp.j_piece_idx(),
            nfp.idx()
        ),
        c!(x_j - x_i - nfp.x_max() >= (1.0 - nfp.right_var.v) * m_29),
    )?;

    let mut all_region_vars = vec![nfp.left_var.v, nfp.right_var.v];
    all_region_vars.extend(nfp.edge_vars.iter().map(|e| &e.v));
    model.add_constr(
        &format!("one_region_{}_{}", nfp.i_piece_idx(), nfp.j_piece_idx(),),
        c!(all_region_vars.grb_sum() == 1),
    )?;

    Ok(())
}

fn add_identical_piece_triplet_constraints<S: Tree>(
    model: &mut Model,
    all_nfp_vars: &[NfpVarInfo],
    trees: &[S],
) -> Result<(), grb::Error> {
    let n = trees.len();

    // Build lookup: (i, j) -> Vec<&NfpVarInfo> for all NFP parts between i and j
    let mut nfp_lookup: HashMap<(usize, usize), Vec<&NfpVarInfo>> = HashMap::new();
    for nfp_info in all_nfp_vars {
        nfp_lookup
            .entry((nfp_info.i_piece_idx(), nfp_info.j_piece_idx()))
            .or_default()
            .push(nfp_info);
    }

    // For all triplets (i, j, u) where i < j < u and pieces j, u are identical
    for i in 0..n {
        for j in (i + 1)..n {
            for u in (j + 1)..n {
                // Check if j and u are identical pieces
                if trees[j].rotation() != trees[u].rotation() {
                    continue;
                }

                // Get NFP parts for (i, j) and (i, u)
                let nfps_ij = match nfp_lookup.get(&(i, j)) {
                    Some(v) => v,
                    None => continue,
                };
                let nfps_iu = match nfp_lookup.get(&(i, u)) {
                    Some(v) => v,
                    None => continue,
                };

                // Collect all top region vars from (i, j) NFPs
                let mut top_vars_ij: Vec<Var> = Vec::new();
                for nfp_info in nfps_ij {
                    for e in &nfp_info.edge_vars {
                        if e.is_top() {
                            top_vars_ij.push(e.v);
                        }
                    }
                }

                // Collect all bottom region vars from (i, u) NFPs
                let mut bottom_vars_iu: Vec<Var> = Vec::new();
                for nfp_info in nfps_iu {
                    for e in &nfp_info.edge_vars {
                        if !e.is_top() {
                            bottom_vars_iu.push(e.v);
                        }
                    }
                }

                // Constraint (35): sum of tops(i,j) + sum of bottoms(i,u) <= 1
                if !top_vars_ij.is_empty() && !bottom_vars_iu.is_empty() {
                    let mut all_vars: Vec<Var> = top_vars_ij;
                    all_vars.extend(bottom_vars_iu);

                    model.add_constr(
                        &format!("identical_triplet_{}_{}_{}", i, j, u),
                        c!(all_vars.iter().grb_sum() <= 1),
                    )?;
                }
            }
        }
    }

    Ok(())
}

/// Add constraint (38) for all NFP parts of a piece pair
fn add_subsumption_constraints(
    model: &mut Model,
    nfp_var_infos: &[NfpVarInfo],
) -> Result<(), grb::Error> {
    let i = nfp_var_infos[0].i_piece_idx();
    let j = nfp_var_infos[0].j_piece_idx();

    let mut constr_idx = 0;

    // For each outer NFP part and its regions
    for (outer_fg_idx, outer_nfp) in nfp_var_infos.iter().enumerate() {
        for outer_region in outer_nfp.all_regions() {
            // For each inner NFP part (different from outer)
            for (inner_fg_idx, inner_nfp) in nfp_var_infos.iter().enumerate() {
                if inner_fg_idx == outer_fg_idx {
                    continue;
                }

                // Collect subsumed regions from THIS inner NFP part only
                let subsumed_vars: Vec<Var> = inner_nfp
                    .all_regions()
                    .into_iter()
                    .filter(|inner_region| is_subsumed(inner_region, &outer_region))
                    .map(|inner_region| inner_region.var())
                    .collect();

                if !subsumed_vars.is_empty() {
                    model.add_constr(
                        &format!("subsume_{}_{}_{}_{}", i, j, outer_fg_idx, constr_idx),
                        c!(subsumed_vars.grb_sum() <= outer_region.var()),
                    )?;
                    constr_idx += 1;
                }
            }
        }
    }

    Ok(())
}

#[derive(Debug)]
pub struct Solution {
    pub s: f64,
    pub positions: Vec<Point>,
    pub status: Status,
}
