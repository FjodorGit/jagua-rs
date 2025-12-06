use grb::prelude::*;

use crate::{
    ncnfp::{NcNfp, Nfp, compute_nfps},
    tree::{ChristmasTree, PieceBounds},
};

/// Solve the packing problem for n trees with given rotations
pub fn solve_packing(trees: &[ChristmasTree]) -> Result<Solution, grb::Error> {
    let n = trees.len();
    let nc_nfps: Vec<NcNfp> = compute_nfps(trees);
    let mut model = Model::new("n_christmas_trees")?;

    // Compute per-piece geometry bounds (distances from reference point to boundaries)
    let bounds: Vec<PieceBounds> = trees.iter().map(|tree| tree.bounds()).collect();

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
    for nc_nfp in nc_nfps {
        for conv_nfp in nc_nfp.sub_nfps() {
            add_nfp_constraints(&mut model, conv_nfp, &x_vars, &y_vars, &bounds, s_ub)?;
        }
    }

    // Symmetry breaking: order identical pieces by y-coordinate
    // This assumes all trees are identical - adjust if they have different shapes
    for i in 0..(n - 1) {
        model.add_constr(
            &format!("symmetry_breaking_{}", i),
            c!(y_vars[i] <= y_vars[i + 1]),
        )?;
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
        let positions: Vec<(f64, f64)> = x_vars
            .iter()
            .zip(y_vars.iter())
            .map(|(x, y)| {
                let x_val = model.get_obj_attr(attr::X, x).unwrap();
                let y_val = model.get_obj_attr(attr::X, y).unwrap();
                (x_val, y_val)
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

/// Add NFP constraints for a pair of pieces (i, j)
fn add_nfp_constraints(
    model: &mut Model,
    nfp: &Nfp,
    x_vars: &[Var],
    y_vars: &[Var],
    bounds: &[PieceBounds],
    s_ub: f64,
) -> Result<(), grb::Error> {
    let x_i = x_vars[nfp.i_piece_idx];
    let y_i = y_vars[nfp.i_piece_idx];
    let x_j = x_vars[nfp.j_piece_idx];
    let y_j = y_vars[nfp.j_piece_idx];

    // Compute NFP x-bounds for left/right regions (equations 10, 11)
    let x_fg_min = nfp.x_min();
    let x_fg_max = nfp.x_max();

    // Collect all region variables for constraint (23)
    let mut all_region_vars: Vec<Var> = Vec::new();

    // Left region variable
    let v_l = add_binvar!(model, name: &format!("v_l_{}_{}_{}",nfp.i_piece_idx, nfp.j_piece_idx, nfp.idx()))?;

    // Right region variable
    let v_r = add_binvar!(model, name: &format!("v_r_{}_{}_{}", nfp.i_piece_idx, nfp.j_piece_idx, nfp.idx()))?;

    // Process all edges including the closing edge
    for (edge_idx, a, b) in nfp.edges() {
        // Skip vertical edges
        if (a.x - b.x).abs() < 1e-9 {
            continue;
        }

        // Create binary variable for this edge's region
        let v = add_binvar!(model, name: &format!("v_{}_{}_{}_{}",nfp.i_piece_idx, nfp.j_piece_idx, nfp.idx(), edge_idx))?;
        all_region_vars.push(v);

        // Constraint (22): half-plane constraint
        let c_val = b.y * a.x - b.x * a.y;
        let m_22 = (b.x - a.x).abs() * s_ub + (a.y - b.y).abs() * s_ub + c_val;

        model.add_constr(
            &format!(
                "halfplane_{}_{}_{}_{}",
                nfp.i_piece_idx,
                nfp.j_piece_idx,
                nfp.idx(),
                edge_idx
            ),
            c!((b.x - a.x) * (y_j - y_i) + (a.y - b.y) * (x_j - x_i) + c_val <= (1.0 - v) * m_22),
        )?;

        if a.x > b.x {
            // TOP edge: constraints (24) and (25)
            let m_24 = b.x + s_ub - bounds[nfp.i_piece_idx].l_max - bounds[nfp.j_piece_idx].l_min;
            model.add_constr(
                &format!(
                    "top_left_{}_{}_{}_{}",
                    nfp.i_piece_idx,
                    nfp.j_piece_idx,
                    nfp.idx(),
                    edge_idx
                ),
                c!(b.x + x_i - x_j <= (1.0 - v) * m_24),
            )?;

            let m_25 = s_ub - bounds[nfp.j_piece_idx].l_max - bounds[nfp.i_piece_idx].l_min - a.x;
            model.add_constr(
                &format!(
                    "top_right_{}_{}_{}_{}",
                    nfp.i_piece_idx,
                    nfp.j_piece_idx,
                    nfp.idx(),
                    edge_idx
                ),
                c!(x_j - x_i - a.x <= (1.0 - v) * m_25),
            )?;
        } else {
            // BOTTOM edge: constraints (26) and (27)
            let m_26 = a.x + s_ub - bounds[nfp.i_piece_idx].l_max - bounds[nfp.j_piece_idx].l_min;
            model.add_constr(
                &format!(
                    "bottom_left_{}_{}_{}_{}",
                    nfp.i_piece_idx,
                    nfp.j_piece_idx,
                    nfp.idx(),
                    edge_idx
                ),
                c!(a.x + x_i - x_j <= (1.0 - v) * m_26),
            )?;

            let m_27 = s_ub - bounds[nfp.j_piece_idx].l_max - bounds[nfp.i_piece_idx].l_min - b.x;
            model.add_constr(
                &format!(
                    "bottom_right_{}_{}_{}_{}",
                    nfp.i_piece_idx,
                    nfp.j_piece_idx,
                    nfp.idx(),
                    edge_idx
                ),
                c!(x_j - x_i - b.x <= (1.0 - v) * m_27),
            )?;
        }
    }

    // Constraint (28): left region constraint
    let m_28 = s_ub - bounds[nfp.j_piece_idx].l_max - bounds[nfp.i_piece_idx].l_min - x_fg_min;
    model.add_constr(
        &format!(
            "left_region_{}_{}_{}",
            nfp.i_piece_idx,
            nfp.j_piece_idx,
            nfp.idx()
        ),
        c!(x_j - x_i - x_fg_min <= (1.0 - v_l) * m_28),
    )?;

    // Constraint (29): right region constraint
    let m_29 = bounds[nfp.j_piece_idx].l_min + bounds[nfp.i_piece_idx].l_max - s_ub - x_fg_max;
    model.add_constr(
        &format!(
            "right_region_{}_{}_{}",
            nfp.i_piece_idx,
            nfp.j_piece_idx,
            nfp.idx()
        ),
        c!(x_j - x_i - x_fg_max >= (1.0 - v_r) * m_29),
    )?;

    // Add left and right vars to the collection
    all_region_vars.insert(0, v_l);
    all_region_vars.insert(1, v_r);

    // Constraint (23): exactly one region must be selected per NFP
    model.add_constr(
        &format!(
            "one_region_{}_{}_{}",
            nfp.i_piece_idx,
            nfp.j_piece_idx,
            nfp.idx()
        ),
        c!(all_region_vars.iter().grb_sum() == 1),
    )?;

    Ok(())
}

#[derive(Debug)]
pub struct Solution {
    pub s: f64,
    pub positions: Vec<(f64, f64)>,
    pub status: Status,
}
