pub mod ncnfp;

use anyhow::{Context, Result};
use clap::Parser as ClapParser;
use jagua_rs::io::import::Importer;
use jagua_rs::io::svg::s_layout_to_svg;
use jagua_rs::probs::qpp;
use jagua_rs::probs::qpp::io::ext_repr::{ExtItem, ExtQPInstance};
use lbf::EPOCH;
use lbf::config::LBFConfig;
use lbf::io::cli::Cli;
use lbf::io::output::{CSVPlacedItem, CombinedCSVItem, QPOutput, layout_to_csv};
use lbf::io::{init_logger, read_qpp_instance, write_combined_csv, write_svg};
use lbf::opt::lbf_qpp::LBFOptimizerQP;
use log::{info, warn};
use nfp::{NFPConvex, point};
use rand::SeedableRng;
use rand::prelude::SmallRng;
use std::fs::{self, File};
use std::io::BufReader;
use std::path::{Path, PathBuf};
use std::process::exit;

use grb::prelude::*;

use crate::ncnfp::christmas_tree;

fn main() -> Result<()> {
    // Reference point of trees is the origin (0,0)
    // Get the tree shape (scaled and rotated)
    let tree1 = christmas_tree(0.);
    let tree2 = christmas_tree(180.);
    let nfps = ncnfp::christmas_tree_nfps(180.);
    let mut model = Model::new("2_christmas_trees")?;

    // Piece geometry parameters (distances from reference point to piece boundaries)
    // These should match your actual Christmas tree geometry
    let h_1_min: f64 = tree1
        .iter()
        .map(|p| p.y)
        .min_by(|y_1, y_2| y_1.partial_cmp(y_2).unwrap())
        .expect("should have a min value")
        .abs();
    let h_1_max: f64 = tree1
        .iter()
        .map(|p| p.y)
        .max_by(|y_1, y_2| y_1.partial_cmp(y_2).unwrap())
        .expect("should have a max value")
        .abs();
    let l_1_min: f64 = tree1
        .iter()
        .map(|p| p.x)
        .min_by(|x_1, x_2| x_1.partial_cmp(x_2).unwrap())
        .expect("should have a min value")
        .abs();
    let l_1_max: f64 = tree1
        .iter()
        .map(|p| p.x)
        .max_by(|x_1, x_2| x_1.partial_cmp(x_2).unwrap())
        .expect("should have a max value")
        .abs();

    let h_2_min: f64 = tree2
        .iter()
        .map(|p| p.y)
        .min_by(|y_1, y_2| y_1.partial_cmp(y_2).unwrap())
        .expect("should have a min value")
        .abs();
    let h_2_max: f64 = tree2
        .iter()
        .map(|p| p.y)
        .max_by(|y_1, y_2| y_1.partial_cmp(y_2).unwrap())
        .expect("should have a max value")
        .abs();
    let l_2_min: f64 = tree2
        .iter()
        .map(|p| p.x)
        .min_by(|x_1, x_2| x_1.partial_cmp(x_2).unwrap())
        .expect("should have a min value")
        .abs();
    let l_2_max: f64 = tree2
        .iter()
        .map(|p| p.x)
        .max_by(|x_1, x_2| x_1.partial_cmp(x_2).unwrap())
        .expect("should have a max value")
        .abs();

    // Compute bounds for s (the square side length)
    let piece_1_width = l_1_min + l_1_max; // total width of piece
    let piece_1_height = h_1_min + h_1_max; // total height of piece
    let piece_2_width = l_2_min + l_2_max; // total width of piece
    let piece_2_height = h_2_min + h_2_max; // total height of piece

    // Constraint (21): Lower bound for s
    // s >= max(largest_piece_dimension, total_area / s)
    // For a square, simplified to at least fit one piece
    let s_lb = piece_1_width
        .max(piece_1_height)
        .max(piece_2_width)
        .max(piece_2_height);

    // Constraint (20): Upper bound for s
    // Worst case: pieces placed end to end
    let s_ub = 2.0 * s_lb;

    // Decision variables (30), (31), (32)
    // Constraint (18): l_min <= x_i <= s - l_max
    let x1 = add_ctsvar!(model, name: "x1", bounds: l_1_min..)?;
    let y1 = add_ctsvar!(model, name: "y1", bounds: h_1_min..)?;
    let x2 = add_ctsvar!(model, name: "x2", bounds: l_2_min..)?;
    let y2 = add_ctsvar!(model, name: "y2", bounds: h_2_min..)?;
    let s = add_ctsvar!(model, name: "s", bounds: s_lb..s_ub)?;

    // Constraints (18) upper bounds: x_i <= s - l_max
    model.add_constr("bound_x1_upper", c!(x1 <= s - l_1_max))?;
    model.add_constr("bound_x2_upper", c!(x2 <= s - l_2_max))?;

    // Constraints (19) upper bounds: y_i <= s - h_max (using s instead of H for square)
    model.add_constr("bound_y1_upper", c!(y1 <= s - h_1_max))?;
    model.add_constr("bound_y2_upper", c!(y2 <= s - h_2_max))?;

    // Process each NFP (for convex parts - with 2 convex pieces, there's typically 1 NFP)
    for (nfp_idx, nfp) in nfps.iter().enumerate() {
        // Compute NFP x-bounds for left/right regions (equations 10, 11)
        let x_fg_min = nfp.iter().map(|p| p.x).fold(f64::INFINITY, f64::min);
        let x_fg_max = nfp.iter().map(|p| p.x).fold(f64::NEG_INFINITY, f64::max);

        // Collect all region variables for constraint (23)
        let mut all_region_vars: Vec<Var> = Vec::new();

        // Left region variable (for positions where piece 2 is far left of piece 1)
        let v_l = add_binvar!(model, name: &format!("v_l_{}", nfp_idx))?;

        // Right region variable (for positions where piece 2 is far right of piece 1)
        let v_r = add_binvar!(model, name: &format!("v_r_{}", nfp_idx))?;

        // Process all edges including the closing edge (last point -> first point)
        let n_points = nfp.len();
        for edge_idx in 0..n_points {
            let a = &nfp[edge_idx];
            let b = &nfp[(edge_idx + 1) % n_points];

            // Skip vertical edges (they don't define top/bottom regions)
            if (a.x - b.x).abs() < 1e-9 {
                continue;
            }

            // Create binary variable for this edge's region
            let v = add_binvar!(model, name: &format!("v_{}_{}", nfp_idx, edge_idx))?;
            all_region_vars.push(v);

            // Constraint (22): half-plane constraint
            // Defines feasible region on the RIGHT side of counter-clockwise oriented edge
            let c_val = b.y * a.x - b.x * a.y;
            let m_22 = (b.x - a.x).abs() * s_ub + (a.y - b.y).abs() * s_ub + c_val;

            model.add_constr(
                &format!("halfplane_{}_{}", nfp_idx, edge_idx),
                c!((b.x - a.x) * (y2 - y1) + (a.y - b.y) * (x2 - x1) + c_val <= (1.0 - v) * m_22),
            )?;

            if a.x > b.x {
                // TOP edge (counter-clockwise, so x decreases): constraints (24) and (25)

                // Constraint (24): left bound of vertical slice
                // b.x + x_i - x_j <= (1 - v) * M'
                let m_24 = b.x + s_ub - l_1_max - l_2_min;
                model.add_constr(
                    &format!("top_left_{}_{}", nfp_idx, edge_idx),
                    c!(b.x + x1 - x2 <= (1.0 - v) * m_24),
                )?;

                // Constraint (25): right bound of vertical slice
                // x_j - x_i - a.x <= (1 - v) * M''
                let m_25 = s_ub - l_2_max - l_1_min - a.x;
                model.add_constr(
                    &format!("top_right_{}_{}", nfp_idx, edge_idx),
                    c!(x2 - x1 - a.x <= (1.0 - v) * m_25),
                )?;
            } else {
                // BOTTOM edge (counter-clockwise, so x increases): constraints (26) and (27)

                // Constraint (26): left bound of vertical slice
                // a.x + x_i - x_j <= (1 - v) * M_bar'
                let m_26 = a.x + s_ub - l_1_max - l_2_min;
                model.add_constr(
                    &format!("bottom_left_{}_{}", nfp_idx, edge_idx),
                    c!(a.x + x1 - x2 <= (1.0 - v) * m_26),
                )?;

                // Constraint (27): right bound of vertical slice
                // x_j - x_i - b.x <= (1 - v) * M_bar''
                let m_27 = s_ub - l_2_max - l_1_min - b.x;
                model.add_constr(
                    &format!("bottom_right_{}_{}", nfp_idx, edge_idx),
                    c!(x2 - x1 - b.x <= (1.0 - v) * m_27),
                )?;
            }
        }

        // Constraint (28): left region constraint
        // x_j - x_i - x_fg_min <= (1 - v_l) * M_l
        // When v_l = 1: piece 2 must be at x-position <= x_fg_min relative to piece 1
        let m_28 = s_ub - l_2_max - l_1_min - x_fg_min;
        model.add_constr(
            &format!("left_region_{}", nfp_idx),
            c!(x2 - x1 - x_fg_min <= (1.0 - v_l) * m_28),
        )?;

        // Constraint (29): right region constraint
        // x_j - x_i - x_fg_max >= (1 - v_r) * M_r
        // When v_r = 1: piece 2 must be at x-position >= x_fg_max relative to piece 1
        // Note: M_r is typically negative, which relaxes the constraint when v_r = 0
        let m_29 = l_2_min + l_1_max - s_ub - x_fg_max;
        model.add_constr(
            &format!("right_region_{}", nfp_idx),
            c!(x2 - x1 - x_fg_max >= (1.0 - v_r) * m_29),
        )?;

        // Add left and right vars to the collection
        // (adding them at the end so edge vars are processed first)
        all_region_vars.insert(0, v_l);
        all_region_vars.insert(1, v_r);

        // Constraint (23): exactly one region must be selected per NFP
        // v_l + v_r + sum(v_k for all edges k) = 1
        model.add_constr(
            &format!("one_region_{}", nfp_idx),
            c!(all_region_vars.iter().grb_sum() == 1),
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
        let x1_val = model.get_obj_attr(attr::X, &x1)?;
        let y1_val = model.get_obj_attr(attr::X, &y1)?;
        let x2_val = model.get_obj_attr(attr::X, &x2)?;
        let y2_val = model.get_obj_attr(attr::X, &y2)?;
        let s_val = model.get_obj_attr(attr::X, &s)?;

        println!("Piece 1: x1 = {:.2}, y1 = {:.2}", x1_val, y1_val);
        println!("Piece 2: x2 = {:.2}, y2 = {:.2}", x2_val, y2_val);
        println!("Square side length s = {:.2}", s_val);
        println!(
            "Relative position: dx = {:.2}, dy = {:.2}",
            x2_val - x1_val,
            y2_val - y1_val
        );

        // Translate trees to their positions
        let tree1_positioned: Vec<_> = tree1
            .iter()
            .map(|p| point(p.x + x1_val, p.y + y1_val))
            .collect();
        let tree2_positioned: Vec<_> = tree2
            .iter()
            .map(|p| point(p.x + x2_val, p.y + y2_val))
            .collect();

        // Plot the result
        plot_solution(
            &tree1_positioned,
            &tree2_positioned,
            s_val,
            "output/solution.svg",
        );
        println!("Solution plotted to output/solution.svg");
    }

    exit(0);

    let args = Cli::parse();
    init_logger(args.log_level)?;

    let config = match args.config_file {
        None => {
            warn!("[MAIN] No config file provided, use --config-file to provide a custom config");
            LBFConfig::default()
        }
        Some(config_file) => {
            let file = File::open(config_file)?;
            let reader = BufReader::new(file);
            serde_json::from_reader(reader).context("incorrect config file format")?
        }
    };

    info!("Successfully parsed LBFConfig: {config:?}");

    if !args.solution_folder.exists() {
        fs::create_dir_all(&args.solution_folder).unwrap_or_else(|_| {
            panic!(
                "could not create solution folder: {:?}",
                args.solution_folder
            )
        });
    }

    let base_ext_qp_instance = read_qpp_instance(args.input_file.as_path())?;
    let mut all_items = Vec::new();

    for size in 1u64..201u64 {
        let item = ExtItem {
            base: base_ext_qp_instance.items[0].base.clone(),
            demand: size,
        };
        let ext_qp_instance = ExtQPInstance {
            name: base_ext_qp_instance.name.clone(),
            items: vec![item],
        };
        let csv_items = main_qpp(
            ext_qp_instance,
            config,
            &size.to_string(),
            args.solution_folder.clone(),
        )?;

        for (idx, item) in csv_items.iter().enumerate() {
            all_items.push(CombinedCSVItem {
                id: format!("{:03}_{}", size, idx),
                x: item.reference_point_x,
                y: item.reference_point_y,
                deg: item.rotation_degrees,
            });
        }
    }

    write_combined_csv(
        &all_items,
        &args.solution_folder.join("combined_solution.csv"),
    )?;

    Ok(())
}

fn main_qpp(
    ext_instance: ExtQPInstance,
    config: LBFConfig,
    input_stem: &str,
    output_folder: PathBuf,
) -> Result<Vec<CSVPlacedItem>> {
    let importer = Importer::new(
        config.cde_config,
        config.poly_simpl_tolerance,
        config.min_item_separation,
        config.narrow_concavity_cutoff_ratio,
    );
    let rng = match config.prng_seed {
        Some(seed) => SmallRng::seed_from_u64(seed),
        None => SmallRng::from_os_rng(),
    };
    let instance = qpp::io::import(&importer, &ext_instance)?;
    let sol = LBFOptimizerQP::new(instance.clone(), config, rng).solve();

    let output = QPOutput {
        instance: ext_instance,
        solution: qpp::io::export(&instance, &sol, *EPOCH),
        config,
    };

    {
        let svg_path = output_folder.join(format!("sol_{input_stem}.svg"));
        let svg = s_layout_to_svg(&sol.layout_snapshot, &instance, config.svg_draw_options, "");
        write_svg(&svg, Path::new(&svg_path))?;
    }

    let csv_items = layout_to_csv(&output.solution.layout);
    Ok(csv_items)
}

fn plot_solution(tree1: &[nfp::Point], tree2: &[nfp::Point], square_size: f64, filename: &str) {
    let all_points: Vec<&nfp::Point> = tree1.iter().chain(tree2.iter()).collect();

    let min_x = all_points.iter().map(|p| p.x).fold(f64::INFINITY, f64::min);
    let max_x = all_points
        .iter()
        .map(|p| p.x)
        .fold(f64::NEG_INFINITY, f64::max);
    let min_y = all_points.iter().map(|p| p.y).fold(f64::INFINITY, f64::min);
    let max_y = all_points
        .iter()
        .map(|p| p.y)
        .fold(f64::NEG_INFINITY, f64::max);

    let margin = 1000.0;
    let view_min_x = min_x.min(0.0) - margin;
    let view_min_y = min_y.min(0.0) - margin;
    let view_max_x = max_x.max(square_size) + margin;
    let view_max_y = max_y.max(square_size) + margin;
    let width = view_max_x - view_min_x;
    let height = view_max_y - view_min_y;

    let mut svg = format!(
        r#"<svg xmlns="http://www.w3.org/2000/svg" viewBox="{} {} {} {}">
"#,
        view_min_x, view_min_y, width, height
    );

    // Draw square boundary
    svg.push_str(&format!(
        r#"<rect x="0" y="0" width="{}" height="{}" fill="lightgray" fill-opacity="0.2" stroke="red" stroke-width="100" />
"#,
        square_size, square_size
    ));

    // Draw trees
    svg.push_str(&format_tree_polygon(tree1, "green", 0.6));
    svg.push_str(&format_tree_polygon(tree2, "darkgreen", 0.6));

    svg.push_str("</svg>");

    std::fs::write(filename, svg).unwrap();
}

fn format_tree_polygon(points: &[nfp::Point], color: &str, opacity: f64) -> String {
    if points.is_empty() {
        return String::new();
    }

    let points_str: String = points
        .iter()
        .map(|p| format!("{},{}", p.x, p.y))
        .collect::<Vec<_>>()
        .join(" ");

    format!(
        r#"<polygon points="{}" fill="{}" stroke="black" stroke-width="20" fill-opacity="{}" />
"#,
        points_str, color, opacity
    )
}
