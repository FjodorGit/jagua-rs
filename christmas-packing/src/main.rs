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
use rand::SeedableRng;
use rand::prelude::SmallRng;
use std::fs::{self, File};
use std::io::BufReader;
use std::path::{Path, PathBuf};

fn main() -> Result<()> {
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
