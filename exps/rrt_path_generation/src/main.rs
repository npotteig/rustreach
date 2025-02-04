
use std::{env, vec};
use rand::distributions::{Distribution, Uniform};
use std::error::Error;
use std::fs::File;


use csv::Writer;
use rrt::dual_rrt_connect;
use pbr::ProgressBar;

use rtreach::obstacle_safety::load_obstacles_from_csv_old;
use rtreach::util::load_paths_from_csv;

const PATH_DATASET_PARENT: &str = "eval_input_data/";
const OBSTACLE_DATASET_PATH: &str = "eval_input_data/rr_nbd_obstacles_no_interior.csv";
fn main() {
    let current_dir: std::path::PathBuf = env::current_dir().expect("Failed to get current directory");
    
    let waypt_algorithm = "astar";
    let path_dataset_parent = current_dir.join(PATH_DATASET_PARENT);
    let path_dataset_path = path_dataset_parent.join(format!("{}_rustreach_paths.csv", waypt_algorithm));
    let obstacle_dataset_path = current_dir.join(OBSTACLE_DATASET_PATH);
    let paths_vec = load_paths_from_csv(&path_dataset_path);
    let obstacles_vec = load_obstacles_from_csv_old(&obstacle_dataset_path);

    let mut pb = ProgressBar::new(1000);
    let mut data: Vec<Vec<Vec<f64>>> = vec![];
    for pth in paths_vec.iter(){
        pb.inc();
        let start = [pth[0][0], pth[0][1]];
        let goal = [pth[pth.len()-1][0], pth[pth.len()-1][1]];
        let result = dual_rrt_connect(
            &start,
            &goal,
            |p: &[f64]| {
                let mut is_valid = true;
                for obs in &obstacles_vec {
                    if distance(p, obs) < 1.5 {
                        is_valid = false;
                        break;
                    }
                }
                is_valid
            },
            || {
                let between = Uniform::new(-50.0, 50.0);
                let mut rng = rand::thread_rng();
                vec![between.sample(&mut rng), between.sample(&mut rng)]
            },
            3.0,
            10000,
        ).unwrap();
        // println!("{result:?}");
        assert!(result.len() > 0);
        data.push(result);
    }

    if let Err(e) = save_to_csv(&data, "eval_input_data/rrt_rustreach_paths.csv") {
        eprintln!("Error saving file: {}", e);
    }
}

fn distance(p1: &[f64], p2: &[f64]) -> f64 {
    ((p1[0] - p2[0]).powi(2) + (p1[1] - p2[1]).powi(2)).sqrt()
}

fn save_to_csv(data: &Vec<Vec<Vec<f64>>>, filename: &str) -> Result<(), Box<dyn Error>> {
    let file = File::create(filename)?;
    let mut wtr = Writer::from_writer(file);

    // Write the header
    wtr.write_record(&["path_id", "x", "y", "z"])?;

    // Write data rows
    for (path_id, path) in data.iter().enumerate() {
        for point in path {
            wtr.write_record(&[
                path_id.to_string(),
                format!("{:.6}", point[0]),
                format!("{:.6}", point[1]),
                "-5.000000".to_string(),
            ])?;
        }
    }

    // Flush and close the writer
    wtr.flush()?;
    Ok(())
}