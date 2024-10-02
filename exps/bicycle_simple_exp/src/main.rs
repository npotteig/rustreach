use std::env;
use std::path::{Path, PathBuf};

use rtreach::debug::DEBUG;
use rtreach::face_lift::ITERATIONS_AT_QUIT;
use bicycle::dynamics_bicycle::BicycleModel;
use bicycle::bicycle_model::{get_simulated_safe_time, run_reachability_bicycle};
use rtreach::obstacle_safety::{allocate_obstacles, load_wallpoints};
use rtreach::util::{save_rects_to_csv, save_states_to_csv};

const OBS_FILE_PATH: &str = "data/porto_obstacles.txt";
const STATES_FILE_PATH: &str = "data/porto_states.csv";
const RECTS_FILE_PATH: &str = "data/porto_rects.csv";
fn main() {
    // Get the current working directory
    let current_dir = env::current_dir().expect("Failed to get current directory");

    // Go two directories up
    let parent_dir = current_dir.parent().unwrap();
    let obs_file_path = parent_dir.join(OBS_FILE_PATH);

    let args: Vec<String> = std::env::args().collect();
    if DEBUG {
        println!("started");
        println!("Argc: {}", args.len());
    }

    let mut runtime_ms: u64 = 0;
    let mut reach_time: f64 = 0.0;
    let mut init_step_size: f64 = 0.0;
    let mut start_state: [f64; 4] = [0.0; 4];
    let mut control_input: [f64; 2] = [0.0; 2];
    let mut load_obstacles: bool = false;
    let mut fixed_step: bool = false;

    if args.len() != 12{
        println!("Error: Usage: -- (milliseconds-runtime) (seconds-reachtime) (seconds-initial-stepsize) (x) (y) (linear velocity) (heading) (throttle control input) (heading control input) (load obstacles) (fixed step)");
        std::process::exit(1);
    }
    else {
        runtime_ms = args[1].parse::<u64>().unwrap();
        reach_time = args[2].parse::<f64>().unwrap();
        init_step_size = args[3].parse::<f64>().unwrap();
        start_state[0] = args[4].parse::<f64>().unwrap();
        start_state[1] = args[5].parse::<f64>().unwrap();
        start_state[2] = args[6].parse::<f64>().unwrap();
        start_state[3] = args[7].parse::<f64>().unwrap();
        control_input[0] = args[8].parse::<f64>().unwrap();
        control_input[1] = args[9].parse::<f64>().unwrap();
        load_obstacles = args[10].parse::<bool>().unwrap();
        fixed_step = args[11].parse::<bool>().unwrap();
        if DEBUG{
            println!("runtime: {} ms\n\rx_0[0]: {}\n\rx_0[1]: {}\n\rx_0[2]: {}\n\rx_0[3]: {}\n\ru_0[0]: {}\n\ru_0[1]: {}\n\r\n", runtime_ms, start_state[0], start_state[1], start_state[2], start_state[3], control_input[0], control_input[1]);
        }
    }

    let delta: f64 = control_input[1];
    let u: f64 = control_input[0];
    let bicycle_model = BicycleModel;
    // simulate the car with a constant input passed from the command line
    let store_state: bool = true;
    let (_, storage_states) = get_simulated_safe_time(&bicycle_model, start_state, delta, u, store_state);
    println!("Number of States: {}\n", storage_states.len());
    // save_states_to_csv(STATES_FILE_PATH, &storage_states);

    // location of obstacles in our scenario
    // load the wall points into the global variable
    if load_obstacles   {
        let num_obstacles: u32 = 5;
        let points: [[f64; 2]; 5] = [[2.0,2.0],[4.7,2.7],[11.36,-1.46],[3.0,6.4],[-9.64,2.96]];
        allocate_obstacles(num_obstacles, &points);
        load_wallpoints(obs_file_path.to_str().unwrap(), true);
    }

    // sim time
    let start_ms: u64 = 0;

    // run reachability analysis test 
    let store_rects: bool = true;
    let (safe, storage_rects) = run_reachability_bicycle(&bicycle_model, 
                                                                                       start_state, 
                                                                                       reach_time,
                                                                                       init_step_size, 
                                                                                       runtime_ms, 
                                                                                       start_ms, 
                                                                                       delta, 
                                                                                       u, 
                                                                                       store_rects, 
                                                                                       fixed_step);
    println!("Number of Rectangles: {}\n", storage_rects.len());
    // save_rects_to_csv(RECTS_FILE_PATH, &storage_rects);

    //int runtimeMs = 20; // run for 20 milliseconds
    if DEBUG {
        let iters: u64 = *ITERATIONS_AT_QUIT.lock().unwrap();
        println!("Number of Iterations: {}", iters);
        println!("done, result = {}", if safe { "safe" } else { "unsafe" });
    }
}