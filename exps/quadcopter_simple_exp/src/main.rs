use std::env;

use rtreach::face_lift::ITERATIONS_AT_QUIT;
use quadcopter::dynamics_quadcopter::QuadcopterModel;
use quadcopter::quadcopter_model::{get_simulated_safe_time, run_reachability_quadcopter};
use rtreach::util::{save_rects_to_csv, save_states_to_csv};
use rtreach::geometry::println;

const STATES_FILE_PATH: &str = "data/quadcopter/simple_exp_states.csv";
const RECTS_FILE_PATH: &str = "data/quadcopter/simple_exp_rects.csv";
fn main() {
    // Get the current working directory
    let current_dir = env::current_dir().expect("Failed to get current directory");

    let runtime_ms: u64 = 100;
    let reach_time: f64 = 2.0;
    let start_ms: u64 = 0;
    let init_step_size: f64 = 0.2;
    let start_state: [f64; 12] = [0.0; 12];
    let control_input: Vec<f64> = vec![0.0, 0.001, -0.001, 0.0];
    let fixed_step: bool = false;
    let store_state: bool = true;
    let store_rects: bool = true;

    println!("initial state: ");
    for i in 0..12 {
        println!("x[{}]: {}", i, start_state[i]);
    }
    println!();

    let quad_model = QuadcopterModel;
    let (_, storage_states) = get_simulated_safe_time(&quad_model, start_state, &control_input, store_state);
    println!("Number of States: {}\n", storage_states.len());
    // if store_state {
    //     save_states_to_csv(current_dir.join(STATES_FILE_PATH).to_str().unwrap(), &storage_states);
    // }

    println!("Running reachability analysis\n");
    let (safe, storage_rects) = run_reachability_quadcopter(&quad_model, 
                                                                                            start_state, 
                                                                                            reach_time,
                                                                                            init_step_size, 
                                                                                            runtime_ms, 
                                                                                            start_ms, 
                                                                                            &control_input,
                                                                                            store_rects, 
                                                                                            fixed_step);
    println!("Number of Rectangles: {}\n", storage_rects.len());
    if store_rects {
        println!("Last Rectangle (Reachtube): ");
        println(&storage_rects[storage_rects.len()-1]);
        // save_rects_to_csv(current_dir.join(RECTS_FILE_PATH).to_str().unwrap(), &storage_rects);
    }
    

    let iters: u64 = *ITERATIONS_AT_QUIT.lock().unwrap();
    println!("Number of Iterations: {}", iters);
    println!("done, result = {}", if safe { "safe" } else { "unsafe" });
}
