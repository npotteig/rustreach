mod rtreach;
mod bicycle;

use rtreach::debug::DEBUG;
use rtreach::face_lift::ITERATIONS_AT_QUIT;
use bicycle::dynamics_bicycle::BicycleModel;
use bicycle::bicycle_model::{get_simulated_safe_time, run_reachability_bicycle};
use rtreach::obstacle_safety::{allocate_obstacles, load_wallpoints};

const FILE_PATH: &str = "data/porto_obstacles.txt";
fn main() {
    let args: Vec<String> = std::env::args().collect();
    if DEBUG {
        println!("started");
        println!("Argc: {}", args.len());
    }

    let mut runtime_ms: u64 = 0;
    let mut start_state: [f64; 4] = [0.0; 4];
    let mut control_input: [f64; 2] = [0.0; 2];

    if args.len() < 8{
        println!("Error: not enough input arguments!");
        std::process::exit(1);
    }
    else {
        runtime_ms = args[1].parse::<u64>().unwrap();
        start_state[0] = args[2].parse::<f64>().unwrap();
        start_state[1] = args[3].parse::<f64>().unwrap();
        start_state[2] = args[4].parse::<f64>().unwrap();
        start_state[3] = args[5].parse::<f64>().unwrap();
        control_input[0] = args[6].parse::<f64>().unwrap();
        control_input[1] = args[7].parse::<f64>().unwrap();
        if DEBUG{
            println!("runtime: {} ms\n\rx_0[0]: {}\n\rx_0[1]: {}\n\rx_0[2]: {}\n\rx_0[3]: {}\n\ru_0[0]: {}\n\ru_0[1]: {}\n\r\n", runtime_ms, start_state[0], start_state[1], start_state[2], start_state[3], control_input[0], control_input[1]);
        }
    }

    let delta: f64 = control_input[1];
    let u: f64 = control_input[0];
    let bicycle_model = BicycleModel;
    // simulate the car with a constant input passed from the command line
    get_simulated_safe_time(&bicycle_model, start_state, delta, u);
    println!();

    // location of obstacles in our scenario
    let num_obstacles: u32 = 5;
    let points: [[f64; 2]; 5] = [[2.0,2.0],[4.7,2.7],[11.36,-1.46],[3.0,6.4],[-9.64,2.96]];
    allocate_obstacles(num_obstacles, &points);

    // sim time
    let time_to_safe: f64 = 2.0;
    let start_ms: u64 = 0;

    // load the wall points into the global variable
    load_wallpoints(FILE_PATH, true);

    // run reachability analysis test 
    let safe: bool = run_reachability_bicycle(&bicycle_model, start_state, time_to_safe, runtime_ms, start_ms, delta, u);

    //int runtimeMs = 20; // run for 20 milliseconds
    if DEBUG {
        let iters: u64 = *ITERATIONS_AT_QUIT.lock().unwrap();
        println!("Number of Iterations: {}", iters);
        println!("done, result = {}", if safe { "safe" } else { "unsafe" });
    }
}