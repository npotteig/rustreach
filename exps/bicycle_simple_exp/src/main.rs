use std::env;
use tract_onnx::prelude::*;

use rtreach::face_lift::ITERATIONS_AT_QUIT;
use bicycle::dynamics_bicycle::{BicycleModel, BICYCLE_NUM_DIMS as NUM_DIMS};
use bicycle::simulate_bicycle::step_bicycle;
use bicycle::bicycle_model::{get_simulated_safe_time, run_reachability_bicycle};
use rtreach::obstacle_safety::{allocate_obstacles, load_wallpoints};
use rtreach::util::{save_rects_to_csv, save_states_to_csv};
use bicycle::utils::normalize_angle;
use bicycle::controller::model_sample_action;
use rtreach::geometry::println;

const OBS_FILE_PATH: &str = "data/porto_obstacles.txt";
const STATES_FILE_PATH: &str = "data/bicycle/paper/approach/gt_ctrl_states.csv";
const RECTS_FC_FILE_PATH: &str = "data/bicycle/paper/approach/rects_fc.csv";
const RECTS_DC_FILE_PATH: &str = "data/bicycle/paper/approach/sg_selection/rects_dc.csv";
fn main() -> TractResult<()> {
    // Get the current working directory
    let current_dir = env::current_dir().expect("Failed to get current directory");

    let obs_file_path = current_dir.join(OBS_FILE_PATH);

    // Load the ONNX model from file
    let model = tract_onnx::onnx()
        .model_for_path("models/bicycle_model_actor.onnx")?
        // specify input type and shape
        .with_input_fact(0, f64::fact([1, 4]).into())?
        .into_optimized()?        // Optimize the model for performance
        .into_runnable()?;         // Make it runnable

    let args: Vec<String> = std::env::args().collect();
    println!("started");
    println!("Argc: {}", args.len());

    let runtime_ms: u64 = 10;
    let reach_time: f64 = 2.0;
    let init_step_size: f64 = 0.1;
    let start_state: [f64; 4] = [0.0; 4];
    // let start_state = [0.3105531022023626,0.45279829645064384,0.7241154063813101,0.2935034007631414];
    let mut state = start_state.clone();
    let goal = [1.0, 1.0];
    // let goal = [3.0, 0.0];
    let mut ctrl_input: [f64; 2] = [0.0; 2];
    let load_obstacles: bool = false;
    let fixed_step: bool = false;

    let step_size = 0.1;
    let mut step = 0;
    let total_steps = 20;

    // if args.len() != 12{
    //     println!("Error: Usage: -- (milliseconds-runtime) (seconds-reachtime) (seconds-initial-stepsize) (x) (y) (linear velocity) (heading) (throttle control input) (heading control input) (load obstacles) (fixed step)");
    //     std::process::exit(1);
    // }
    // else {
    //     runtime_ms = args[1].parse::<u64>().unwrap();
    //     reach_time = args[2].parse::<f64>().unwrap();
    //     init_step_size = args[3].parse::<f64>().unwrap();
    //     start_state[0] = args[4].parse::<f64>().unwrap();
    //     start_state[1] = args[5].parse::<f64>().unwrap();
    //     start_state[2] = args[6].parse::<f64>().unwrap();
    //     start_state[3] = args[7].parse::<f64>().unwrap();
    //     control_input[0] = args[8].parse::<f64>().unwrap();
    //     control_input[1] = args[9].parse::<f64>().unwrap();
    //     load_obstacles = args[10].parse::<bool>().unwrap();
    //     fixed_step = args[11].parse::<bool>().unwrap();
    //     println!("runtime: {} ms\n\rx_0[0]: {}\n\rx_0[1]: {}\n\rx_0[2]: {}\n\rx_0[3]: {}\n\ru_0[0]: {}\n\ru_0[1]: {}\n\r\n", runtime_ms, start_state[0], start_state[1], start_state[2], start_state[3], control_input[0], control_input[1]);
    // }

    // let delta: f64 = control_input[1];
    // let u: f64 = control_input[0];
    let mut bicycle_model = BicycleModel::default();
    bicycle_model.set_ctrl_fn(model_sample_action);
    bicycle_model.set_goal(goal);
    bicycle_model.set_model(&model);
    // simulate the car with a constant input passed from the command line
    // let store_state: bool = true;
    // let (_, storage_states) = get_simulated_safe_time(&bicycle_model, start_state, delta, u, store_state);
    // println!("Number of States: {}\n", storage_states.len());
    // save_states_to_csv(STATES_FILE_PATH, &storage_states);

    let mut states_vec: Vec<[f64; NUM_DIMS]> = Vec::new();

    while step < total_steps {
        ctrl_input = bicycle_model.sample_state_action(&state);
        let mut next_state = step_bicycle(&bicycle_model, &state, ctrl_input[0], ctrl_input[1], step_size);
        
        next_state[3] = normalize_angle(next_state[3]);
        states_vec.push(next_state);
        state = next_state;
        step += 1;
    }
    save_states_to_csv(STATES_FILE_PATH, &states_vec);

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
    ctrl_input = bicycle_model.sample_state_action(&start_state);
    let (safe_fc, storage_rects_fc) = run_reachability_bicycle(&bicycle_model, 
                                                                                       start_state, 
                                                                                       reach_time,
                                                                                       init_step_size, 
                                                                                       runtime_ms, 
                                                                                       start_ms, 
                                                                                       ctrl_input[0], 
                                                                                       ctrl_input[1], 
                                                                                       store_rects, 
                                                                                       fixed_step,
                                                                                    false);

    let (safe_dc, storage_rects_dc) = run_reachability_bicycle(&bicycle_model, 
                                                                                       start_state, 
                                                                                       reach_time,
                                                                                       init_step_size, 
                                                                                       runtime_ms, 
                                                                                       start_ms, 
                                                                                       ctrl_input[0], 
                                                                                       ctrl_input[1], 
                                                                                       store_rects, 
                                                                                       fixed_step,
                                                                                    true);
    // println!("Number of Rectangles: {}\n", storage_rects.len());
    if store_rects {
        // println!("Last Rectangle: ");
        // println(&storage_rects[storage_rects.len()-1]);
        save_rects_to_csv(current_dir.join(RECTS_FC_FILE_PATH).to_str().unwrap(), &storage_rects_fc);
        save_rects_to_csv(current_dir.join(RECTS_DC_FILE_PATH).to_str().unwrap(), &storage_rects_dc);
    }

    //int runtimeMs = 20; // run for 20 milliseconds
    // let iters: u64 = *ITERATIONS_AT_QUIT.lock().unwrap();
    // println!("Number of Iterations: {}", iters);
    // println!("done, result = {}", if safe { "safe" } else { "unsafe" });

    Ok(())
}