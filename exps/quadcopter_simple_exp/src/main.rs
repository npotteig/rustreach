use std::env;
use std::fs;

use tract_onnx::prelude::*;

use quadcopter::dynamics_quadcopter::{QUAD_NUM_DIMS as NUM_DIMS, QuadcopterModel};
use quadcopter::quadcopter_model::run_reachability_quadcopter;
use quadcopter::controller::model_sample_action;
use quadcopter::simulate_quadcopter::simulate_quadcopter;
use quadcopter::utils::normalize_angle;
use rtreach::util::{save_rects_to_csv, save_states_to_csv};
use rtreach::geometry::println;

const STATES_FILE_PATH: &str = "data/quadcopter/simple_exp/gt_ctrl_states.csv";
const RECTS_FC_FILE_PATH: &str = "data/quadcopter/simple_exp/rects_fc.csv";
const RECTS_RLC_FILE_PATH: &str = "data/quadcopter/simple_exp/rects_rlc.csv";
fn main() -> TractResult<()>{
    let save_data = true;
    // Get the current working directory
    let current_dir = env::current_dir().expect("Failed to get current directory");

    let states_path = current_dir.join(STATES_FILE_PATH);
    let rects_fc_path = current_dir.join(RECTS_FC_FILE_PATH);
    let rects_dc_path = current_dir.join(RECTS_RLC_FILE_PATH);
    if save_data{
        if let Some(parent) = states_path.parent() {
            println!("Saving data to: {:?}", parent);
            fs::create_dir_all(parent)?; // Creates parent directories if they don't exist
        }
    }

    // Load the ONNX model from file
    let model = tract_onnx::onnx()
        .model_for_path("models/quadcopter_model_actor.onnx")?
        // specify input type and shape
        .with_input_fact(0, f64::fact([1, 12]).into())?
        .into_optimized()?        // Optimize the model for performance
        .into_runnable()?;         // Make it runnable

    let runtime_ms: u64 = 10;
    let reach_time: f64 = 2.0;
    let init_step_size: f64 = 0.1;
    let euler_step_size: f64 = 0.0002;
    let start_state: [f64; 12] = [0.0; 12];
    let mut state = start_state.clone();
    let goal = [1.0, 1.0, 0.0];
    let mut ctrl_input;
    let fixed_step: bool = false;

    let step_size = 0.1;
    let mut step = 0;
    let total_steps = 20;

    let mut quadcopter_model = QuadcopterModel::default();
    quadcopter_model.set_ctrl_fn(model_sample_action);
    quadcopter_model.set_goal(goal);
    quadcopter_model.set_model(&model);

    let mut states_vec: Vec<[f64; NUM_DIMS]> = Vec::new();

    while step < total_steps {
        ctrl_input = quadcopter_model.sample_state_action(&state).to_vec();
        let mut next_state = simulate_quadcopter(&quadcopter_model, state, &ctrl_input, euler_step_size, step_size);
        
        next_state[3] = normalize_angle(next_state[3]); // phi
        next_state[4] = normalize_angle(next_state[4]); // theta
        next_state[5] = normalize_angle(next_state[5]); // psi
        states_vec.push(next_state);
        state = next_state;
        step += 1;
    }
    if save_data{
        save_states_to_csv(states_path.to_str().unwrap(), &states_vec);
    }
    println!("Final ground truth state: {:?}\n", state);

    // sim time
    let start_ms: u64 = 0;

    // run reachability analysis test 
    let store_rects: bool = true;
    ctrl_input = quadcopter_model.sample_state_action(&start_state).to_vec();
    let (_, storage_rects_fc) = run_reachability_quadcopter(&quadcopter_model, 
                                                                                       start_state, 
                                                                                       reach_time,
                                                                                       init_step_size, 
                                                                                       runtime_ms, 
                                                                                       start_ms, 
                                                                                       &ctrl_input, 
                                                                                       store_rects, 
                                                                                       fixed_step,
                                                                                    false);

    let (_, storage_rects_dc) = run_reachability_quadcopter(&quadcopter_model, 
                                                                                       start_state, 
                                                                                       reach_time,
                                                                                       init_step_size, 
                                                                                       runtime_ms, 
                                                                                       start_ms, 
                                                                                       &ctrl_input, 
                                                                                       store_rects, 
                                                                                       fixed_step,
                                                                                    true);
    if save_data {
        save_rects_to_csv(rects_fc_path.to_str().unwrap(), &storage_rects_fc);
        save_rects_to_csv(rects_dc_path.to_str().unwrap(), &storage_rects_dc);
    }
    println!("Final Hyperrectangle for Fixed Control: ");
    println(&storage_rects_fc[storage_rects_fc.len()-2]);

    println!("Final Hyperrectangle for Dynamic RL Control: ");
    println(&storage_rects_dc[storage_rects_dc.len()-2]);



    Ok(())
}
