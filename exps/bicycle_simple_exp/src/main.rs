use std::env;
use std::fs;

use tract_onnx::prelude::*;

use rtreach::geometry::println;
use bicycle::dynamics_bicycle::{BicycleModel, BICYCLE_NUM_DIMS as NUM_DIMS};
use bicycle::simulate_bicycle::step_bicycle;
use bicycle::bicycle_model::run_reachability_bicycle;
use rtreach::util::{save_rects_to_csv, save_states_to_csv};
use bicycle::utils::normalize_angle;
use bicycle::controller::model_sample_action;

const STATES_FILE_PATH: &str = "data/bicycle/simple_exp/gt_ctrl_states.csv";
const RECTS_FC_FILE_PATH: &str = "data/bicycle/simple_exp/rects_fc.csv";
const RECTS_RLC_FILE_PATH: &str = "data/bicycle/simple_exp/rects_rlc.csv";
fn main() -> TractResult<()> {
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
        .model_for_path("models/bicycle_model_actor.onnx")?
        // specify input type and shape
        .with_input_fact(0, f64::fact([1, 4]).into())?
        .into_optimized()?        // Optimize the model for performance
        .into_runnable()?;         // Make it runnable

    let runtime_ms: u64 = 10;
    let reach_time: f64 = 2.0;
    let init_step_size: f64 = 0.1;
    let start_state: [f64; 4] = [0.0; 4];
    let mut state = start_state.clone();
    let goal = [1.0, 1.0];
    let mut ctrl_input: [f64; 2];
    let fixed_step: bool = false;

    let step_size = 0.1;
    let mut step = 0;
    let total_steps = 20;

    let mut bicycle_model = BicycleModel::default();
    bicycle_model.set_ctrl_fn(model_sample_action);
    bicycle_model.set_goal(goal);
    bicycle_model.set_model(&model);

    let mut states_vec: Vec<[f64; NUM_DIMS]> = Vec::new();

    while step < total_steps {
        ctrl_input = bicycle_model.sample_state_action(&state);
        let mut next_state = step_bicycle(&bicycle_model, &state, ctrl_input[0], ctrl_input[1], step_size);
        
        next_state[3] = normalize_angle(next_state[3]);
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
    ctrl_input = bicycle_model.sample_state_action(&start_state);
    let (_, storage_rects_fc) = run_reachability_bicycle(&bicycle_model, 
                                                                                       start_state, 
                                                                                       reach_time,
                                                                                       init_step_size, 
                                                                                       runtime_ms, 
                                                                                       start_ms, 
                                                                                       ctrl_input[0], 
                                                                                       ctrl_input[1], 
                                                                                       store_rects, 
                                                                                       fixed_step,
                                                                                    false,
                                                                                       obstacle_sim_fn_static);

    let (_, storage_rects_dc) = run_reachability_bicycle(&bicycle_model, 
                                                                                       start_state, 
                                                                                       reach_time,
                                                                                       init_step_size, 
                                                                                       runtime_ms, 
                                                                                       start_ms, 
                                                                                       ctrl_input[0], 
                                                                                       ctrl_input[1], 
                                                                                       store_rects, 
                                                                                       fixed_step,
                                                                                    true,
                                                                                       obstacle_sim_fn_static);
    if save_data {
        save_rects_to_csv(rects_fc_path.to_str().unwrap(), &storage_rects_fc);
        save_rects_to_csv(rects_dc_path.to_str().unwrap(), &storage_rects_dc);
    }
    println!("Final Hyperrectangle for Fixed Control: ");
    println(&storage_rects_fc[storage_rects_fc.len()-2].1);

    println!("Final Hyperrectangle for Dynamic RL Control: ");
    println(&storage_rects_dc[storage_rects_dc.len()-2].1);

    Ok(())
}

fn obstacle_sim_fn_static(_: f64, _: &mut Vec<Vec<Vec<f64>>>) {
    // Do nothing
}