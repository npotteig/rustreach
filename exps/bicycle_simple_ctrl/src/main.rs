use std::f64::consts::PI;
use std::env;
use tract_onnx::prelude::*;

use rtreach::obstacle_safety::allocate_obstacles;
use rtreach::util::{save_states_to_csv, save_reachtubes_to_csv};
use rtreach::geometry::HyperRectangle;

use bicycle::simulate_bicycle::step_bicycle;
use bicycle::bicycle_model::has_collided;
use bicycle::dynamics_bicycle::{BicycleModel, BICYCLE_NUM_DIMS as NUM_DIMS};
use bicycle::utils::{distance, normalize_angle};
use bicycle::controller::{select_safe_subgoal_rtreach, select_safe_subgoal_circle, goal_conditioned_sample_action, model_sample_action};

const STATES_FILE_PATH: &str = "data/bicycle/ctrl_states.csv";
const SUBGOAL_FILE_PATH: &str = "data/bicycle/subgoals.csv";
const REACHTUBE_FILE_PATH: &str = "data/bicycle/reachtubes.csv";

fn main() -> TractResult<()> { 
    // Get the current working directory
    let current_dir = env::current_dir().expect("Failed to get current directory");

    let states_file_path = current_dir.join(STATES_FILE_PATH);
    let subgoal_file_path = current_dir.join(SUBGOAL_FILE_PATH);
    let reachtube_file_path = current_dir.join(REACHTUBE_FILE_PATH);

    // Load the ONNX model from file
    let model = tract_onnx::onnx()
        .model_for_path("models/bicycle_model_actor.onnx")?
        // specify input type and shape
        .with_input_fact(0, f64::fact([1, 4]).into())?
        .into_optimized()?        // Optimize the model for performance
        .into_runnable()?;         // Make it runnable

    let mut bicycle_model = BicycleModel::default();

    let num_obstacles: u32 = 2;
    let points: [[f64; 2]; 2] = [[2.,0.7], [2., -0.7]];
    allocate_obstacles(num_obstacles, &points);

    // Start & Goal States
    let mut start_state = [0.0; NUM_DIMS];
    let goal_list = [[4., 0.]];
    let mut goal_idx = 0;
    start_state[0] = 0.0;       // x
    start_state[1] = 0.0;       // y
    start_state[2] = 0.0;       // v
    start_state[3] = PI/2.0;       // theta

    // let mut ctrl_input = [0.0; 2];
    // ctrl_input[0] = 1.0;     // throttle
    // ctrl_input[1] = 0.0;     // heading

    // Data Storage
    let save_data = true;
    let mut states_vec: Vec<[f64; NUM_DIMS]> = Vec::new();
    let mut subgoal_vec: Vec<[f64; 2]> = Vec::new();
    let mut reachtube_vec: Vec<Vec<HyperRectangle<NUM_DIMS>>> = Vec::new();

    let mut state = start_state.clone();
    states_vec.push(state);

    // Simulation Parameters
    let step_size = 0.1;  // seconds
    let total_steps = 100;
    let mut time = 0.0;
    let mut step = 0;
    let mut collided = false;
    let thresh = 0.2;

    // Control Parameters
    let learning_enabled = false;
    let use_subgoal_ctrl = true;
    let use_rtreach = true;
    let use_rtreach_dynamic_control = true;
    let pi_low = model_sample_action;
    let sim_time = 2.0;
    let wall_time_ms = 100;
    let start_ms = 0;
    let store_rect = true;
    let fixed_step = false;
    let num_subgoal_cands = 10;

    bicycle_model.set_ctrl_fn(pi_low);
    bicycle_model.set_goal(goal_list[goal_idx]);
    if learning_enabled {
        bicycle_model.set_model(&model);
    }

    for i in 0..goal_list.len() {
        println!("Goal {}: [{}, {}]", i, goal_list[i][0], goal_list[i][1]);
    }
    println!("The state at time 0 s is: \n [{},{},{},{}] \n",state[0],state[1],state[2],state[3]);

    while step < total_steps && distance(&state, &goal_list[goal_list.len() - 1]) > thresh {
        if distance(&state, &goal_list[goal_idx]) < thresh {
            println!("Goal {} Reached [{}, {}]", goal_idx, goal_list[goal_idx][0], goal_list[goal_idx][1]);
            goal_idx += 1;
            bicycle_model.set_goal(goal_list[goal_idx]);
        }

        let ctrl_input;
        if use_subgoal_ctrl {
            let (safe, subgoal, storage_vec) = 
            if use_rtreach {
                select_safe_subgoal_rtreach(&mut bicycle_model, state, [start_state[0], start_state[1]], goal_list[goal_idx], num_subgoal_cands, sim_time, step_size, wall_time_ms, start_ms, store_rect, fixed_step, use_rtreach_dynamic_control)
            }
            else{
                select_safe_subgoal_circle(&state, [start_state[0], start_state[1]], goal_list[goal_idx], num_subgoal_cands*10)
            };
            
            subgoal_vec.push(subgoal);
            reachtube_vec.push(storage_vec);
            if !safe {
                println!("No safe subgoal found");
                break;
            }
            bicycle_model.set_goal(subgoal);
            ctrl_input = bicycle_model.sample_state_action(&state);
        }
        else {
            ctrl_input = bicycle_model.sample_state_action(&state);
        }

        let mut next_state = step_bicycle(&bicycle_model, &state, ctrl_input[0], ctrl_input[1], step_size);
        
        next_state[3] = normalize_angle(next_state[3]);
        states_vec.push(next_state);
        time += step_size;
        state = next_state;
        if has_collided(&state) {
            collided = true;
            break;
        }
        step += 1;
    }
    if collided{
        println!("Collision Detected");
    }
    else if distance(&state, &goal_list[goal_list.len() - 1]) < thresh{
        println!("Goal {} Reached [{}, {}]", goal_idx, goal_list[goal_list.len() - 1][0], goal_list[goal_list.len() - 1][1]);
    }
    else{
        println!("Max Steps Reached");
    }
    if save_data{
        save_states_to_csv(states_file_path.to_str().unwrap(), &states_vec);
        if use_subgoal_ctrl{
            save_states_to_csv(subgoal_file_path.to_str().unwrap(), &subgoal_vec);
            save_reachtubes_to_csv(reachtube_file_path.to_str().unwrap(), &reachtube_vec);
        }
    }
    println!("The state after {} s is: \n [{},{},{},{}] \n", time-step_size,state[0],state[1],state[2],state[3]);

    Ok(())
}