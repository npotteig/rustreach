use std::env;
use tract_onnx::prelude::*;

use rtreach::obstacle_safety::allocate_obstacles;
use rtreach::util::{save_states_to_csv, save_reachtubes_to_csv};
use rtreach::geometry::HyperRectangle;

use quadcopter::simulate_quadcopter::simulate_quadcopter;
use quadcopter::quadcopter_model::has_collided;
use quadcopter::dynamics_quadcopter::{QuadcopterModel, QUAD_NUM_DIMS as NUM_DIMS};
use quadcopter::controller::{select_safe_subgoal_circle, select_safe_subgoal_rtreach, model_sample_action};
use quadcopter::utils::{distance, normalize_angle};

const STATES_FILE_PATH: &str = "data/quadcopter/ctrl_states.csv";
const SUBGOAL_FILE_PATH: &str = "data/quadcopter/subgoals.csv";
const REACHTUBE_FILE_PATH: &str = "data/quadcopter/reachtubes.csv";
fn main() -> TractResult<()> {
    // Get the current working directory
    let current_dir = env::current_dir().expect("Failed to get current directory");

    let states_file_path = current_dir.join(STATES_FILE_PATH);
    let subgoal_file_path = current_dir.join(SUBGOAL_FILE_PATH);
    let reachtube_file_path = current_dir.join(REACHTUBE_FILE_PATH);

    // Load the ONNX model from file
    let model = tract_onnx::onnx()
        .model_for_path("models/quadcopter_model_actor.onnx")?
        // specify input type and shape
        .with_input_fact(0, f64::fact([1, 12]).into())?
        .into_optimized()?        // Optimize the model for performance
        .into_runnable()?;         // Make it runnable

    let mut quadcopter_model = QuadcopterModel::default();

    let num_obstacles: u32 = 2;
    let points: [[f64; 2]; 2] = [[2.,0.7], [2., -0.7]];
    allocate_obstacles(num_obstacles, &points);

    // Start & Goal States
    let mut start_state = [0.0; NUM_DIMS];
    start_state[7] = 0.6; // v_y
    let goal_list = [[4., 0., 0.]];
    let mut goal_idx = 0;

    // let mut ctrl_input = vec![0.0; 4];
    // ctrl_input[0] = 0.0;     // thrust upward
    // ctrl_input[1] = 0.001;     // x torque
    // ctrl_input[2] = -0.001;     // y torque
    // ctrl_input[3] = 0.0;     // z torque

    // Data Storage
    let save_data = false;
    let mut states_vec: Vec<[f64; NUM_DIMS]> = Vec::new();
    let mut subgoal_vec: Vec<[f64; 3]> = Vec::new();
    let mut reachtube_vec: Vec<Vec<HyperRectangle<NUM_DIMS>>> = Vec::new();

    let mut state = start_state.clone();
    states_vec.push(state);

    // Simulation Parameters
    let step_size = 0.1;  // seconds
    let euler_step_size = 0.0002;
    let total_steps = 100;
    let mut time = 0.0;
    let mut step = 0;
    let mut collided = false;
    let thresh = 0.25;

    // Control Parameters
    let learning_enabled = true;
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

    quadcopter_model.set_ctrl_fn(pi_low);
    quadcopter_model.set_goal(goal_list[goal_idx]);
    if learning_enabled {
        quadcopter_model.set_model(&model);
    }

    for i in 0..goal_list.len() {
        println!("Goal {}: [{}, {}, {}]", i, goal_list[i][0], goal_list[i][1], goal_list[i][2]);
    }

    println!("initial state: ");
    for i in 0..12 {
        println!("x[{}]: {}", i, state[i]);
    }
    println!();

    while step < total_steps && distance(&state, &goal_list[goal_list.len() - 1]) > thresh {
        if distance(&state, &goal_list[goal_idx]) < thresh {
            println!("Goal {} Reached [{}, {}, {}]", goal_idx, goal_list[goal_idx][0], goal_list[goal_idx][1], goal_list[goal_idx][2]);
            goal_idx += 1;
        }

        let ctrl_input;
        if use_subgoal_ctrl {
            let (safe, subgoal, storage_vec) = 
            if use_rtreach {
                select_safe_subgoal_rtreach(&mut quadcopter_model, state, [start_state[0], start_state[1], start_state[2]], goal_list[goal_idx], num_subgoal_cands, sim_time, step_size, wall_time_ms, start_ms, store_rect, fixed_step, use_rtreach_dynamic_control, false)
            }
            else{
                select_safe_subgoal_circle(&state, [start_state[0], start_state[1], start_state[2]], goal_list[goal_idx], num_subgoal_cands*10, false)
            };
            
            if !safe {
                println!("No safe subgoal found");
                break;
            }
            subgoal_vec.push(subgoal);
            reachtube_vec.push(storage_vec);
            quadcopter_model.set_goal(subgoal);
            ctrl_input = quadcopter_model.sample_state_action(&state).to_vec();
        }
        else {
            ctrl_input = quadcopter_model.sample_state_action(&state).to_vec();
        }
        // println!("Control Input: {:?}", ctrl_input);

        let mut next_state = simulate_quadcopter(&quadcopter_model, state, &ctrl_input, euler_step_size, step_size);

        next_state[3] = normalize_angle(next_state[3]); // phi
        next_state[4] = normalize_angle(next_state[4]); // theta
        next_state[5] = normalize_angle(next_state[5]); // psi
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
        println!("Goal {} Reached [{}, {}, {}]", goal_idx, goal_list[goal_list.len() - 1][0], goal_list[goal_list.len() - 1][1], goal_list[goal_list.len() - 1][2]);
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
    println!("The state after {} s is: ", time-step_size);
    for i in 0..12 {
        println!("x[{}]: {}", i, state[i]);
    }
    println!();

    Ok(())
}
