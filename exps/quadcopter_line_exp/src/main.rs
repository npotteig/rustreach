use std::{env, vec};
use std::time::Instant;
use std::fs;

use csv::ReaderBuilder;
use tract_onnx::prelude::*;

use rtreach::obstacle_safety::allocate_obstacles;

use quadcopter::simulate_quadcopter::simulate_quadcopter;
use quadcopter::quadcopter_model::has_collided;
use quadcopter::dynamics_quadcopter::{QuadcopterModel, QUAD_NUM_DIMS as NUM_DIMS};
use quadcopter::utils::{distance, normalize_angle};
use quadcopter::controller::{select_safe_subgoal_rtreach, select_safe_subgoal_circle, model_sample_action};

const LINE_DATASET_PATH: &str = "eval_input_data/quadcopter/line_dataset.csv";
const EVAL_OUTPUT_PATH: &str = "eval_output_data/quadcopter/line_exp/line_eval_output.csv";

fn main() -> TractResult<()> {
    let save_data = false;

    // Get the current working directory
    let current_dir = env::current_dir().expect("Failed to get current directory");

    let line_dataset_path = current_dir.join(LINE_DATASET_PATH);
    let eval_output_path = current_dir.join(EVAL_OUTPUT_PATH);

    if save_data{
        if let Some(parent) = eval_output_path.parent() {
            println!("Saving data to: {:?}", parent);
            fs::create_dir_all(parent)?; // Creates parent directories if they don't exist
        }
    }

    // Create a CSV reader
    let mut rdr = ReaderBuilder::new()
        .has_headers(true)  // Set to false if there are no headers in your CSV
        .from_path(line_dataset_path)?;

    // Load the ONNX model from file
    let model = tract_onnx::onnx()
        .model_for_path("models/quadcopter_model_actor.onnx")?
        // specify input type and shape
        .with_input_fact(0, f64::fact([1, 12]).into())?
        .into_optimized()?        // Optimize the model for performance
        .into_runnable()?;         // Make it runnable

    let mut quad_model = QuadcopterModel::default();

    let num_obstacles: u32 = 2;
    let points: [[f64; 2]; 2] = [[2.,0.7], [2., -0.7]];
    allocate_obstacles(num_obstacles, &points);
    
    // Start & Goal States
    let start_state = [0.0; NUM_DIMS];

    // Simulation Parameters
    let step_size = 0.1;  // seconds
    let euler_step_size = 0.0002;
    let total_steps = 200;
    let thresh = 0.2;

    // Control Parameters
    let learning_enabled = true;
    let use_subgoal_ctrl = true;
    let use_rtreach = true;
    let use_rtreach_dynamic_control = true;
    let pi_low = model_sample_action;
    let sim_time = 2.0;
    let wall_time_ms = 100;
    let start_ms = 0;
    let store_rect = false;
    let fixed_step = false;
    let num_subgoal_cands = 5;

    quad_model.set_ctrl_fn(pi_low);
    if learning_enabled {
        quad_model.set_model(&model);
    }

    let mut index = 0;
    let mut time_vec = vec![];
    let mut collisions = vec![];
    let mut no_subgoal_ctrl = vec![];
    let mut avg_subgoal_computation_time = vec![];
    let mut max_subgoal_computation_time = vec![];
    let mut deadline_violations = vec![];

    for row in rdr.records() {
        println!("index: {}", index);
        index += 1;
        let mut state = start_state.clone();
        let mut time = 0.0;
        let mut step = 0;
        let mut collision = false;
        let mut no_subgoal = false;
        let mut start_waypoint = [0.0; 3];
        let mut goal_waypoint = [0.0; 3];
        let mut subgoal_compute_time = vec![];
        let mut deadline_violation_count = 0.0;

        let record = row?;
        state[0] = record.get(0).unwrap().parse::<f64>().unwrap();
        state[1] = record.get(1).unwrap().parse::<f64>().unwrap();
        start_waypoint[0] = record.get(2).unwrap().parse::<f64>().unwrap();
        start_waypoint[1] = record.get(3).unwrap().parse::<f64>().unwrap();
        goal_waypoint[0] = record.get(4).unwrap().parse::<f64>().unwrap();
        goal_waypoint[1] = record.get(5).unwrap().parse::<f64>().unwrap();

        quad_model.set_goal(goal_waypoint);

        while !collision && !no_subgoal && step < total_steps && distance(&state, &goal_waypoint) > thresh {
    
            let ctrl_input;
            if use_subgoal_ctrl {
                let start_time = Instant::now();
                let (safe, subgoal, _) = 
                if use_rtreach {
                    select_safe_subgoal_rtreach(&mut quad_model, state, start_waypoint, goal_waypoint, num_subgoal_cands, sim_time, step_size, wall_time_ms, start_ms, store_rect, fixed_step, use_rtreach_dynamic_control, false)
                }
                else{
                    select_safe_subgoal_circle(&state, start_waypoint, goal_waypoint, num_subgoal_cands*10, false)
                };
                let duration = start_time.elapsed().as_micros() as f64;

                subgoal_compute_time.push(duration);
                if duration > wall_time_ms as f64 * 1000.0 {
                    deadline_violation_count += 1.0;
                }
                
                if !safe {
                    no_subgoal = true;
                }
                quad_model.set_goal(subgoal);
                ctrl_input = quad_model.sample_state_action(&state).to_vec();
            }
            else {
                ctrl_input = quad_model.sample_state_action(&state).to_vec();
            }
    
            let mut next_state = simulate_quadcopter(&quad_model, state, &ctrl_input, euler_step_size, step_size);
            
            next_state[3] = normalize_angle(next_state[3]);
            time += step_size;
            state = next_state;
            if has_collided(&state) {
                collision = true;
            }
            step += 1;
        }

        if collision || no_subgoal {
            if collision {
                collisions.push(1.0);
            }
            else {
                collisions.push(0.0);
            }
            if no_subgoal {
                no_subgoal_ctrl.push(1.0);
            }
            else {
                no_subgoal_ctrl.push(0.0);
            }
            time_vec.push(-1.0);
        }   
        else {
            collisions.push(0.0);
            no_subgoal_ctrl.push(0.0);
            time_vec.push(time);
        }

        if use_subgoal_ctrl {
            avg_subgoal_computation_time.push(subgoal_compute_time.iter().sum::<f64>() / subgoal_compute_time.len() as f64);
            max_subgoal_computation_time.push(*subgoal_compute_time.iter().max_by(|x, y| x.partial_cmp(y).unwrap()).unwrap());
        }
        else{
            avg_subgoal_computation_time.push(0.0);
            max_subgoal_computation_time.push(0.0);
        }

        deadline_violations.push(deadline_violation_count);
    }

    let valid_time_sum: f64 = time_vec.iter().filter(|&&x| x != -1.0 && x < 19.9).sum();
    let valid_time_count = time_vec.iter().filter(|&&x| x != -1.0 && x < 19.9).count();
    let avg_time = valid_time_sum / valid_time_count as f64;
    let timeouts = time_vec.iter().filter(|&&x| x >= 19.9).count();
    let total_collisions = collisions.iter().sum::<f64>();
    let total_no_subgoal_ctrl = no_subgoal_ctrl.iter().sum::<f64>();
    let avg_subgoal_time = avg_subgoal_computation_time.iter().sum::<f64>() / avg_subgoal_computation_time.len() as f64;
    let max_subgoal_time = *max_subgoal_computation_time.iter().max_by(|x, y| x.partial_cmp(y).unwrap()).unwrap();
    let total_deadline_violations = deadline_violations.iter().sum::<f64>();

    println!("Average time to reach goal (TTG) without safety violation and timeouts: {}s", avg_time);
    println!("Timeouts: {}", timeouts);
    println!("Total collisions: {}", total_collisions);
    println!("Total no subgoal ctrl: {}", total_no_subgoal_ctrl);
    println!("Average subgoal computation time: {}us", avg_subgoal_time);
    println!("Max subgoal computation time: {}us", max_subgoal_time);
    println!("Total deadline violations: {}", total_deadline_violations);

    if save_data{
        let mut wtr = csv::Writer::from_path(eval_output_path)?;
        wtr.write_record(&["TTG", "Collision", "No Subgoal", "Avg Subgoal Compute Time", "Max Subgoal Compute Time", "Deadline Violations"])?;
        for i in 0..time_vec.len() {
            wtr.write_record(&[time_vec[i].to_string(), collisions[i].to_string(), no_subgoal_ctrl[i].to_string(), avg_subgoal_computation_time[i].to_string(), max_subgoal_computation_time[i].to_string(), deadline_violations[i].to_string()])?;
        }
        wtr.flush()?;
    }

    Ok(())
}

