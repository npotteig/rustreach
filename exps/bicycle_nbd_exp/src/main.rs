use std::{env, vec};
use std::time::Instant;
use std::fs;

use tract_onnx::prelude::*;

use rtreach::obstacle_safety::load_obstacles_from_csv;
use rtreach::util::load_paths_from_csv;

use bicycle::simulate_bicycle::step_bicycle;
use bicycle::bicycle_model::has_collided;
use bicycle::dynamics_bicycle::{BicycleModel, BICYCLE_NUM_DIMS as NUM_DIMS};
use bicycle::utils::{distance, normalize_angle};
use bicycle::controller::{select_safe_subgoal_rtreach, select_safe_subgoal_circle, model_sample_action};

const PATH_DATASET_PARENT: &str = "eval_input_data/";
const OBSTACLE_DATASET_PATH: &str = "eval_input_data/rr_nbd_obstacles.csv";
const EVAL_OUTPUT_PARENT: &str = "eval_output_data/bicycle/nbd_exp/";

fn main() -> TractResult<()> {
    let args: Vec<String> = env::args().collect();
    
    // Print all the arguments
    println!("Arguments: {:?}", args);

    // Access specific arguments
    let save_data: i32;
    let algorithm: &str;
    let waypt_algorithm: &str;
    if args.len() == 4 {
        algorithm = &args[1];
        if args[2] != "astar" && args[2] != "rrt" {
            eprintln!("Error: Invalid waypoint algorithm provided.");
            eprintln!("Waypoint algorithm must be one of the following: astar, rrt");
            std::process::exit(1); // Exit with a non-zero status code
        }
        waypt_algorithm = &args[2];
        save_data = args[3].parse().expect("Second argument must be an integer");
        println!("Algorithm: {}", algorithm);
        println!("Waypoint Algorithm: {}", waypt_algorithm);
        println!("Save Data: {}", save_data);
    }
    else {
        eprintln!("Error: Not enough arguments provided.");
        eprintln!("Usage: {} <algorithm> <waypt_algorithm> <save_data>", args[0]);
        std::process::exit(1); // Exit with a non-zero status code
    }

    // Set algorithm parameters
    // [learning_enabled, use_subgoal_ctrl, use_rtreach, use_rtreach_dynamic_control]
    let algorithm_parameters: Vec<bool> ;
    if algorithm == "wo"{
        algorithm_parameters = vec![true, false, false, false];
    }
    else if algorithm == "rrfc"{
        algorithm_parameters = vec![true, true, true, false];
    }
    else if algorithm == "rrrlc"{
        algorithm_parameters = vec![true, true, true, true];
    }
    else{
        eprintln!("Error: Invalid algorithm provided.");
        eprintln!("Algorithm must be one of the following: wo, rrfc, rrrlc");
        std::process::exit(1); // Exit with a non-zero status code
    }

    // Get the current working directory
    let current_dir: std::path::PathBuf = env::current_dir().expect("Failed to get current directory");
    
    let path_dataset_parent = current_dir.join(PATH_DATASET_PARENT);
    let path_dataset_path = path_dataset_parent.join(format!("{}_rustreach_paths.csv", waypt_algorithm));
    let obstacle_dataset_path = current_dir.join(OBSTACLE_DATASET_PATH);
    let eval_output_parent = current_dir.join(EVAL_OUTPUT_PARENT);
    let eval_output_path = eval_output_parent.join(format!("{}_nbd_exp.csv", algorithm));

    if save_data == 1 {
        print!("Saving data to: {}", eval_output_path.to_str().unwrap());
        fs::create_dir_all(eval_output_parent)?; // Creates parent directories if they don't exist
    }

    let paths_vec = load_paths_from_csv(&path_dataset_path);
    load_obstacles_from_csv(&obstacle_dataset_path);

    // Load the ONNX model from file
    let model = tract_onnx::onnx()
        .model_for_path("models/bicycle_model_actor.onnx")?
        // specify input type and shape
        .with_input_fact(0, f64::fact([1, 4]).into())?
        .into_optimized()?        // Optimize the model for performance
        .into_runnable()?;         // Make it runnable

    let mut bicycle_model = BicycleModel::default();

    // Start & Goal States
    let start_state = [0.0; NUM_DIMS];

    // Simulation Parameters
    let step_size = 0.1;  // seconds
    let total_steps = 1000;
    let thresh = 1.0;

    // Control Parameters
    let learning_enabled = algorithm_parameters[0];
    let use_subgoal_ctrl = algorithm_parameters[1];
    let use_rtreach = algorithm_parameters[2];
    let use_rtreach_dynamic_control = algorithm_parameters[3];
    let pi_low = model_sample_action;
    let sim_time = 2.0;
    let wall_time_ms = 100;
    let start_ms = 0;
    let store_rect = false;
    let fixed_step = false;
    let num_subgoal_cands = 10;

    bicycle_model.set_ctrl_fn(pi_low);
    if learning_enabled {
        bicycle_model.set_model(&model);
    }

    let mut index = 0;
    let mut time_vec = vec![];
    let mut collisions = vec![];
    let mut no_subgoal_ctrl = vec![];
    let mut avg_subgoal_computation_time = vec![];
    let mut max_subgoal_computation_time = vec![];
    let mut deadline_violations = vec![];

    for pth in paths_vec.iter(){
        println!("index: {}", index);
        index += 1;
        let mut state = start_state.clone();
        state[0] = pth[0][0];
        state[1] = pth[0][1];
        let mut time = 0.0;
        let mut step = 0;
        let mut collision = false;
        let mut no_subgoal = false;
        let mut goal_idx = 1;
        let mut prev_goal_waypoint = [pth[0][0], pth[0][1]];
        let mut cur_goal_waypoint = [pth[goal_idx][0], pth[goal_idx][1]];
        let final_goal_waypoint = [pth[pth.len()-1][0], pth[pth.len()-1][1]];
        let mut subgoal_compute_time = vec![];
        let mut deadline_violation_count = 0.0;

        bicycle_model.set_goal(cur_goal_waypoint);

        while !collision && !no_subgoal && step < total_steps && distance(&state, &final_goal_waypoint) > thresh {
            if cur_goal_waypoint != final_goal_waypoint && distance(&state, &cur_goal_waypoint) < thresh{
                goal_idx += 1;
                prev_goal_waypoint = cur_goal_waypoint.clone();
                cur_goal_waypoint = [pth[goal_idx][0], pth[goal_idx][1]];
                bicycle_model.set_goal(cur_goal_waypoint);
            }

            let ctrl_input;
            if use_subgoal_ctrl {
                let start_time = Instant::now();
                let (safe, subgoal, _) = 
                if use_rtreach {
                    select_safe_subgoal_rtreach(&mut bicycle_model, state, prev_goal_waypoint, cur_goal_waypoint, num_subgoal_cands, sim_time, step_size, wall_time_ms, start_ms, store_rect, fixed_step, use_rtreach_dynamic_control, true)
                }
                else{
                    select_safe_subgoal_circle(&state, prev_goal_waypoint, cur_goal_waypoint, num_subgoal_cands*10, true)
                };
                let duration = start_time.elapsed().as_micros() as f64;

                subgoal_compute_time.push(duration);
                if duration > wall_time_ms as f64 * 1000.0 {
                    deadline_violation_count += 1.0;
                }
                
                if !safe {
                    no_subgoal = true;
                }
                bicycle_model.set_goal(subgoal);
                ctrl_input = bicycle_model.sample_state_action(&state);
            }
            else {
                ctrl_input = bicycle_model.sample_state_action(&state);
            }
    
            let mut next_state = step_bicycle(&bicycle_model, &state, ctrl_input[0], ctrl_input[1], step_size);
            
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
            if subgoal_compute_time.len() > 0 {
                avg_subgoal_computation_time.push(subgoal_compute_time.iter().sum::<f64>() / subgoal_compute_time.len() as f64);
                max_subgoal_computation_time.push(*subgoal_compute_time.iter().max_by(|x, y| x.partial_cmp(y).unwrap()).unwrap());
            }
            else{
                avg_subgoal_computation_time.push(0.0);
                max_subgoal_computation_time.push(0.0);
            }
        }
        else{
            avg_subgoal_computation_time.push(0.0);
            max_subgoal_computation_time.push(0.0);
        }

        deadline_violations.push(deadline_violation_count);
    }

    let valid_time_sum: f64 = time_vec.iter().filter(|&&x| x != -1.0 && x < 99.9).sum();
    let valid_time_count = time_vec.iter().filter(|&&x| x != -1.0 && x < 99.9).count();
    let avg_time = valid_time_sum / valid_time_count as f64;
    let timeouts = time_vec.iter().filter(|&&x| x >= 99.9).count();
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

    if save_data == 1{
        let mut wtr = csv::Writer::from_path(eval_output_path)?;
        wtr.write_record(&["TTG", "Collision", "No Subgoal", "Avg Subgoal Compute Time", "Max Subgoal Compute Time", "Deadline Violations"])?;
        for i in 0..time_vec.len() {
            wtr.write_record(&[time_vec[i].to_string(), collisions[i].to_string(), no_subgoal_ctrl[i].to_string(), avg_subgoal_computation_time[i].to_string(), max_subgoal_computation_time[i].to_string(), deadline_violations[i].to_string()])?;
        }
        wtr.flush()?;
    }

    Ok(())
    
}
