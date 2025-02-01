use std::{env, vec};
use std::time::Instant;
use std::fs;

use csv::ReaderBuilder;
use tract_onnx::prelude::*;
use pbr::ProgressBar;

use rtreach::obstacle_safety::{allocate_obstacles, OBSTACLES, DYNAMIC_OBSTACLE_COUNT};

use quadcopter::simulate_quadcopter::simulate_quadcopter;
use quadcopter::quadcopter_model::has_collided;
use quadcopter::dynamics_quadcopter::{QuadcopterModel, QUAD_NUM_DIMS as NUM_DIMS};
use quadcopter::utils::{distance, normalize_angle};
use quadcopter::controller::{select_safe_subgoal_rtreach, select_safe_subgoal_circle, model_sample_action};

const CORR_DATASET_PATH: &str = "eval_input_data/quadcopter/corr_dataset.csv";
const EVAL_OUTPUT_PARENT: &str = "eval_output_data/quadcopter/corr_exp/";
const OBSTACLE_SPEED: f64 = 0.5; // m/s

fn main() -> TractResult<()> {
    let args: Vec<String> = env::args().collect();

    // Print all the arguments
    println!("Arguments: {:?}", args);

    // Access specific arguments
    let save_data: i32;
    let algorithm: &str;
    let obstacle_type: &str;
    if args.len() == 4 {
        algorithm = &args[1];
        obstacle_type = &args[2];
        save_data = args[3].parse().expect("Second argument must be an integer");
        println!("Algorithm: {}", algorithm);
        println!("Save Data: {}", save_data);
    }
    else {
        eprintln!("Error: Not enough arguments provided.");
        eprintln!("Usage: {} <algorithm> <obstacle_type> <save_data>", args[0]);
        std::process::exit(1); // Exit with a non-zero status code
    }

    // Set algorithm parameters
    // [learning_enabled, use_subgoal_ctrl, use_rtreach, use_rtreach_dynamic_control]
    let algorithm_parameters: Vec<bool>;
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

    // Set obstacle type
    let obstacle_sim_fn: fn(f64, &mut Vec<Vec<Vec<f64>>>);
    if obstacle_type == "static" {
        obstacle_sim_fn = obstacle_sim_fn_static;
    }
    else if obstacle_type == "dynamic" {
        obstacle_sim_fn = obstacle_sim_fn_dynamic;
        let mut dyn_obs_count = DYNAMIC_OBSTACLE_COUNT.lock().unwrap();
        *dyn_obs_count = 2;
    }
    else {
        eprintln!("Error: Invalid obstacle type provided.");
        eprintln!("Obstacle type must be one of the following: static, dynamic");
        std::process::exit(1); // Exit with a non-zero status code
    }

    // Get the current working directory
    let current_dir = env::current_dir().expect("Failed to get current directory");

    let line_dataset_path = current_dir.join(CORR_DATASET_PATH);
    let eval_output_parent = current_dir.join(EVAL_OUTPUT_PARENT);
    let eval_output_path = eval_output_parent.join(format!("{}_{}_corr_exp.csv", algorithm, obstacle_type));

    if save_data == 1 {
        print!("Saving data to: {}", eval_output_path.to_str().unwrap());
        fs::create_dir_all(eval_output_parent)?; // Creates parent directories if they don't exist
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
    let initial_obstacles: Vec<Vec<Vec<f64>>>;
    {
        let obstacles_lock = OBSTACLES.lock().unwrap();
        initial_obstacles = obstacles_lock.clone().unwrap();
    }
    
    // Start & Goal States
    let start_state = [0.0; NUM_DIMS];

    // Simulation Parameters
    let step_size = 0.1;  // seconds
    let euler_step_size = 0.0002;
    let total_steps = 200;
    let thresh = 0.2;

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
    let num_subgoal_cands = 5;

    quad_model.set_ctrl_fn(pi_low);
    if learning_enabled {
        quad_model.set_model(&model);
    }

    // let mut index = 0;
    let mut time_vec = vec![];
    let mut collisions = vec![];
    let mut no_subgoal_ctrl = vec![];
    let mut avg_subgoal_computation_time = vec![];
    let mut max_subgoal_computation_time = vec![];
    let mut deadline_violations = vec![];

    let mut pb = ProgressBar::new(1000);
    for row in rdr.records() {
        pb.inc();
        // index += 1;
        // if index == 100 {
        //     break;
        // }
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

        {
            let mut obstacles_lock = OBSTACLES.lock().unwrap();
            *obstacles_lock = Some(initial_obstacles.clone());
        }

        while !collision && !no_subgoal && step < total_steps && distance(&state, &goal_waypoint) > thresh {
    
            let ctrl_input;
            if use_subgoal_ctrl {
                let start_time = Instant::now();
                let (safe, subgoal, _) = 
                if use_rtreach {
                    select_safe_subgoal_rtreach(&mut quad_model, state, start_waypoint, goal_waypoint, num_subgoal_cands, sim_time, step_size, wall_time_ms, start_ms, store_rect, fixed_step, use_rtreach_dynamic_control, false, obstacle_sim_fn)
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
            
            {
                let mut obstacles_lock = OBSTACLES.lock().unwrap();
                if let Some(obstacles) = obstacles_lock.as_mut() {
                    obstacle_sim_fn(step_size, obstacles);
                }
            }

            next_state[3] = normalize_angle(next_state[3]); // phi
            next_state[4] = normalize_angle(next_state[4]); // theta
            next_state[5] = normalize_angle(next_state[5]); // psi
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

fn obstacle_sim_fn_static(_: f64, _: &mut Vec<Vec<Vec<f64>>>) {
    // Do nothing
}

fn obstacle_sim_fn_dynamic(t: f64, obstacles: &mut Vec<Vec<Vec<f64>>>) {
    let offset = OBSTACLE_SPEED * t;
    obstacles[0][1][0] -= offset;
    if obstacles[0][1][0] < -0.95 {
        obstacles[0][1][0] = -0.95;
    }
    obstacles[0][1][1] -= offset;
    if obstacles[0][1][1] < -0.45 {
        obstacles[0][1][1] = -0.45;
    }
    obstacles[1][1][0] += offset;
    if obstacles[1][1][0] > 0.45 {
        obstacles[1][1][0] = 0.45;
    }
    obstacles[1][1][1] += offset;
    if obstacles[1][1][1] > 0.95 {
        obstacles[1][1][1] = 0.95;
    }
}
