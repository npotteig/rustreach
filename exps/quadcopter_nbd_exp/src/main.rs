use std::{env, vec};
use std::time::Instant;
use std::fs;
use std::sync::Mutex;

use tract_onnx::prelude::*;
use lazy_static::lazy_static;
use pbr::ProgressBar;

use rtreach::obstacle_safety::{load_obstacles_from_csv, allocate_obstacles, DYNAMIC_OBSTACLE_COUNT, OBSTACLES};
use rtreach::util::load_paths_from_csv;

use quadcopter::simulate_quadcopter::simulate_quadcopter;
use quadcopter::quadcopter_model::has_collided;
use quadcopter::dynamics_quadcopter::{QuadcopterModel, QUAD_NUM_DIMS as NUM_DIMS};
use quadcopter::utils::{distance, normalize_angle};
use quadcopter::controller::{select_safe_subgoal_rtreach, select_safe_subgoal_circle, model_sample_action};

const PATH_DATASET_PARENT: &str = "eval_input_data/";
const ASTAR_OBSTACLE_DATASET_PATH: &str = "eval_input_data/astar_rr_nbd_obstacles_near_path.csv";
const RRT_OBSTACLE_DATASET_PATH: &str = "eval_input_data/rrt_rr_nbd_obstacles_near_path.csv";
const EVAL_OUTPUT_PARENT: &str = "eval_output_data/quadcopter/nbd_exp/";
const OBSTACLE_SPEED: f64 = 0.5; // m/s

// Define global variables using lazy_static
lazy_static! {
    static ref PERP_SLOPE_DX: Mutex<f64> = Mutex::new(0.0);
    static ref PERP_SLOPE_DY: Mutex<f64> = Mutex::new(0.0);
}

fn main() -> TractResult<()> {
    let args: Vec<String> = env::args().collect();
    
    // Print all the arguments
    println!("Arguments: {:?}", args);

    // Access specific arguments
    let save_data: i32;
    let algorithm: &str;
    let waypt_algorithm: &str;
    let obstacle_type: &str;
    if args.len() == 5 {
        algorithm = &args[1];
        if args[2] != "astar" && args[2] != "rrt" {
            eprintln!("Error: Invalid waypoint algorithm provided.");
            eprintln!("Waypoint algorithm must be one of the following: astar, rrt");
            std::process::exit(1); // Exit with a non-zero status code
        }
        waypt_algorithm = &args[2];
        obstacle_type = &args[3];
        save_data = args[4].parse().expect("Second argument must be an integer");
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

    // Set obstacle type
    let obstacle_sim_fn: fn(f64, &mut Vec<Vec<Vec<f64>>>);
    let initial_points: Vec<[f64; 2]>;
    if obstacle_type == "static" {
        obstacle_sim_fn = obstacle_sim_fn_static;
        initial_points = vec![];
    }
    else if obstacle_type == "dynamic" {
        obstacle_sim_fn = obstacle_sim_fn_dynamic;
        initial_points = vec![[0.0, 0.0]];
        let mut dyn_obs_count = DYNAMIC_OBSTACLE_COUNT.lock().unwrap();
        *dyn_obs_count = 1;
    }
    else {
        eprintln!("Error: Invalid obstacle type provided.");
        eprintln!("Obstacle type must be one of the following: static, dynamic");
        std::process::exit(1); // Exit with a non-zero status code
    }


    // Get the current working directory
    let current_dir: std::path::PathBuf = env::current_dir().expect("Failed to get current directory");

    let path_dataset_parent = current_dir.join(PATH_DATASET_PARENT);
    let path_dataset_path = path_dataset_parent.join(format!("{}_rustreach_paths.csv", waypt_algorithm));
    let obstacle_dataset_path = if waypt_algorithm == "astar" {
        current_dir.join(ASTAR_OBSTACLE_DATASET_PATH)
    }
    else {
        current_dir.join(RRT_OBSTACLE_DATASET_PATH)
    };
    let eval_output_parent = current_dir.join(EVAL_OUTPUT_PARENT);
    let eval_output_path = eval_output_parent.join(format!("{}_{}_{}_nbd_exp.csv", algorithm, waypt_algorithm, obstacle_type));

    if save_data == 1 {
        print!("Saving data to: {}", eval_output_path.to_str().unwrap());
        fs::create_dir_all(eval_output_parent)?; // Creates parent directories if they don't exist
    }

    let paths_vec = load_paths_from_csv(&path_dataset_path);
    let obstacles_vec = load_obstacles_from_csv(&obstacle_dataset_path);

    // Load the ONNX model from file
    let model = tract_onnx::onnx()
        .model_for_path("models/quadcopter_model_actor.onnx")?
        // specify input type and shape
        .with_input_fact(0, f64::fact([1, 12]).into())?
        .into_optimized()?        // Optimize the model for performance
        .into_runnable()?;         // Make it runnable

    let mut quadcopter_model = QuadcopterModel::default();

    // Start & Goal States
    let start_state = [0.0; NUM_DIMS];

    // Simulation Parameters
    let step_size = 0.1;  // seconds
    let euler_step_size = 0.0002;
    let total_steps = 1000;
    let thresh = 1.0;

    // Control Parameters
    let learning_enabled = algorithm_parameters[0];
    let use_subgoal_ctrl = algorithm_parameters[1];
    let use_rtreach = algorithm_parameters[2];
    let use_rtreach_dynamic_control = algorithm_parameters[3];
    let pi_low = model_sample_action;
    let sim_time = 1.0;
    let wall_time_ms = 100;
    let start_ms = 0;
    let store_rect = false;
    let fixed_step = false;
    let num_subgoal_cands = 5;

    quadcopter_model.set_ctrl_fn(pi_low);
    if learning_enabled {
        quadcopter_model.set_model(&model);
    }

    let mut time_vec = vec![];
    let mut collisions = vec![];
    let mut no_subgoal_ctrl = vec![];
    let mut avg_subgoal_computation_time = vec![];
    let mut max_subgoal_computation_time = vec![];
    let mut deadline_violations = vec![];

    let mut pb = ProgressBar::new(1000);
    for (i, pth) in paths_vec.iter().enumerate(){
        pb.inc();
        let mut state = start_state.clone();
        state[0] = pth[0][0];
        state[1] = pth[0][1];
        state[2] = 0.0;
        let mut time = 0.0;
        let mut step = 0;
        let mut collision = false;
        let mut no_subgoal = false;
        let mut goal_idx = 1;
        let mut prev_goal_waypoint = [pth[0][0], pth[0][1], 0.0];
        let mut cur_goal_waypoint = [pth[goal_idx][0], pth[goal_idx][1], 0.0];
        let final_goal_waypoint = [pth[pth.len()-1][0], pth[pth.len()-1][1], 0.0];
        let mut subgoal_compute_time = vec![];
        let mut deadline_violation_count = 0.0;

        quadcopter_model.set_goal(cur_goal_waypoint);

        let mut obstacle_set = obstacles_vec[i].clone();
        for pt in initial_points.iter() {
            obstacle_set.insert(0, pt.clone());
        }
        allocate_obstacles(obstacle_set.len() as u32, &obstacle_set);

        {
            if obstacle_type == "dynamic" && distance(&prev_goal_waypoint, &cur_goal_waypoint) > 2.9 {
                let mut obstacles_lock = OBSTACLES.lock().unwrap();
                if let Some(obstacles) = obstacles_lock.as_mut() {
                    update_obstacle_pos(obstacles, &prev_goal_waypoint, &cur_goal_waypoint);
                }
            }
        }

        while !collision && !no_subgoal && step < total_steps && distance(&state, &final_goal_waypoint) > thresh {
            if cur_goal_waypoint != final_goal_waypoint && distance(&state, &cur_goal_waypoint) < thresh{
                goal_idx += 1;
                prev_goal_waypoint = cur_goal_waypoint.clone();
                cur_goal_waypoint = [pth[goal_idx][0], pth[goal_idx][1], 0.0];
                quadcopter_model.set_goal(cur_goal_waypoint);
                {
                    if obstacle_type == "dynamic" && distance(&prev_goal_waypoint, &cur_goal_waypoint) > 4.0 {
                        let mut obstacles_lock = OBSTACLES.lock().unwrap();
                        if let Some(obstacles) = obstacles_lock.as_mut() {
                            update_obstacle_pos(obstacles, &prev_goal_waypoint, &cur_goal_waypoint);
                        }
                    }
                }
            }

            let ctrl_input;
            if use_subgoal_ctrl {
                let start_time = Instant::now();
                let (safe, subgoal, _) = 
                if use_rtreach {
                    select_safe_subgoal_rtreach(&mut quadcopter_model, state, prev_goal_waypoint, cur_goal_waypoint, num_subgoal_cands, sim_time, step_size, wall_time_ms, start_ms, store_rect, fixed_step, use_rtreach_dynamic_control, true, obstacle_sim_fn)
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
                quadcopter_model.set_goal(subgoal);
                ctrl_input = quadcopter_model.sample_state_action(&state).to_vec();
            }
            else {
                ctrl_input = quadcopter_model.sample_state_action(&state).to_vec();
            }
    
            let mut next_state = simulate_quadcopter(&quadcopter_model, state, &ctrl_input, euler_step_size, step_size);
            
            {
                if obstacle_type == "dynamic" {
                    let mut obstacles_lock = OBSTACLES.lock().unwrap();
                    if let Some(obstacles) = obstacles_lock.as_mut() {
                        obstacle_sim_fn(step_size, obstacles);
                    }
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

fn obstacle_sim_fn_static(_: f64, _: &mut Vec<Vec<Vec<f64>>>) {
    // Do nothing
}

fn obstacle_sim_fn_dynamic(t: f64, obstacles: &mut Vec<Vec<Vec<f64>>>) {
    let offset = OBSTACLE_SPEED * t;
    let perp_slope_dx = *PERP_SLOPE_DX.lock().unwrap();
    let perp_slope_dy = *PERP_SLOPE_DY.lock().unwrap();
    obstacles[0][0][0] -= perp_slope_dx*offset;
    obstacles[0][0][1] -= perp_slope_dx*offset;
    obstacles[0][1][0] -= perp_slope_dy*offset;
    obstacles[0][1][1] -= perp_slope_dy*offset;
}

fn update_obstacle_pos(obstacles: &mut Vec<Vec<Vec<f64>>>, prev_goal_waypoint: &[f64; 3], cur_goal_waypoint: &[f64; 3]) {
    let w = 0.5;
    let h = 0.5;
    
    let mid_x = (prev_goal_waypoint[0] + cur_goal_waypoint[0]) / 2.0;
    let mid_y = (prev_goal_waypoint[1] + cur_goal_waypoint[1]) / 2.0;

    let new_x: f64;
    let new_y: f64;
    let offset = 2.0;
    let mut perp_slope_dx = PERP_SLOPE_DX.lock().unwrap();
    let mut perp_slope_dy = PERP_SLOPE_DY.lock().unwrap();
    if prev_goal_waypoint[0] == cur_goal_waypoint[0] {
        new_x = mid_x + offset;
        new_y = mid_y;
        *perp_slope_dx = 1.0;
        *perp_slope_dy = 0.0;
    }
    else if prev_goal_waypoint[1] == cur_goal_waypoint[1] {
        new_x = mid_x;
        new_y = mid_y + offset;
        *perp_slope_dx = 0.0;
        *perp_slope_dy = 1.0;
    }
    else {
        // General case: Compute perpendicular offset using a normal vector
        let dx = cur_goal_waypoint[0] - prev_goal_waypoint[0];
        let dy = cur_goal_waypoint[1] - prev_goal_waypoint[1];
        let length = (dx * dx + dy * dy).sqrt();
        
        // Perpendicular unit vector (-dy/length, dx/length)
        let nx = -dy / length;
        let ny = dx / length;
        *perp_slope_dx = nx;
        *perp_slope_dy = ny;
        
        // Apply offset in the normal direction
        new_x = mid_x + offset * nx;
        new_y = mid_y + offset * ny;
    }

    obstacles[0][0][0] = new_x - w/2.0;
    obstacles[0][0][1] = new_x + w/2.0;
    obstacles[0][1][0] = new_y - h/2.0;
    obstacles[0][1][1] = new_y + h/2.0;
}
