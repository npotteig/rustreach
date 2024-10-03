use std::f64::consts::PI;
use std::env;

use bicycle::simulate_bicycle::step_bicycle;
use bicycle::bicycle_model::has_collided;
use bicycle::dynamics_bicycle::{BicycleModel, BICYCLE_NUM_DIMS as NUM_DIMS};
use rtreach::obstacle_safety::allocate_obstacles;
use rtreach::util::{save_rects_to_csv, save_states_to_csv};


const STATES_FILE_PATH: &str = "data/ctrl_states_02_coll.csv";
fn main() {
    // Get the current working directory
    let current_dir = env::current_dir().expect("Failed to get current directory");

    let states_file_path = current_dir.join(STATES_FILE_PATH);

    let bicycle_model = BicycleModel;
    let c_h = -37.1967;
    let cm: f64 = 0.0342;
    let c_a = 1.9569;

    let num_obstacles: u32 = 2;
    let points: [[f64; 2]; 2] = [[2.,0.7], [2., -0.7]];
    allocate_obstacles(num_obstacles, &points);

    let mut start_state = [0.0; NUM_DIMS];
    let goal_list = [[4., 0.]];
    start_state[0] = 0.0;       // x
    start_state[1] = 0.0;       // y
    start_state[2] = 0.0;       // v
    start_state[3] = PI/2.0;       // theta

    let mut ctrl_input = [0.0; 2];
    ctrl_input[0] = 1.0;        // throttle
    ctrl_input[1] = 0.0;     // heading

    let step_size = 0.1;  // seconds
    let total_steps = 100;

    let mut states_vec: Vec<[f64; NUM_DIMS]> = Vec::new();
    let mut state = start_state.clone();
    states_vec.push(state);

    let mut time = 0.0;
    let mut step = 0;
    let mut collided = false;
    let thresh = 0.2;
    let k_p_theta = 1.0;
    let k_p_v = 1.0;
    let mut goal_idx = 0;

    while step < total_steps && distance(&state, &goal_list[goal_list.len() - 1]) > thresh {
        if distance(&state, &goal_list[goal_idx]) < thresh {
            goal_idx += 1;
        }
        
        let vx_des = goal_list[goal_idx][0] - state[0];
        let vy_des = goal_list[goal_idx][1] - state[1];
        println!("vx_des: {}, vy_des: {}", vx_des, vy_des);

        let (throttle, heading_input) = velocity_controller(&[vx_des, vy_des], &state, k_p_v, k_p_theta, c_h, cm, c_a);
        println!("heading_input: {}, throttle: {}", heading_input, throttle);
        let mut next_state = step_bicycle(&bicycle_model, &state, heading_input, throttle, step_size);
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
    save_states_to_csv(states_file_path.to_str().unwrap(), &states_vec);
    println!("The state after {} s is: \n [{},{},{},{}] \n", time-step_size,state[0],state[1],state[2],state[3]);

}


fn distance(pos1: &[f64], pos2: &[f64]) -> f64 {
    let dx = pos1[0] - pos2[0];
    let dy = pos1[1] - pos2[1];
    norm(&[dx, dy])
}

fn norm(vec: &[f64]) -> f64 {
    let mut sum = 0.0;
    for v in vec {
        sum += v*v;
    }
    sum.sqrt()
}

fn velocity_controller(v_des: &[f64], state: &[f64], k_p_v: f64,  k_p_theta: f64, c_h: f64, c_m: f64, c_a: f64) -> (f64, f64) {
    let cur_vx = state[2] * state[3].cos();
    let cur_vy = state[2] * state[3].sin();
    
    let e_vx = v_des[0] - cur_vx;
    let e_vy = v_des[1] - cur_vy;
    
    let theta_des = v_des[1].atan2(v_des[0]); 
    let e_theta = heading_error(state[3], theta_des);

    let e_longitudinal = (e_vx * state[3].cos() + e_vy * state[3].sin()).max(0.1);

    let mut throttle_input = (k_p_v * e_longitudinal + c_a * state[2]) / (c_a * c_m) + c_h;
    // penalize large heading errors
    if e_theta.abs() > PI / 2.0 {
        throttle_input *= 1.0 - e_theta.abs() / PI;
    }

    let heading_input = (k_p_theta * e_theta).min(PI / 4.0).max(-PI / 4.0);
    (throttle_input, heading_input)
}

// Function to normalize the angle between -π and π
fn normalize_angle(angle: f64) -> f64 {
    let mut normalized = angle;
    while normalized > PI {
        normalized -= 2.0 * PI;
    }
    while normalized < -PI {
        normalized += 2.0 * PI;
    }
    normalized
}

// Function to calculate the difference between headings
fn heading_error(current_heading: f64, goal_direction: f64) -> f64 {
    // Calculate the heading difference
    let heading_diff = goal_direction - current_heading;

    // Step 4: Normalize the difference to the range [-π, π]
    normalize_angle(heading_diff)
}