use tract_onnx::prelude::*;

use rtreach::geometry::HyperRectangle;
use rtreach::interval::{new_interval, new_interval_v};
use rtreach::obstacle_safety::check_safety_obstacles_circumscribe;

use super::quadcopter_model::run_reachability_quadcopter;
use super::dynamics_quadcopter::{QuadcopterModel, QUAD_NUM_DIMS as NUM_DIMS};
use super::utils::{distance, normalize_angle};

const G: f64 = 9.81;
const M: f64 = 1.2;
const I_X: f64 = 0.0123;
const I_Y: f64 = 0.0123;
const I_Z: f64 = 0.0224;

const K_P_U: f64 = 0.75;
const K_P_V: f64 = 0.75;
const K_P_W: f64 = 1.0;

const K_P_PHI: f64 = 8.0;
const K_P_THETA: f64 = 8.0;
const K_P_PSI: f64 = 1.5;

const K_P_P: f64 = 1.5;
const K_P_Q: f64 = 1.5;
const K_P_R: f64 = 1.0;

pub fn goal_conditioned_sample_action(state: &[f64; NUM_DIMS], goal: &[f64; 3]) -> [f64; 4] {
    let vx_des = goal[0] - state[0];
    let vy_des = goal[1] - state[1];
    xy_vel_z_pos_controller(vx_des, vy_des, goal[2], true, state)
}

pub fn model_sample_action(state: &[f64; NUM_DIMS], goal: &[f64; 3], model: Option<&SimplePlan<TypedFact, Box<dyn TypedOp>, Graph<TypedFact, Box<dyn TypedOp>>>>) -> [f64; 4] {
    match model {
        Some(actor) => {
            let input = tract_ndarray::Array::from_shape_vec((1, 12), vec![goal[0] - state[0], 
                                                                                                                goal[1] - state[1], 
                                                                                                                goal[2] - state[2], 
                                                                                                                state[3],
                                                                                                                state[4],
                                                                                                                state[5],
                                                                                                                state[6],
                                                                                                                state[7],
                                                                                                                state[8],
                                                                                                                state[9],
                                                                                                                state[10],
                                                                                                                state[11]]).unwrap();

            let result = actor.run(tvec!(input.into_tensor().into())).unwrap();
            let output = result[0].to_array_view::<f32>().unwrap();
            let model_output = output.iter().collect::<Vec<_>>();

            if model_output.len() == 2 {
                let v_des =  [*model_output[0] as f64 * 5.0, *model_output[1] as f64 * 5.0];
                return xy_vel_z_pos_controller(v_des[0], v_des[1], goal[2], true, state)
            }
            return [0.0, 0.0, 0.0, 0.0];
        }
        None => {
            return goal_conditioned_sample_action(state, goal);
        }
    }
}

pub fn xy_vel_z_pos_controller(
    x_dot_des: f64,
    y_dot_des: f64,
    z_des: f64,
    forward_looking: bool,
    state: &[f64],
) -> [f64; 4]{
    assert !(state.len() == 12, "State vector must have 12 dimensions");

    let z = state[2];
    let phi = state[3];
    let theta = state[4];
    let psi = state[5];
    let u = state[6];
    let v = state[7];
    let w = state[8];
    let p = state[9];
    let q = state[10];
    let r = state[11];

    let u_des = x_dot_des;
    let v_des = y_dot_des;
    let w_des = z_des - z;

    let theta_des = - K_P_U * (u_des - u) / G;
    let phi_des = K_P_V * (v_des - v) / G;
    let psi_des = y_dot_des.atan2(x_dot_des);

    let p_des = K_P_PHI * (phi_des - phi);
    let q_des = K_P_THETA * (theta_des - theta);
    let r_des = if forward_looking {K_P_PSI * normalize_angle(psi_des - psi)} else {0.0};

    let f_t = - M * K_P_W * (w_des - w);
    let tor_x = I_X * K_P_P * (p_des - p);
    let tor_y = I_Y * K_P_Q * (q_des - q);
    let tor_z = I_Z * K_P_R * (r_des - r);

    [f_t, tor_x, tor_y, tor_z]
}

// Function to generate subgoal candiates evenly spaced along the path
// Inputs: start and goal points, number of subgoals to generate
// Output: Vector of subgoal candidates
fn generate_linear_subgoals_simple(start: &[f64; 3], goal: &[f64; 3], num_subgoals: u32) -> Vec<[f64; 3]> {
    let mut subgoals = Vec::new();
    let dx = (goal[0] - start[0]) / num_subgoals as f64;
    let dy = (goal[1] - start[1]) / num_subgoals as f64;
    let dz = (goal[2] - start[2]) / num_subgoals as f64;
    for i in 1..(num_subgoals+1) {
        subgoals.push([start[0] + i as f64 * dx, start[1] + i as f64 * dy, start[2] + i as f64 * dz]);
    }
    subgoals
}

fn generate_linear_subgoals_sliding(
    start: &[f64; 3],
    goal: &[f64; 3],
    robot_position: &[f64; 3],
    num_subgoals: u32,
    range_behind: f64,
    range_ahead: f64,
) -> Vec<[f64; 3]> {
    let mut subgoals = Vec::new();
    
    // Vector from start to goal
    let line_vec = [goal[0] - start[0], goal[1] - start[1]];
    let line_length = (line_vec[0].powi(2) + line_vec[1].powi(2)).sqrt();
    let unit_line_vec = [line_vec[0] / line_length, line_vec[1] / line_length];

    // Vector from start to robot position
    let to_robot_vec = [robot_position[0] - start[0], robot_position[1] - start[1]];
    let projection_length = to_robot_vec[0] * unit_line_vec[0] + to_robot_vec[1] * unit_line_vec[1];
    
    // Projected point on the line
    let projected_point = [
        start[0] + projection_length * unit_line_vec[0],
        start[1] + projection_length * unit_line_vec[1],
    ];

    // Define the segment start and end around the projected point
    let mut segment_start = [
        projected_point[0] - range_behind * unit_line_vec[0],
        projected_point[1] - range_behind * unit_line_vec[1],
    ];
    let mut segment_end = [
        projected_point[0] + range_ahead * unit_line_vec[0],
        projected_point[1] + range_ahead * unit_line_vec[1],
    ];

    // Trim segment start if it is behind the `start` point
    if (segment_start[0] - start[0]) * unit_line_vec[0] + (segment_start[1] - start[1]) * unit_line_vec[1] < 0.0 {
        segment_start = [start[0], start[1]];
    }

    // Trim segment end if it is ahead of the `goal` point
    if (segment_end[0] - goal[0]) * unit_line_vec[0] + (segment_end[1] - goal[1]) * unit_line_vec[1] > 0.0 {
        segment_end = [goal[0], goal[1]];
    }

    // Divide the segment into `num_subgoals` evenly spaced points
    let dx = (segment_end[0] - segment_start[0]) / num_subgoals as f64;
    let dy = (segment_end[1] - segment_start[1]) / num_subgoals as f64;

    for i in 0..=num_subgoals {
        let subgoal = [
            segment_start[0] + i as f64 * dx,
            segment_start[1] + i as f64 * dy,
            0.0,
        ];
        subgoals.push(subgoal);
    }
    
    subgoals
}

pub fn select_safe_subgoal_circle(
    state: &[f64; NUM_DIMS],
    start: [f64; 3], 
    goal: [f64; 3],
    num_subgoal_cands: u32,
    sliding_window: bool,
)-> (bool, [f64; 3], Vec<HyperRectangle<NUM_DIMS>>){
    let mut subgoals = 
    if sliding_window{
        generate_linear_subgoals_sliding(&start, &goal, &[state[0], state[1], state[2]], num_subgoal_cands, 1.0, 5.0)
    } else {
        generate_linear_subgoals_simple(&start, &goal, num_subgoal_cands)
    };
    subgoals.reverse(); // Reverse the order to prioritize subgoals closer to the goal
    for subgoal in subgoals.iter() {
        let rad_des = distance(state, subgoal);
        let subgoal_rect = HyperRectangle::<NUM_DIMS> {
            dims: [
                new_interval(subgoal[0] - rad_des, subgoal[0] + rad_des),
                new_interval(subgoal[1] - rad_des, subgoal[1] + rad_des),
                new_interval(subgoal[2] - rad_des, subgoal[2] + rad_des),
                new_interval_v(0.0),
                new_interval_v(0.0),
                new_interval_v(0.0),
                new_interval_v(0.0),
                new_interval_v(0.0),
                new_interval_v(0.0),
                new_interval_v(0.0),
                new_interval_v(0.0),
                new_interval_v(0.0),
            ],
        };
        if check_safety_obstacles_circumscribe(subgoal, 0.16, rad_des){
            return (true, *subgoal, vec![subgoal_rect]);
        }
    }
    (false, [0.0, 0.0, 0.0], Vec::new())
}

// Given a list of control inputs in priority order and current state,
// Return the first control input index rtreach determined to be safe
// If none are determined to be safe boolean is false
fn select_safe_control(
    system_model: &mut QuadcopterModel, 
    start_state: [f64; NUM_DIMS], 
    sim_time: f64,
    init_step_size: f64, 
    wall_time_ms: u64, 
    start_ms: u64, 
    subgoals: &Vec<[f64; 3]>,
    control_inputs: &Vec<[f64; 4]>, 
    store_rect: bool,
    fixed_step: bool,
    rtreach_dynamic_control: bool,
) -> (bool, usize, Vec<HyperRectangle<NUM_DIMS>>) {
    let wall_time_per_input = wall_time_ms / control_inputs.len() as u64;
    for (idx, control_input) in control_inputs.iter().enumerate() {
        system_model.set_goal(subgoals[idx]);
        let (safe, storage_vec) = run_reachability_quadcopter(&system_model, 
                                                                                        start_state, 
                                                                                        sim_time,
                                                                                        init_step_size, 
                                                                                        wall_time_per_input, 
                                                                                        start_ms, 
                                                                                        &control_input.to_vec(), 
                                                                                        store_rect, 
                                                                                        fixed_step,
                                                                                        rtreach_dynamic_control);
        if safe {
            return (true, idx, storage_vec);
        }
    }
    return (false, 0, Vec::new());
}

// Function to select subgoal based on if its associated control input is safe
// Output none if no safe subgoal is found
pub fn select_safe_subgoal_rtreach(
    system_model: &mut QuadcopterModel, 
    state: [f64; NUM_DIMS],
    start: [f64; 3], 
    goal: [f64; 3],
    num_subgoal_cands: u32,
    sim_time: f64,
    init_step_size: f64, 
    wall_time_ms: u64, 
    start_ms: u64,  
    store_rect: bool,
    fixed_step: bool,
    rtreach_dynamic_control: bool,
    sliding_window: bool,
) -> (bool, [f64; 3], Vec<HyperRectangle<NUM_DIMS>>) {
    let mut subgoals = 
    if sliding_window{
        generate_linear_subgoals_sliding(&start, &goal, &[state[0], state[1], state[2]], num_subgoal_cands, 1.0, 5.0)
    } else {
        generate_linear_subgoals_simple(&start, &goal, num_subgoal_cands)
    };
    subgoals.reverse(); // Reverse the order to prioritize subgoals closer to the goal
    let mut control_inputs = Vec::new();
    // Generate control input for each subgoal
    for subgoal in subgoals.iter() {
        system_model.set_goal(*subgoal);
        let ctrl_input = system_model.sample_state_action(&state);
        control_inputs.push(ctrl_input);
    }
    let (safe, idx, storage_vec) = select_safe_control(system_model, state, sim_time, init_step_size, wall_time_ms, start_ms, &subgoals, &control_inputs, store_rect, fixed_step, rtreach_dynamic_control);
    if safe {
        return (true, subgoals[idx], storage_vec);
    }
    (false, [0.0, 0.0, 0.0], Vec::new())
}