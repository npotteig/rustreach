
use std::f64::consts::PI;

use rtreach::geometry::HyperRectangle;

use super::bicycle_model::run_reachability_bicycle;
use super::dynamics_bicycle::{BicycleModel, BICYCLE_NUM_DIMS as NUM_DIMS};
use super::utils::heading_error;

pub trait GoalConditionedController<const NUM_DIMS: usize, const CTRL_DIMS: usize, const GOAL_DIM: usize> {
    fn sample_action(&self, state: &[f64; NUM_DIMS], goal: &[f64; GOAL_DIM]) -> [f64; CTRL_DIMS];
}

pub struct SimpleGoalController;

impl GoalConditionedController<NUM_DIMS, 2, 2> for SimpleGoalController {
    fn sample_action(&self, state: &[f64; NUM_DIMS], goal: &[f64; 2]) -> [f64; 2] {
        let vx_des = goal[0] - state[0];
        let vy_des = goal[1] - state[1];
        velocity_controller(&[vx_des, vy_des], state)
    }
}

fn velocity_controller(v_des: &[f64], state: &[f64]) -> [f64; 2] {
    let c_h = -37.1967;
    let c_m: f64 = 0.0342;
    let c_a = 1.9569;
    
    let k_p_theta = 1.0;
    let k_p_v = 1.0;
    
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
    [throttle_input, heading_input]
}

// Function to select subgoal based on if its associated control input is safe
// Output none if no safe subgoal is found
pub fn select_safe_subgoal<
                        T1: GoalConditionedController<NUM_DIMS, 2, 2>
                        >(
    controller: &T1,
    system_model: &BicycleModel, 
    state: [f64; NUM_DIMS],
    start: [f64; 2], 
    goal: [f64; 2],
    num_subgoal_cands: u32,
    sim_time: f64,
    init_step_size: f64, 
    wall_time_ms: u64, 
    start_ms: u64,  
    store_rect: bool,
    fixed_step: bool
) -> (bool, [f64; 2], Vec<HyperRectangle<NUM_DIMS>>) {
    let mut subgoals = generate_linear_subgoals(&start, &goal, num_subgoal_cands);
    subgoals.reverse(); // Reverse the order to prioritize subgoals closer to the goal
    let mut control_inputs = Vec::new();
    // Generate control input for each subgoal
    for subgoal in subgoals.iter() {
        let ctrl_input = controller.sample_action(&state, &subgoal);
        control_inputs.push(ctrl_input);
    }
    let (safe, idx, storage_vec) = select_safe_control(system_model, state, sim_time, init_step_size, wall_time_ms, start_ms, &control_inputs, store_rect, fixed_step);
    if safe {
        return (true, subgoals[idx], storage_vec);
    }
    return (false, [0.0, 0.0], Vec::new());

}

// Given a list of control inputs in priority order and current state,
// Return the first control input index rtreach determined to be safe
// If none are determined to be safe boolean is false
fn select_safe_control(
    system_model: &BicycleModel, 
    start_state: [f64; NUM_DIMS], 
    sim_time: f64,
    init_step_size: f64, 
    wall_time_ms: u64, 
    start_ms: u64, 
    control_inputs: &Vec<[f64; 2]>, 
    store_rect: bool,
    fixed_step: bool
) -> (bool, usize, Vec<HyperRectangle<NUM_DIMS>>) {
    let wall_time_per_input = wall_time_ms / control_inputs.len() as u64;
    for (idx, control_input) in control_inputs.iter().enumerate() {
        let (safe, storage_vec) = run_reachability_bicycle(&system_model, 
                                                                                        start_state, 
                                                                                        sim_time,
                                                                                        init_step_size, 
                                                                                        wall_time_per_input, 
                                                                                        start_ms, 
                                                                                        control_input[1], 
                                                                                        control_input[0], 
                                                                                        store_rect, 
                                                                                        fixed_step);
        if safe {
            return (true, idx, storage_vec);
        }
    }
    return (false, 0, Vec::new());
}

// Function to generate subgoal candiates evenly spaced along the path
// Inputs: start and goal points, number of subgoals to generate
// Output: Vector of subgoal candidates
fn generate_linear_subgoals(start: &[f64; 2], goal: &[f64; 2], num_subgoals: u32) -> Vec<[f64; 2]> {
    let mut subgoals = Vec::new();
    let dx = (goal[0] - start[0]) / num_subgoals as f64;
    let dy = (goal[1] - start[1]) / num_subgoals as f64;
    for i in 1..(num_subgoals+1) {
        subgoals.push([start[0] + i as f64 * dx, start[1] + i as f64 * dy]);
    }
    subgoals
}