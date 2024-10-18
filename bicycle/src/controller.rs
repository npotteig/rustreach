
use std::f64::consts::PI;
use tract_onnx::prelude::*;

use rtreach::geometry::HyperRectangle;
use rtreach::interval::{new_interval, new_interval_v};
use rtreach::obstacle_safety::check_safety_obstacles_circumscribe;

use super::bicycle_model::run_reachability_bicycle;
use super::dynamics_bicycle::{BicycleModel, BICYCLE_NUM_DIMS as NUM_DIMS};
use super::utils::{heading_error, distance};

pub fn goal_conditioned_sample_action(state: &[f64; NUM_DIMS], goal: &[f64; 2]) -> [f64; 2] {
    let vx_des = goal[0] - state[0];
    let vy_des = goal[1] - state[1];
    velocity_controller(&[vx_des, vy_des], state)
}

pub fn model_sample_action(state: &[f64; NUM_DIMS], goal: &[f64; 2], model: Option<&SimplePlan<TypedFact, Box<dyn TypedOp>, Graph<TypedFact, Box<dyn TypedOp>>>>) -> [f64; 2] {
    match model {
        Some(actor) => {
            let input = tract_ndarray::Array::from_shape_vec((1, 4), vec![goal[0] - state[0], goal[1] - state[1], state[2], state[3]]).unwrap();

            let result = actor.run(tvec!(input.into_tensor().into())).unwrap();
            let output = result[0].to_array_view::<f32>().unwrap();
            let model_output = output.iter().collect::<Vec<_>>();

            if model_output.len() == 2 {
                let v_des =  [*model_output[0] as f64 * 5.0, *model_output[1] as f64 * 5.0];
                return velocity_controller(&v_des, state)
            }
            return [0.0, 0.0];
        }
        None => {
            return goal_conditioned_sample_action(state, goal);
        }
    }
}

pub fn velocity_controller(v_des: &[f64], state: &[f64]) -> [f64; 2] {
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
    [heading_input, throttle_input]
}

pub fn select_safe_subgoal_circle(
    state: &[f64; NUM_DIMS],
    start: [f64; 2], 
    goal: [f64; 2],
    num_subgoal_cands: u32,
)-> (bool, [f64; 2], Vec<HyperRectangle<NUM_DIMS>>){
    // let robot_rad = (0.25f64.powf(2.0) + 0.15f64.powf(2.0)).sqrt();
    let robot_rad = 0.1;
    let mut subgoals = generate_linear_subgoals(&start, &goal, num_subgoal_cands);
    subgoals.reverse(); // Reverse the order to prioritize subgoals closer to the goal
    for subgoal in subgoals.iter() {
        let rad_des = distance(state, subgoal);
        let subgoal_rect = HyperRectangle::<NUM_DIMS> {
            dims: [
                new_interval(subgoal[0] - rad_des, subgoal[0] + rad_des),
                new_interval(subgoal[1] - rad_des, subgoal[1] + rad_des),
                new_interval_v(0.0),
                new_interval_v(0.0),
            ],
        };
        if check_safety_obstacles_circumscribe(subgoal, robot_rad, rad_des){
            return (true, *subgoal, vec![subgoal_rect]);
        }
    }
    (false, [0.0, 0.0], Vec::new())
}
// Function to select subgoal based on if its associated control input is safe
// Output none if no safe subgoal is found
pub fn select_safe_subgoal_rtreach(
    system_model: &mut BicycleModel, 
    state: [f64; NUM_DIMS],
    start: [f64; 2], 
    goal: [f64; 2],
    num_subgoal_cands: u32,
    sim_time: f64,
    init_step_size: f64, 
    wall_time_ms: u64, 
    start_ms: u64,  
    store_rect: bool,
    fixed_step: bool,
    rtreach_dynamic_control: bool,
) -> (bool, [f64; 2], Vec<HyperRectangle<NUM_DIMS>>) {
    let mut subgoals = generate_linear_subgoals(&start, &goal, num_subgoal_cands);
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
    (false, [0.0, 0.0], Vec::new())
}

// Given a list of control inputs in priority order and current state,
// Return the first control input index rtreach determined to be safe
// If none are determined to be safe boolean is false
fn select_safe_control(
    system_model: &mut BicycleModel, 
    start_state: [f64; NUM_DIMS], 
    sim_time: f64,
    init_step_size: f64, 
    wall_time_ms: u64, 
    start_ms: u64, 
    subgoals: &Vec<[f64; 2]>,
    control_inputs: &Vec<[f64; 2]>, 
    store_rect: bool,
    fixed_step: bool,
    rtreach_dynamic_control: bool,
) -> (bool, usize, Vec<HyperRectangle<NUM_DIMS>>) {
    let wall_time_per_input = wall_time_ms / control_inputs.len() as u64;
    for (idx, control_input) in control_inputs.iter().enumerate() {
        system_model.set_goal(subgoals[idx]);
        let (safe, storage_vec) = run_reachability_bicycle(&system_model, 
                                                                                        start_state, 
                                                                                        sim_time,
                                                                                        init_step_size, 
                                                                                        wall_time_per_input, 
                                                                                        start_ms, 
                                                                                        control_input[0], 
                                                                                        control_input[1], 
                                                                                        store_rect, 
                                                                                        fixed_step,
                                                                                        rtreach_dynamic_control);
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