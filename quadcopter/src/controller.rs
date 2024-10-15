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

pub trait GoalConditionedController<const NUM_DIMS: usize, const CTRL_DIMS: usize, const GOAL_DIM: usize> {
    fn sample_action(&self, state: &[f64; NUM_DIMS], goal: &[f64; GOAL_DIM]) -> [f64; CTRL_DIMS];
}

pub struct SimpleGoalController;

impl GoalConditionedController<NUM_DIMS, 4, 3> for SimpleGoalController {
    fn sample_action(&self, state: &[f64; NUM_DIMS], goal: &[f64; 3]) -> [f64; 4] {
        let vx_des = goal[0] - state[0];
        let vy_des = goal[1] - state[1];
        xy_vel_z_pos_controller(vx_des, vy_des, goal[2], true, state)
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
fn generate_linear_subgoals(start: &[f64; 3], goal: &[f64; 3], num_subgoals: u32) -> Vec<[f64; 3]> {
    let mut subgoals = Vec::new();
    let dx = (goal[0] - start[0]) / num_subgoals as f64;
    let dy = (goal[1] - start[1]) / num_subgoals as f64;
    let dz = (goal[2] - start[2]) / num_subgoals as f64;
    for i in 1..(num_subgoals+1) {
        subgoals.push([start[0] + i as f64 * dx, start[1] + i as f64 * dy, start[2] + i as f64 * dz]);
    }
    subgoals
}

pub fn select_safe_subgoal_circle(
    state: &[f64; NUM_DIMS],
    start: [f64; 3], 
    goal: [f64; 3],
    num_subgoal_cands: u32,
)-> (bool, [f64; 3], Vec<HyperRectangle<NUM_DIMS>>){
    let mut subgoals = generate_linear_subgoals(&start, &goal, num_subgoal_cands);
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
    system_model: &QuadcopterModel, 
    start_state: [f64; NUM_DIMS], 
    sim_time: f64,
    init_step_size: f64, 
    wall_time_ms: u64, 
    start_ms: u64, 
    control_inputs: &Vec<[f64; 4]>, 
    store_rect: bool,
    fixed_step: bool
) -> (bool, usize, Vec<HyperRectangle<NUM_DIMS>>) {
    let wall_time_per_input = wall_time_ms / control_inputs.len() as u64;
    for (idx, control_input) in control_inputs.iter().enumerate() {
        let (safe, storage_vec) = run_reachability_quadcopter(&system_model, 
                                                                                        start_state, 
                                                                                        sim_time,
                                                                                        init_step_size, 
                                                                                        wall_time_per_input, 
                                                                                        start_ms, 
                                                                                        &control_input.to_vec(), 
                                                                                        store_rect, 
                                                                                        fixed_step);
        if safe {
            return (true, idx, storage_vec);
        }
    }
    return (false, 0, Vec::new());
}

// Function to select subgoal based on if its associated control input is safe
// Output none if no safe subgoal is found
pub fn select_safe_subgoal_rtreach<
                        T1: GoalConditionedController<NUM_DIMS, 4, 3>
                        >(
    controller: &T1,
    system_model: &QuadcopterModel, 
    state: [f64; NUM_DIMS],
    start: [f64; 3], 
    goal: [f64; 3],
    num_subgoal_cands: u32,
    sim_time: f64,
    init_step_size: f64, 
    wall_time_ms: u64, 
    start_ms: u64,  
    store_rect: bool,
    fixed_step: bool
) -> (bool, [f64; 3], Vec<HyperRectangle<NUM_DIMS>>) {
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
    (false, [0.0, 0.0, 0.0], Vec::new())
}