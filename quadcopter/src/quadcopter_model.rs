use super::dynamics_quadcopter::{QuadcopterModel, QUAD_NUM_DIMS as NUM_DIMS};
use super::simulate_quadcopter::simulate_quadcopter_exp;
use rtreach::geometry::HyperRectangle;
use rtreach::obstacle_safety::{check_safety_obstacles, check_safety_wall};
use rtreach::face_lift::{LiftingSettings, face_lifting_iterative_improvement};
// a note from the quadcopter simulator 
// the arm length in x direction is 0.16 meters
// the arm length in y direction is 0.16 meters
// the motor height is 0.05 meters
// the total width of the quadcopter is 0.32 meters 

// function that stops simulation after two seconds
pub fn should_stop(state: [f64; NUM_DIMS], sim_time: f64, stop_time: &mut f64) -> bool {
    let mut rv = false;
    let max_time = 2.0;
    // stop if the maximum simulation time 
    if sim_time >= max_time {
        rv = true;
        *stop_time = -1.0;
    }

    rv
}

pub fn get_simulated_safe_time(system_model: &QuadcopterModel, start: [f64; NUM_DIMS], ctrl_input: &Vec<f64>, store_state: bool) -> (f64, Vec<[f64; NUM_DIMS]>) {
    let step_size: f64 = 0.0002;
    let mut rv: f64 = 0.0;
    let mut storage_vec: Vec<[f64; NUM_DIMS]> = Vec::new();
    simulate_quadcopter_exp(system_model, start, ctrl_input, step_size, should_stop, &mut rv, store_state, &mut storage_vec);

    (rv, storage_vec)
}

// called on states reached during the computation
pub fn intermediate_state(r: &mut HyperRectangle<NUM_DIMS>, store_rect: bool, storage_vec: &mut Vec<HyperRectangle<NUM_DIMS>>) -> bool {
    if store_rect {
        storage_vec.push(*r);
    }
    
    let mut allowed = true;
    let dxm = 0.16;

    // bloat the box for the width of the quadcopter
    r.dims[0].min = r.dims[0].min - dxm;
    r.dims[0].max = r.dims[0].max + dxm;
    r.dims[1].min = r.dims[1].min - dxm;
    r.dims[1].max = r.dims[1].max + dxm;

    allowed = check_safety_obstacles(r);

    if allowed {
        allowed = check_safety_wall(r);
    }

    // reset it
    r.dims[0].min = r.dims[0].min + dxm;
    r.dims[0].max = r.dims[0].max - dxm;
    r.dims[1].min = r.dims[1].min + dxm;
    r.dims[1].max = r.dims[1].max - dxm;

    allowed
}

// This function enumerates all of the corners of the current HyperRectangle and 
// returns whether or not any of the points lies outside of the ellipsoid
pub fn final_state(r: &mut HyperRectangle<NUM_DIMS>, store_rect: bool, storage_vec: &mut Vec<HyperRectangle<NUM_DIMS>>) -> bool {
    intermediate_state(r, store_rect, storage_vec)
}

// Clear all but the first rectangle (initial state) in the storage vector
pub fn restarted_computation(_: bool, storage_vec: &mut Vec<HyperRectangle<NUM_DIMS>>) {
    storage_vec.truncate(1);
}

pub fn has_collided(state: &[f64; NUM_DIMS]) -> bool {
    let mut rv = false;
    let mut r: HyperRectangle<NUM_DIMS> = HyperRectangle::default();
    for d in 0..NUM_DIMS {
        r.dims[d].min = state[d];
        r.dims[d].max = state[d];
    }
    let dxm = 0.16;
    r.dims[0].min = r.dims[0].min - dxm;
    r.dims[0].max = r.dims[0].max + dxm;
    r.dims[1].min = r.dims[1].min - dxm;
    r.dims[1].max = r.dims[1].max + dxm;

    let mut allowed = check_safety_obstacles(&r);

    if allowed {
        allowed = check_safety_wall(&r);
    }

    if !allowed {
        rv = true;
    }

    rv
}

pub fn run_reachability_quadcopter(system_model: &QuadcopterModel, 
    start: [f64; NUM_DIMS], 
    sim_time: f64,
    init_step_size: f64, 
    wall_time_ms: u64, 
    start_ms: u64, 
    ctrl_input: &Vec<f64>,
    store_rect: bool,
    fixed_step: bool,
    dynamic_control: bool) -> (bool, Vec<HyperRectangle<NUM_DIMS>>) {
    let mut set: LiftingSettings<NUM_DIMS> = LiftingSettings::<NUM_DIMS> {
        init: HyperRectangle::default(),
        reach_time: sim_time,
        initial_step_size: init_step_size,
        max_rect_width_before_error: 100.0,
        max_runtime_milliseconds: wall_time_ms,
        reached_at_intermediate_time: Some(intermediate_state),
        reached_at_final_time: Some(final_state),
        restarted_computation: Some(restarted_computation),
    };
    for d in 0..NUM_DIMS {
        set.init.dims[d].min = start[d];
        set.init.dims[d].max = start[d];
    }
    let mut storage_vec: Vec<HyperRectangle<NUM_DIMS>> = Vec::new();
    storage_vec.push(set.init);
    let safe = face_lifting_iterative_improvement(system_model, 
                                start_ms, 
                                &mut set, 
                                ctrl_input, 
                                store_rect, 
                                &mut storage_vec,
                                fixed_step,
                                dynamic_control);
    (safe, storage_vec)
}
