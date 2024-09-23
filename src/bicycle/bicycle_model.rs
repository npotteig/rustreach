use super::dynamics_bicycle::{BicycleModel, BICYCLE_NUM_DIMS as NUM_DIMS};
use super::simulate_bicycle::simulate_bicycle;
use super::super::rtreach::geometry::HyperRectangle;
use super::super::rtreach::obstacle_safety::{check_safety_obstacles, check_safety_wall};
use super::super::rtreach::face_lift::{LiftingSettings, face_lifting_iterative_improvement};
// a note from the f1tenth simulator 
// the car is 0.5 m long in the x direction 
// 0.3 long in the y direction

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

pub fn get_simulated_safe_time(system_model: &BicycleModel, start: [f64; NUM_DIMS], heading_input: f64, throttle: f64, store_state: bool) -> (f64, Vec<[f64; NUM_DIMS]>) {
    let step_size: f64 = 0.02;
    let mut rv: f64 = 0.0;
    let mut storage_vec: Vec<[f64; NUM_DIMS]> = Vec::new();
    simulate_bicycle(system_model, start, heading_input, throttle, step_size, should_stop, &mut rv, store_state, &mut storage_vec);

    (rv, storage_vec)
}

// called on states reached during the computation
pub fn intermediate_state(r: &mut HyperRectangle<NUM_DIMS>, store_rect: bool, storage_vec: &mut Vec<HyperRectangle<NUM_DIMS>>) -> bool {
    if store_rect {
        storage_vec.push(*r);
    }
    
    let mut allowed = true;
    //const REAL FIFTEEN_DEGREES_IN_RADIANS = 0.2618;

    // bloat the box for the width of the car
    r.dims[0].min = r.dims[0].min - 0.25;
    r.dims[0].max = r.dims[0].max + 0.25;
    r.dims[1].min = r.dims[1].min - 0.15;
    r.dims[1].max = r.dims[1].max + 0.15;

    allowed = check_safety_obstacles(r);

    if allowed {
        allowed = check_safety_wall(r);
    }

    // reset it
    r.dims[0].min = r.dims[0].min + 0.25;
    r.dims[0].max = r.dims[0].max - 0.25;
    r.dims[1].min = r.dims[1].min + 0.15;
    r.dims[1].max = r.dims[1].max - 0.15;

    if !allowed {
       println!("unsafe....");
    }
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

pub fn run_reachability_bicycle(system_model: &BicycleModel, 
                                start: [f64; NUM_DIMS], 
                                sim_time: f64,
                                init_step_size: f64, 
                                wall_time_ms: u64, 
                                start_ms: u64, 
                                heading_input: f64, 
                                throttle: f64, 
                                store_rect: bool,
                                fixed_step: bool) -> (bool, Vec<HyperRectangle<NUM_DIMS>>) {
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
                                                        &vec![heading_input, throttle], 
                                                        store_rect, 
                                                        &mut storage_vec,
                                                        fixed_step);
    (safe, storage_vec)
}
