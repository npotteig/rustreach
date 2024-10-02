use std::sync::Mutex;
use std::time::{SystemTime, UNIX_EPOCH, Duration};
use std::vec;
use lazy_static::lazy_static;
use super::geometry::*;
use super::system_model::SystemModel;
use super::util::*;
use super::debug::DEBUG;

lazy_static! {
    pub static ref ITERATIONS_AT_QUIT: Mutex<u64> = Mutex::new(0); // lazy_
}

#[derive(Copy, Clone)]
pub struct LiftingSettings<const NUM_DIMS: usize> {
    pub init: HyperRectangle<NUM_DIMS>,                // initial rectangle
    pub reach_time: f64,                     // total reach time
    pub initial_step_size: f64,              // the initial size of the steps to use
    pub max_rect_width_before_error: f64,    // maximum allowed rectangle size
    pub max_runtime_milliseconds: u64,       // maximum runtime in milliseconds
    pub reached_at_intermediate_time: Option<fn(&mut HyperRectangle<NUM_DIMS>, store_rect: bool, storage_vec: &mut Vec<HyperRectangle<NUM_DIMS>>) -> bool>, // callback for intermediate time
    pub reached_at_final_time: Option<fn(&mut HyperRectangle<NUM_DIMS>, store_rect: bool, storage_vec: &mut Vec<HyperRectangle<NUM_DIMS>>) -> bool>,        // callback for final time
    pub restarted_computation: Option<fn(store_rect: bool, storage_vec: &mut Vec<HyperRectangle<NUM_DIMS>>)>,         // callback for restarted computation
}

// Constants necessary to guarantee loop termination.
// These bound the values of the derivatives
pub const MAX_DER_B: f64 = 99999.0;
pub const MIN_DER_B: f64 = -99999.0;

fn get_rk4_rect_terms<const NUM_DIMS: usize, T: SystemModel<NUM_DIMS>>(
    system_model: &T,
    rect: &HyperRectangle<NUM_DIMS>,
    face_index: usize,
    ctrl_input: &Vec<f64>,
    step_size: f64,
) -> f64 {
    let dim: usize = face_index / 2;
    let is_min: bool = (face_index % 2) == 0;
    let k1 = system_model.get_derivative_bounds_rect(rect, ctrl_input);
    let mut rect_k2 = *rect;
    for d in 0..NUM_DIMS{
        rect_k2.dims[d].min += k1.dims[d].min * step_size / 2.0;
        rect_k2.dims[d].max += k1.dims[d].max * step_size / 2.0;
    }

    let k2 = system_model.get_derivative_bounds_rect(&rect_k2, ctrl_input);
    let mut rect_k3 = *rect;
    for d in 0..NUM_DIMS{
        rect_k3.dims[d].min += k2.dims[d].min * step_size / 2.0;
        rect_k3.dims[d].max += k2.dims[d].max * step_size / 2.0;
    }

    let k3 = system_model.get_derivative_bounds_rect(&rect_k3, ctrl_input);
    let mut rect_k4 = *rect;
    for d in 0..NUM_DIMS{
        rect_k4.dims[d].min += k3.dims[d].min * step_size;
        rect_k4.dims[d].max += k3.dims[d].max * step_size;
    }

    let k4 = system_model.get_derivative_bounds_rect(&rect_k4, ctrl_input);

    if is_min{
        (step_size / 6.0) * (k1.dims[dim].min + 2.0 * k2.dims[dim].min + 2.0 * k3.dims[dim].min + k4.dims[dim].min)
    }
    else{
        (step_size / 6.0) * (k1.dims[dim].max + 2.0 * k2.dims[dim].max + 2.0 * k3.dims[dim].max + k4.dims[dim].max)
    }
}

// make a face's neighborhood of a given width
// At each dimension, there are two faces corresponding to that dimension, minimum_face and maximum_face
// For example, Rect: 0 <= x <= 2: the minimum_face is at x = 0 (a point in this case), the maximum_face is at x = 2
// For two dimensional Rectangle:     0 <= x1 <= 2; 1 <= x2 <= 3: at the dimension 1 (i.e., x1 axis) the minimum face
// is a line x1 = 0, 1 <= x2 <= 3 and the maximum face is a line x1 = 2, 1 <= x2 <= 3

fn make_neighborhood_rect<const NUM_DIMS: usize>(
    out: &mut HyperRectangle<NUM_DIMS>,
    face_index: usize,
    bloated_rect: &HyperRectangle<NUM_DIMS>,
    original_rect: &HyperRectangle<NUM_DIMS>,
    neb_width: f64,
) {
    *out = *bloated_rect; // Clone the bloated rectangle into out

    let is_min: bool = (face_index % 2) == 0;
    let dimension: usize = face_index / 2;

    // Flatten
    // The derivatives are evaluated along the face
    // So what the next line does is take face value based on the dimension 
    // and whether the face is oriented to the negative or positive direction, respectively
    // e_i+ = x_i = ui, e_i- = l_i

    // Select the negative face for this dimension
    if is_min {
        out.dims[dimension].min = original_rect.dims[dimension].min;
        out.dims[dimension].max = original_rect.dims[dimension].min;
    } else {
        // Select the positive face for this dimension
        out.dims[dimension].min = original_rect.dims[dimension].max;
        out.dims[dimension].max = original_rect.dims[dimension].max;
    }

    // Depending on the value returned by the derivative 
    // Extend the dimensions by the derivative
    // If it's a negative facing face, the negative directions move it outward 
    // and vice versa
    // Swap if nebWidth was negative

    if neb_width < 0.0 {
        out.dims[dimension].min += neb_width;
    } else {
        out.dims[dimension].max += neb_width;
    }
}

// do a single face lifting operation
// et (error tracker) is set if you want to track the sources of errors, can be null
// returns time elapsed

fn lift_single_rect<const NUM_DIMS: usize, T: SystemModel<NUM_DIMS>>(system_model: &T, rect: &mut HyperRectangle<NUM_DIMS>, step_size: f64, time_remaining: f64, ctrl_input: &Vec<f64>, settings: &LiftingSettings<NUM_DIMS>) -> f64 {
    // Create a copy of the rectangle for face-lifting operations
    let mut bloated_rect: HyperRectangle<NUM_DIMS> = *rect;
    
    // Create an array to store neighborhood widths
    let mut neb_width = vec![0.0; system_model.num_faces()];

    let mut need_recompute: bool = true;
    let mut min_neb_cross_time: f64 = 0.0;
    let mut min_neb_cross_time_rk4: f64 = 0.0;
    let mut ders_rk4_term = vec![0.0; system_model.num_faces()];
    let mut face_rects = vec![HyperRectangle::<NUM_DIMS>::default(); system_model.num_faces()];
    while need_recompute {
        need_recompute = false;
        min_neb_cross_time = f64::MAX;
        min_neb_cross_time_rk4 = f64::MAX;
    
        for f in 0..system_model.num_faces() {
            let dim: usize = f / 2;
            let is_min: bool = (f % 2) == 0;

            let mut face_neb_rect: HyperRectangle<NUM_DIMS> = HyperRectangle::<NUM_DIMS>::default();

            // make candidate neighborhood
            make_neighborhood_rect::<NUM_DIMS>(&mut face_neb_rect, f, &bloated_rect, rect, neb_width[f]);

            // test derivative inside neighborhood
            let mut der_rk4_term = get_rk4_rect_terms(system_model, &face_neb_rect, f, ctrl_input, step_size);

            // so we cap the derivative at 999999 and min at the negative of that
            if der_rk4_term > MAX_DER_B {
                der_rk4_term = MAX_DER_B;
            } else if der_rk4_term < MIN_DER_B {
                der_rk4_term = MIN_DER_B;
            }

            let prev_neb_width: f64 = neb_width[f];
            let mut new_neb_width: f64 = der_rk4_term;

            // check if it's growing outward
            let grew_outward = (is_min && new_neb_width < 0.0) || (!is_min && new_neb_width > 0.0);
            let prev_grew_outward = (is_min && prev_neb_width < 0.0) || (!is_min && prev_neb_width > 0.0);

            // prevent flipping from outward face to inward face
            if !grew_outward && prev_grew_outward {
                new_neb_width = 0.0;
                der_rk4_term = 0.0;
            }

            // check if recomputation is needed
            if !prev_grew_outward && grew_outward {
                need_recompute = true;
            }

            if new_neb_width.abs() > 2.0 * prev_neb_width.abs() {
                need_recompute = true;
            }

            if need_recompute {
                neb_width[f] = new_neb_width;

                if is_min && neb_width[f] < 0.0 {
                    bloated_rect.dims[dim].min = rect.dims[dim].min + neb_width[f];
                } else if !is_min && neb_width[f] > 0.0 {
                    bloated_rect.dims[dim].max = rect.dims[dim].max + neb_width[f];
                }

            } else {
                // last iteration, compute min time to cross face

                // clamp derivative if it changed direction
				// this means along the face it's inward, but in the neighborhood it's outward
                if der_rk4_term < 0.0 && prev_neb_width > 0.0 {
                    der_rk4_term = 0.0;
                } else if der_rk4_term > 0.0 && prev_neb_width < 0.0 {
                    der_rk4_term = 0.0;
                }

                if der_rk4_term != 0.0 {
                    let cross_time_rk4: f64 = prev_neb_width * step_size / der_rk4_term;
                    if cross_time_rk4 < min_neb_cross_time_rk4 {
                        min_neb_cross_time_rk4 = cross_time_rk4;
                    }
                }

                
                face_rects[f] = face_neb_rect;
                ders_rk4_term[f] = der_rk4_term;
            }
        }
    }

    // just as a note the minTime to cross is the prevNebwidth / der
	// the nebWidth btw is stepSize * der
    if min_neb_cross_time * 2.0 < step_size {
        error_exit("minNebCrossTime is less than half of step size.", settings, true);
    }

    ////////////////////////////////////////
	// lift each face by the minimum time //

    let mut time_to_elapse: f64 = min_neb_cross_time_rk4;


	// subtract a tiny amount time due to multiplication / division rounding
	time_to_elapse = time_to_elapse * 99999.0 / 100000.0;

    if time_remaining < time_to_elapse {
        time_to_elapse = time_remaining;
    }

    // do the lifting
    for d in 0..NUM_DIMS{
        if ders_rk4_term[2*d] != 0.0{
            let rk4_min = get_rk4_rect_terms(system_model, &face_rects[2*d], 2*d, ctrl_input, time_to_elapse);
            rect.dims[d].min += rk4_min;
        }

        if ders_rk4_term[2*d+1] != 0.0{
            let rk4_max = get_rk4_rect_terms(system_model, &face_rects[2*d+1], 2*d+1, ctrl_input, time_to_elapse);
            rect.dims[d].max += rk4_max;
        }
    } 

    if !hyperrectangle_contains(&bloated_rect, rect, true){
        error_exit("lifted rect is outside of bloated rect", &settings, true);
    }

    time_to_elapse
}

pub fn face_lifting_iterative_improvement<const NUM_DIMS: usize, T: SystemModel<NUM_DIMS>>(
    system_model: &T,
    start_ms: u64,
    settings: &mut LiftingSettings<NUM_DIMS>,
    ctrl_input: &Vec<f64>,
    store_rect: bool,
    storage_vec: &mut Vec<HyperRectangle<NUM_DIMS>>,
    fixed_step: bool,
) -> bool {
    let mut rv = false;
    let mut last_iteration_safe = false;
    
    let now: SystemTime = SystemTime::now();
    let start: Duration = now.duration_since(UNIX_EPOCH).unwrap();
    let mut elapsed_total: u64 = 0;

    // Get the settings from the facelifting settings
    let mut step_size: f64 = settings.initial_step_size;

    let mut iter: u64 = 0;           // number of iterations
    let mut previous_iter: u64 = 1;
    let mut elapsed_prev: u64 = 0;
    let mut next_iter_estimate: u64 = 0;

    loop{
        iter += 1;
        let mut safe: bool = true; // until proven otherwise

        // if we've cut the step size way too far where floating point error may be a factor
        if step_size < 0.0000001 {
            if DEBUG{
                println!("Quitting from step size too small: stepSize: {} at iteration: {}\n\r", step_size, iter);
            }
            rv = false;
            break;
        }

        // this is primarily used in the plotting functions, so that we only plot the last iteration
        if let Some(restart_func) = settings.restarted_computation {
            restart_func(store_rect, storage_vec);
        }

        // This function gets the reachtime passed from the settings
        let mut time_remaining: f64 = settings.reach_time; 

		// Get the initial set from which to perform reachability analysis.
        let mut tracked_rect = settings.init;

		// Create a new hyperrectangle
		let mut hull: HyperRectangle<NUM_DIMS> = HyperRectangle::default();

		// I want to visualize an over-approximation of the over-all reachset too
        let mut total_hull: HyperRectangle<NUM_DIMS> = tracked_rect;

        // compute reachability up to split time
		while safe && time_remaining > 0.0 {
            // reachedAtIntermediateTime is a function that checks the current hyper-rectangle against the safety specification,
			// whatever that might be
            if let Some(_) = settings.reached_at_intermediate_time {
                hull = tracked_rect;
            }

            // debug changed so error tracker is always passed in (see note)
            let time_elapsed: f64 = lift_single_rect::<NUM_DIMS, T>(system_model, &mut tracked_rect, step_size, time_remaining, &ctrl_input, settings);

            // if we're not even close to the desired step size
            if hyperrectangle_max_width(&tracked_rect) > settings.max_rect_width_before_error {
                if DEBUG{
                    println!("maxRectWidthBeforeError exceeded at time {}, rect = ", settings.reach_time - time_remaining);
                    println(&tracked_rect);
                }
                safe = false;
            } else if let Some(reached_at_intermediate_time) = settings.reached_at_intermediate_time {
                hyperrectangle_grow_to_convex_hull(&mut hull, &tracked_rect);
                hyperrectangle_grow_to_convex_hull(&mut total_hull, &tracked_rect);
                
                // println!("safe1: {}", safe);
                safe = safe && reached_at_intermediate_time(&mut hull, store_rect, storage_vec);
                // println!("safe2: {}", safe);
            }

            if time_elapsed == time_remaining{
                if let Some(reached_at_final_time) = settings.reached_at_final_time {
                    safe = safe && reached_at_final_time(&mut tracked_rect, store_rect, storage_vec);
                }
            }

            time_remaining -= time_elapsed;
            
        } // This is the end of this first while loop
        // This does the reachset computation for the reachtime
        // it continues until the simulation time is over, or we encounter an unsafe state,
        // whichever occurs first. 

        // Don't do another iteration unless you want to miss the deadline
        let now: u64 = milliseconds2(&start);
        elapsed_total = now;
        previous_iter = elapsed_total - elapsed_prev;

        // its O(2^N) in terms of box checking so have to scale the next iteration by 2 and add 1ms (for over-approximating how long it takes to compute the reachset)
        if previous_iter == 0{
            next_iter_estimate = 2;
        } else {
            if (previous_iter * 2 + 1) < next_iter_estimate{
                next_iter_estimate = next_iter_estimate * 2;
            } else {
                next_iter_estimate = previous_iter * 2 + 1;
            }
        }

        elapsed_prev = elapsed_total;
        if settings.max_runtime_milliseconds > 0 {
            let remaining: i32  = settings.max_runtime_milliseconds as i32 - elapsed_total as i32;
            if DEBUG && remaining < 0 {
                println!("remaining: {}\r\n",remaining);
            }
            if remaining <= next_iter_estimate as i32 {
                // we've exceeded our time, use the result from the last iteration
				// note in a real system you would have an interrupt or something to cut off computation
                if DEBUG{
                    println!("Quitting from runtime maxed out");
                    println(&tracked_rect);
                }

                if let Some(reached_at_final_time) = settings.reached_at_final_time {
                    reached_at_final_time(&mut total_hull, store_rect, storage_vec);
                }
                if iter > 1 {
                    rv = last_iteration_safe;
                } else {
                    rv = safe;
                }
                break;
            }
            if !safe{
                if let Some(reached_at_final_time) = settings.reached_at_final_time {
                    reached_at_final_time(&mut total_hull, store_rect, storage_vec);
                }
            }
        } else {
            if settings.max_runtime_milliseconds == 0 {
                settings.max_runtime_milliseconds += 1;
                if DEBUG{
                    println!("Splitting\n\r");
                }
                rv = safe;
                break;
            }
        }

        
		last_iteration_safe = safe;
        if fixed_step{
            rv = safe;
            break;
        }
		// apply error-reducing strategy
		step_size /= 2.0;

    }

    {
        let mut iterations_at_quit: std::sync::MutexGuard<'_, u64> = ITERATIONS_AT_QUIT.lock().unwrap();
        *iterations_at_quit = iter;
    }

    if DEBUG{
        println!("{}ms: step_size = {}", elapsed_total, step_size);
        println!("iterations at quit: {}", iter);
    }

    rv
}
