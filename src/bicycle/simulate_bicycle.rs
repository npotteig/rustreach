// use super::dynamics_bicycle::{NUM_DIMS, get_derivative_bounds_bicycle};
use super::super::rtreach::geometry::HyperRectangle;
use super::super::rtreach::system_model::SystemModel;
use super::dynamics_bicycle::{BicycleModel, BICYCLE_NUM_DIMS as NUM_DIMS};
use super::super::rtreach::debug::DEBUG;

fn get_rk4_rect(
    system_model: &BicycleModel,
    rect: &HyperRectangle<NUM_DIMS>,
    ctrl_input: &Vec<f64>,
    step_size: f64,
) -> HyperRectangle<NUM_DIMS> {
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


    let mut out = HyperRectangle::<NUM_DIMS>::default();
    for d in 0..NUM_DIMS {
        out.dims[d].min = (step_size / 6.0) * (k1.dims[d].min + 2.0 * k2.dims[d].min + 2.0 * k3.dims[d].min + k4.dims[d].min);
        out.dims[d].max = (step_size / 6.0) * (k1.dims[d].max + 2.0 * k2.dims[d].max + 2.0 * k3.dims[d].max + k4.dims[d].max);
    }
    out
}

// simulate dynamics using Euler's method
pub fn simulate_bicycle(system_model: &BicycleModel, start_point: [f64; NUM_DIMS], heading_input: f64, throttle: f64,
    step_size: f64,
    should_stop: fn([f64; NUM_DIMS], f64, &mut f64) -> bool,
    stop_time: &mut f64,
    store_state: bool,
    storage_vec: &mut Vec<[f64; NUM_DIMS]>)
{
    let mut point: [f64; NUM_DIMS] = std::array::from_fn(|d| start_point[d]);
    if store_state {
        storage_vec.push(point);
    }

    let mut rect: HyperRectangle<NUM_DIMS> = HyperRectangle::default();
    let mut time: f64 = 0.0;

    loop{
        if should_stop(point, time, stop_time) {
            if DEBUG {
                println!("Quitting simulation: time {}, step_size: {}", time, step_size);
            }
            break;
        }

        // initialize the hyper-rectangle. Since we are doing simulation of a point then 
		// interval.min and interval.max are the same point
        for d in 0..NUM_DIMS {
            rect.dims[d].min = point[d];
            rect.dims[d].max = point[d];
        }

        // rk4 method
        let der = get_rk4_rect(system_model, &rect, &vec![heading_input, throttle], step_size);
        for d in 0..NUM_DIMS {
            point[d] += der.dims[d].min;
        }
        if store_state {
            storage_vec.push(point);
        }
        time += step_size;
    }

    println!("The state after {} s is: \n [{},{},{},{}] \n", time-step_size,point[0],point[1],point[2],point[3]);

}
