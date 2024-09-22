// use super::dynamics_bicycle::{NUM_DIMS, get_derivative_bounds_bicycle};
use super::super::rtreach::geometry::HyperRectangle;
use super::super::rtreach::system_model::SystemModel;
use super::dynamics_bicycle::{BicycleModel, BICYCLE_NUM_DIMS as NUM_DIMS};
use super::super::rtreach::debug::DEBUG;

// simulate dynamics using Euler's method
pub fn simulate_bicycle(system_model: &BicycleModel, start_point: [f64; NUM_DIMS], heading_input: f64, throttle: f64,
    step_size: f64,
    should_stop: fn([f64; NUM_DIMS], f64, &mut f64) -> bool,
    stop_time: &mut f64)
{
    let mut point: [f64; NUM_DIMS] = std::array::from_fn(|d| start_point[d]);

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

        // euler's method
        for d in 0..NUM_DIMS {
            let der: f64 = system_model.get_derivative_bounds(&rect, 2*d, &vec![heading_input, throttle]);

            point[d] += step_size * der;
        }

        time += step_size;
    }

    println!("The state after {} s is: \n [{},{},{},{}] \n", time-step_size,point[0],point[1],point[2],point[3]);

}
