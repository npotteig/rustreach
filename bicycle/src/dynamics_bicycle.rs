use rtreach::geometry::*;
use rtreach::interval::*;
use rtreach::system_model::SystemModel;

pub const BICYCLE_NUM_DIMS: usize = 4;

// a bicycle model to model the car's dynamics. The bicycle model is a standard model for cars with front steering. 
// This model tracks well for slow speeds

// x' = v * cos(theta + beta)
// y' = v * sin(theta + beta)
// v' = -ca * v + ca*cm*(u - ch)
// theta' = v * (cos(beta)/(lf+lr)) * tan(delta)
// beta = arctan(lr*tan(delta)/(lf+lr))

// for simplicity we are going to assume beta is 0. 

// Inputs, there are two inputs
// u is the throttle input
// delta is the heading input

// v is linear velocity 
// theta car's orientation 
// Beta is the car's slip angle 
// x and y are car's position
// u is throttle input 
// delta is the heading input 
// ca is an acceleration 
// cm is car motor constant
// ch is hysterisis constant
// lf, lr are the distances from the car's center of mass 

// parameters from (https://repository.upenn.edu/cgi/viewcontent.cgi?article=1908&context=cis_papers)
// ca = 1.633
// cm = 0.2
// ch = 4
// lf = 0.225
// lr = 0.225
// u = 16 constant speed (2.4 m/s)

// state vector x,y,v,theta

pub struct BicycleModel;

impl SystemModel<BICYCLE_NUM_DIMS> for BicycleModel {
    fn get_derivative_bounds(
        &self,
        rect: &HyperRectangle<BICYCLE_NUM_DIMS>,
        face_index: usize,
        ctrl_inputs: &Vec<f64>,
    ) -> f64 {
        _get_derivative_bounds_bicycle(rect, face_index, ctrl_inputs[0], ctrl_inputs[1])
    }
}

// implement the derivative using interval arithmetic
fn _get_derivative_bounds_bicycle(
    rect: &HyperRectangle<BICYCLE_NUM_DIMS>,
    face_index: usize,
    heading_input: f64,
    throttle: f64,
) -> f64 {
    let u: f64 = throttle;
    let delta: f64 = heading_input;
    let ca: f64 = 1.9569;      // 1.633
    let cm: f64 = 0.0342;      // 0.2
    let ch: f64 = -37.1967;    // 4
    let lf: f64 = 0.225;
    let lr: f64 = 0.225;

    let dim: usize = face_index / 2;
    let is_min: bool = (face_index % 2) == 0;

    // Interval rv.min = rv.max = 0
    let mut rv: Interval = new_interval_v(0.0);
    let v: Interval = rect.dims[2];
    let theta: Interval = rect.dims[3];

    // beta = arctan(lr * tan(delta) / (lf + lr))

    match dim {
        0 => {
            // x' = v * cos(theta + beta)
            rv = mul_interval(v, cos_interval(theta));
        }
        1 => {
            // y' = v * sin(theta + beta)
            rv = mul_interval(v, sin_interval(theta));
        }
        2 => {
            // v' = -ca * v + ca * cm * (u - ch)
            let a: Interval = mul_interval(v, new_interval_v(-ca));
            let b: Interval = mul_interval(new_interval_v(ca), new_interval_v(cm));
            let c: Interval = sub_interval(new_interval_v(u), new_interval_v(ch));
            let d: Interval = mul_interval(b, c);
            rv = add_interval(a, d);
        }
        3 => {
            // theta' = v * (cos(beta) / (lf + lr)) * tan(delta)
            let mult: Interval = new_interval_v(1.0 / (lf + lr));
            let a: Interval = mul_interval(v, mult);
            let delt: Interval = new_interval_v(delta);
            let tan: Interval = div_interval(sin_interval(delt), cos_interval(delt));
            rv = mul_interval(a, tan);
        }
        _ => {
            eprintln!("Error: Invalid Dimension");
            std::process::exit(1);
        }
    }

    if is_min {
        rv.min
    } else {
        rv.max
    }
}