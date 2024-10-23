use tract_onnx::prelude::*;

use rtreach::geometry::*;
use rtreach::interval::*;
use rtreach::system_model::SystemModel;

pub const QUAD_NUM_DIMS: usize = 12;

// A linear model of a quadcopter. This model is a simplified version of the quadcopter dynamics.
// This model is based on the paper
// Quadcopter control: modeling, nonlinear control design, and simulation 
// by Francesco Sabatino, Masters Thesis, KTH, School of Electrical Engineering (EES), Automatic Control.

// x' = u
// y' = v
// z' = w
// phi' = p
// theta' = q
// psi' = r
// u' = -g * theta
// v' = g * phi
// w' = -f_t / m
// p' = tor_x / I_x
// q' = tor_y / I_y
// r' = tor_z / I_z

// (x, y, z) is the position of the quadcopter
// (phi, theta, psi) is the orientation of the quadcopter
// (u, v, w) is the linear velocity of the quadcopter
// (p, q, r) is the angular velocity of the quadcopter

// Inputs, there are four inputs
// f_t is the thrust input (N)
// tor_x is the torque input about the x-axis (Nm)
// tor_y is the torque input about the y-axis (Nm)
// tor_z is the torque input about the z-axis (Nm)

// Parameters from https://github.com/bobzwik/Quadcopter_SimCon/tree/master
// DJI F450 Quadcopter
// g = 9.81 m/s^2 is the acceleration due to gravity
// m = 1.2 kg is the mass of the quad
// I_x = 0.0123 kg m^2 is the moment of inertia about the x-axis
// I_y = 0.0123 kg m^2 is the moment of inertia about the y-axis
// I_z = 0.0224 kg m^2 is the moment of inertia about the z-axis

pub struct QuadcopterModel<'a>{
    pub goal: [f64; 3],
    pub ctrl_fn: fn(&[f64; QUAD_NUM_DIMS], &[f64; 3], Option<&'a SimplePlan<TypedFact, Box<dyn TypedOp>, Graph<TypedFact, Box<dyn TypedOp>>>>) -> [f64; 4],
    pub model: Option<&'a SimplePlan<TypedFact, Box<dyn TypedOp>, Graph<TypedFact, Box<dyn TypedOp>>>>
}

impl Default for QuadcopterModel<'_> {
    fn default() -> Self {
        QuadcopterModel {
            goal: [0.0; 3],
            ctrl_fn: |_, _, _| [0.0; 4],
            model: None
        }
    }
}

impl<'a> QuadcopterModel<'a> {
    pub fn set_ctrl_fn(&mut self, ctrl_fn: fn(&[f64; QUAD_NUM_DIMS], &[f64; 3], Option<&'a SimplePlan<TypedFact, Box<dyn TypedOp>, Graph<TypedFact, Box<dyn TypedOp>>>>) -> [f64; 4]) {
        self.ctrl_fn = ctrl_fn;
    }

    pub fn set_goal(&mut self, goal: [f64; 3]) {
        self.goal = goal;
    }

    pub fn set_model(&mut self, model: &'a SimplePlan<TypedFact, Box<dyn TypedOp>, Graph<TypedFact, Box<dyn TypedOp>>>) {
        self.model = Some(model);
    }

    pub fn sample_state_action(&self, state: &[f64; QUAD_NUM_DIMS]) -> [f64; 4] {
        (self.ctrl_fn)(state, &self.goal, self.model)
    }
}


impl SystemModel<QUAD_NUM_DIMS> for QuadcopterModel<'_> {
    fn get_derivative_bounds(
        &self,
        rect: &HyperRectangle<QUAD_NUM_DIMS>,
        face_index: usize,
        ctrl_inputs: &Vec<f64>,
    ) -> f64 {
        _get_derivative_bounds_quadcopter(rect, face_index, ctrl_inputs[0], ctrl_inputs[1], ctrl_inputs[2], ctrl_inputs[3])
    }

    fn sample_control(
            &self,
            rect: &HyperRectangle<QUAD_NUM_DIMS>,
        ) -> Vec<f64> {
        (self.ctrl_fn)(&rect.mean_point().dims, &self.goal, self.model).to_vec()
    }
}

// implement the derivative using interval arithmetic
fn _get_derivative_bounds_quadcopter(
    rect: &HyperRectangle<QUAD_NUM_DIMS>,
    face_index: usize,
    f_t: f64,
    tor_x: f64,
    tor_y: f64,
    tor_z: f64,
) -> f64 {
    let g = 9.81;
    let m = 1.2;
    let i_x = 0.0123;
    let i_y = 0.0123;
    let i_z = 0.0224;

    let dim: usize = face_index / 2;
    let is_min: bool = (face_index % 2) == 0;

    // Interval rv.min = rv.max = 0
    let rv: Interval;
    // let x: Interval = rect.dims[0];
    // let y: Interval = rect.dims[1];
    // let z: Interval = rect.dims[2];
    let phi: Interval = rect.dims[3];
    let theta: Interval = rect.dims[4];
    // let psi: Interval = rect.dims[5];
    let u: Interval = rect.dims[6];
    let v: Interval = rect.dims[7];
    let w: Interval = rect.dims[8];
    let p: Interval = rect.dims[9];
    let q: Interval = rect.dims[10];
    let r: Interval = rect.dims[11];

    match dim {
        0 => {
            // x' = u
            rv = u;
        },
        1 => {
            // y' = v
            rv = v;
        },
        2 => {
            // z' = w
            rv = w;
        },
        3 => {
            // phi' = p
            rv = p;
        },
        4 => {
            // theta' = q
            rv = q;
        },
        5 => {
            // psi' = r
            rv = r;
        },
        6 => {
            // u' = -g * theta
            rv = mul_interval(new_interval_v(-g), theta);
        },
        7 => {
            // v' = g * phi
            rv = mul_interval(new_interval_v(g), phi);
        },
        8 => {
            // w' = -f_t / m
            let a: Interval = new_interval_v(-f_t);
            let b: Interval = new_interval_v(m);
            let c: Interval = div_interval(a, b);
            rv = c;
        },
        9 => {
            // p' = tor_x / I_x
            let a: Interval = new_interval_v(tor_x);
            let b: Interval = new_interval_v(i_x);
            let c: Interval = div_interval(a, b);
            rv = c;
        },
        10 => {
            // q' = tor_y / I_y
            let a: Interval = new_interval_v(tor_y);
            let b: Interval = new_interval_v(i_y);
            let c: Interval = div_interval(a, b);
            rv = c;
        },
        11 => {
            // r' = tor_z / I_z
            let a: Interval = new_interval_v(tor_z);
            let b: Interval = new_interval_v(i_z);
            let c: Interval = div_interval(a, b);
            rv = c;
        },
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