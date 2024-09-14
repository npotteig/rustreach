use std::sync::Mutex;
use lazy_static::lazy_static;
use super::geometry::HyperRectangle;

lazy_static! {
    pub static ref ITERATIONS_AT_QUIT: Mutex<u64> = Mutex::new(0); // lazy_
}

#[derive(Copy, Clone)]
pub struct LiftingSettings {
    pub init: HyperRectangle,                // initial rectangle
    pub reach_time: f64,                     // total reach time
    pub initial_step_size: f64,              // the initial size of the steps to use
    pub max_rect_width_before_error: f64,    // maximum allowed rectangle size
    pub max_runtime_milliseconds: u64,       // maximum runtime in milliseconds
    pub reached_at_intermediate_time: Option<fn(&mut HyperRectangle) -> bool>, // callback for intermediate time
    pub reached_at_final_time: Option<fn(&mut HyperRectangle) -> bool>,        // callback for final time
    pub restarted_computation: Option<fn()>,         // callback for restarted computation
}