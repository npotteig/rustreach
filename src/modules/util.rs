use std::sync::Mutex;
use lazy_static::lazy_static;
use super::geometry::println;
use std::time::{SystemTime, UNIX_EPOCH, Duration};
use super::face_lift::LiftingSettings;

lazy_static! {
    pub static ref ERROR_PRINT_PARAMS: Mutex<Option<LiftingSettings>> = Mutex::new(None);
    pub static ref ERROR_PARAMS_ASSIGNED: Mutex<bool> = Mutex::new(false);
    pub static ref INIIALIZED: Mutex<bool> = Mutex::new(false);
    pub static ref START_SEC: Mutex<u64> = Mutex::new(0);
}

pub fn set_error_print_params(params: &LiftingSettings) {
    {
        let mut guard: std::sync::MutexGuard<'_, Option<LiftingSettings>> = ERROR_PRINT_PARAMS.lock().unwrap();
        *guard = Some(*params);
    }
    {
        let mut guard: std::sync::MutexGuard<'_, bool> = ERROR_PARAMS_ASSIGNED.lock().unwrap();
        *guard = false;
    }
}


pub fn error_exit(str: &str) {
    println!("Error: {}", str);
    let error_params_assigned: bool = *ERROR_PARAMS_ASSIGNED.lock().unwrap();
    let error_print_params: std::sync::MutexGuard<'_, Option<LiftingSettings>> = ERROR_PRINT_PARAMS.lock().unwrap();

    // print the params that caused the error
    if error_params_assigned {
        match *error_print_params{
            Some(ref settings) => {
                println!("\nSettings:");
                println!("Reach Time = {}", settings.reach_time);
                println!("Runtime = {} ms", settings.max_runtime_milliseconds);
                println!("Init = ");
                println(&settings.init);
            }
            None => {
                println!("Error print params were not assigned.");
            }
        }
    } else {
        println!("Error print params were not assigned.");
    }

    std::process::exit(1);
}

pub fn milliseconds() -> u64 {
    // Set startSec to 0
    let mut start_sec: std::sync::MutexGuard<'_, u64> = START_SEC.lock().unwrap();
    *start_sec = 0;

    // Get current time
    let now: SystemTime = SystemTime::now();
    let now_since_epoch: Duration = now.duration_since(UNIX_EPOCH).unwrap();
    let now_secs: u64 = now_since_epoch.as_secs();
    let now_millis: u32 = now_since_epoch.subsec_millis();

    // Initialize if not already
    let mut initialized: std::sync::MutexGuard<'_, bool> = INIIALIZED.lock().unwrap();
    if !*initialized {
        *initialized = true;
    }

    // Calculate difference in milliseconds
    let dif_sec: u64 = now_secs - *start_sec;
    let ds: u64 = dif_sec * 1000 + now_millis as u64;

    // Uncomment to print the result
    // println!("from milliseconds: {}", ds);

    ds
}


pub fn milliseconds2(t1: &Duration) -> u64 {
    // Get the current time
    let now: SystemTime = SystemTime::now();
    let now_duration: Duration = now.duration_since(UNIX_EPOCH).unwrap();
    let now_sec: u64 = now_duration.as_secs();
    let now_usec: u32 = now_duration.subsec_micros();

    // Calculate elapsed time
    let elapsed_time_secs: i64 = now_sec as i64 - t1.as_secs() as i64;
    let elapsed_time_usecs: i64 = now_usec as i64 - t1.subsec_micros() as i64;

    let mut elapsed_time_millis: i64 = elapsed_time_secs * 1000;
    elapsed_time_millis += elapsed_time_usecs / 1000;

    elapsed_time_millis as u64
}