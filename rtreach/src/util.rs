use std::sync::Mutex;
use csv::Writer;
use csv::ReaderBuilder;
use lazy_static::lazy_static;
use super::geometry::println;
use std::time::{SystemTime, UNIX_EPOCH, Duration};
use super::face_lift::LiftingSettings;
use super::geometry::HyperRectangle;

lazy_static! {
    pub static ref INIIALIZED: Mutex<bool> = Mutex::new(false);
    pub static ref START_SEC: Mutex<u64> = Mutex::new(0);
}

pub fn save_states_to_csv<const NUM_DIMS: usize>(filename: &str, data: &Vec<[f64; NUM_DIMS]>) {
    let mut wtr = Writer::from_path(filename).unwrap();

    let mut header = vec![];
    for d in 0..NUM_DIMS {
        header.push(format!("dim{}", d));
    }
    let _ = wtr.write_record(&header);

    for r in data {
        let mut record = vec![];
        for d in 0..NUM_DIMS {
            record.push(format!("{}", r[d]));
        }
        let _ = wtr.write_record(&record);
    }
    wtr.flush().unwrap();
}

pub fn save_rects_to_csv<const NUM_DIMS: usize>(filename: &str, data: &Vec<(f64, HyperRectangle<NUM_DIMS>)>) {
    let mut wtr = Writer::from_path(filename).unwrap();
    // Create a single header
    let mut header = vec![];
    header.push(format!("rect_time"));
    for d in 0..NUM_DIMS {
        header.push(format!("min{}", d));
        header.push(format!("max{}", d));
    }
    let _ = wtr.write_record(&header);

    // Write each HyperRectangle's min and max values
    for r in data {
        let mut record = vec![];
        record.push(format!("{}", r.0));
        for d in 0..NUM_DIMS {
            record.push(format!("{}", r.1.dims[d].min));
            record.push(format!("{}", r.1.dims[d].max));
        }
        let _ = wtr.write_record(&record);
    }
    wtr.flush().unwrap();
}

pub fn save_reachtubes_to_csv<const NUM_DIMS: usize>(filename: &str, data: &Vec<Vec<(f64, HyperRectangle<NUM_DIMS>)>>) {
    let mut wtr = Writer::from_path(filename).unwrap();
    // Create a single header
    let mut header = vec![];
    header.push(format!("time"));
    header.push(format!("rect_time"));
    for d in 0..NUM_DIMS {
        header.push(format!("min{}", d));
        header.push(format!("max{}", d));
    }
    let _ = wtr.write_record(&header);

    for time in 0..data.len() {
        for r in &data[time] {
            let mut record = vec![];
            record.push(format!("{}", time));
            record.push(format!("{}", r.0));
            for d in 0..NUM_DIMS {
                record.push(format!("{}", r.1.dims[d].min));
                record.push(format!("{}", r.1.dims[d].max));
            }
            let _ = wtr.write_record(&record);
        }
    }
    wtr.flush().unwrap();
}

pub fn load_paths_from_csv(filename: &std::path::PathBuf) -> Vec<Vec<[f64; 3]>>{
    let mut paths: Vec<Vec<[f64; 3]>> = vec![];
    println!("Loading paths from file: {:?}", filename);
    let mut reader = ReaderBuilder::new().has_headers(true).from_path(filename).unwrap();
    let mut cur_path_id = 0;
    let mut path: Vec<[f64; 3]> = vec![];
    for result in reader.records() {
        let record = result.unwrap();
        let path_id = record.get(0).unwrap().parse::<i32>().unwrap();
        let x = record.get(1).unwrap().parse::<f64>().unwrap();
        let y = record.get(2).unwrap().parse::<f64>().unwrap();
        let z = record.get(3).unwrap().parse::<f64>().unwrap();
        if path_id != cur_path_id {
            paths.push(path);
            path = vec![];
            cur_path_id = path_id;
        }
        path.push([x, y, z]);
    }
    paths.push(path);
    paths
}


pub fn error_exit<const NUM_DIMS: usize>(str: &str, error_print_params: &LiftingSettings<NUM_DIMS>, error_params_assigned: bool) {
    println!("Error: {}", str);

    // print the params that caused the error
    if error_params_assigned {
        println!("\nSettings:");
        println!("Reach Time = {}", error_print_params.reach_time);
        println!("Runtime = {} ms", error_print_params.max_runtime_milliseconds);
        println!("Init = ");
        println(&error_print_params.init);
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

pub fn distance_3d(pos1: &[f64], pos2: &[f64]) -> f64 {
    let dx = pos1[0] - pos2[0];
    let dy = pos1[1] - pos2[1];
    let dz = pos1[2] - pos2[2];
    norm(&[dx, dy, dz])
}

pub fn distance_2d(pos1: &[f64], pos2: &[f64]) -> f64 {
    let dx = pos1[0] - pos2[0];
    let dy = pos1[1] - pos2[1];
    norm(&[dx, dy])
}

pub fn norm(vec: &[f64]) -> f64 {
    let mut sum = 0.0;
    for v in vec {
        sum += v*v;
    }
    sum.sqrt()
}