use std::sync::Mutex;
use std::fs::File;
use std::io::{self, BufRead, BufReader};
use lazy_static::lazy_static;
use super::geometry::HyperRectangle;

// Define global variables using lazy_static
lazy_static! {
    pub static ref WALL_COORDS: Mutex<Option<Vec<Vec<f64>>>> = Mutex::new(None);
    pub static ref OBSTACLES: Mutex<Option<Vec<Vec<Vec<f64>>>>> = Mutex::new(None);
    static ref FILE_ROWS: Mutex<u32> = Mutex::new(0);
    static ref FILE_COLUMNS: Mutex<u32> = Mutex::new(2);
    static ref OBSTACLE_COUNT: Mutex<u32> = Mutex::new(0);
}

fn count_lines(filename: &str) -> io::Result<usize> {
    // Open the file
    let file = File::open(filename);

    match file {
        Ok(f) => {
            // Use BufReader for efficient reading line by line
            let reader = BufReader::new(f);
            
            Ok(reader.lines().count())
        },
        Err(e) => {
            // File could not be opened
            eprintln!("Could not open file {}: {}", filename, e);
            Err(e)
        }
    }
    
}

pub fn load_wallpoints(filename: &str, print: bool){
    {
        let mut file_rows: std::sync::MutexGuard<'_, u32> = FILE_ROWS.lock().unwrap();
        *file_rows = count_lines(filename).unwrap() as u32;
    }

    if print{
        println!("Opening file... with {} points", FILE_ROWS.lock().unwrap());
    }

    let wall_points = File::open(filename);
    match wall_points {
        Ok(f) => {
            let reader: BufReader<File> = BufReader::new(f);
            let mut wall_coords: Vec<Vec<f64>> = Vec::new();
            for line in reader.lines() {
                let line = line.unwrap();
                let coords: Vec<f64> = line.split(", ")
                    .map(|s| s.parse().unwrap_or_default())
                    .collect();
                wall_coords.push(coords);
            }
            {
                let mut wall_coords_lock: std::sync::MutexGuard<'_, Option<Vec<Vec<f64>>>> = WALL_COORDS.lock().unwrap();
                *wall_coords_lock = Some(wall_coords);
            }
        },
        Err(e) => {
            eprintln!("Could not open file {}: {}", filename, e);
        }
    }
}

pub fn check_safety<const NUM_DIMS: usize>(rect: &HyperRectangle<NUM_DIMS>, cone: &[[f64; 2]; 2]) -> bool {
    let l1: [f64; 2] = [rect.dims[0].min, rect.dims[1].max];
    let r1: [f64; 2] = [rect.dims[0].max, rect.dims[1].min];

    let l2: [f64; 2] = [cone[0][0], cone[1][1]];
    let r2: [f64; 2] = [cone[0][1], cone[1][0]];

    if l1[0] >= r2[0] || l2[0] >= r1[0] {
        return true;
    }

    if l1[1] <= r2[1] || l2[1] <= r1[1] {
        return true;
    }

    false
}

pub fn check_safety_obstacles<const NUM_DIMS: usize>(rect: &HyperRectangle<NUM_DIMS>) -> bool {
    let mut allowed: bool = true;

    let obstacle_count: u32 = *OBSTACLE_COUNT.lock().unwrap();
    let obstacles: &Option<Vec<Vec<Vec<f64>>>> = &*OBSTACLES.lock().unwrap();

    match obstacles {
        Some(obst) => {
            for j in 0..obstacle_count {
                let obs: [[f64; 2]; 2] = [
                    [obst[j as usize][0][0], obst[j as usize][0][1]],
                    [obst[j as usize][1][0], obst[j as usize][1][1]]
                ];
                
                allowed = check_safety(rect, &obs);
                if !allowed {
                    println!("offending cone [{}, {}], [{}, {}]",
                    obst[j as usize][0][0], obst[j as usize][0][1],
                    obst[j as usize][1][0], obst[j as usize][1][1]);
                    break;
                }
            }
        },
        None => {
            allowed = true;
        }
    }
    
    allowed
}

pub fn check_safety_wall<const NUM_DIMS: usize>(rect: &HyperRectangle<NUM_DIMS>) -> bool {
    let wall_coords_guard: &Option<Vec<Vec<f64>>> = &*WALL_COORDS.lock().unwrap();
    let file_rows: u32 = *FILE_ROWS.lock().unwrap();  // Get the value of file_rows
    
    let mut safe: bool = true;

    match wall_coords_guard {
        Some(wall_coords) => {

            for i in 0..file_rows {
                // Access the wall coordinates
                let point: [[f64; 2]; 2] = [
                    [wall_coords[i as usize][0], wall_coords[i as usize][0]], // Assuming you meant `wallCoords[i][1]` for both coordinates
                    [wall_coords[i as usize][1], wall_coords[i as usize][1]],
                ];
                
                safe = check_safety(rect, &point);
                if !safe {
                    // Uncomment the next line to print the offending point
                    println!("offending point ({}, {})", wall_coords[i as usize][0], wall_coords[i as usize][1]);
                    break;
                }
            }
        },
        None => {
            safe = true;
        }
    }
    
    safe
}

pub fn allocate_obstacles(num_obstacles: u32, points: &[[f64; 2]]){
    let rows: usize = num_obstacles as usize;
    let mut obstacle_count = OBSTACLE_COUNT.lock().unwrap();
    *obstacle_count = num_obstacles;
    let cols: usize = 2;
    let height: usize = 2;
    let w: f64 = 0.13;
    let h: f64 = 0.13;

    println!("interval list of obstacles: ");
    let mut obstacles = vec![vec![vec![0.0; cols]; height]; rows];
    for i in 0..rows {
        obstacles[i][0][0] = points[i][0] - w/2.0;
        obstacles[i][0][1] = points[i][0] + w/2.0;
        obstacles[i][1][0] = points[i][1] - h/2.0;
        obstacles[i][1][1] = points[i][1] + h/2.0;
        println!("[{}, {}], [{}, {}]", obstacles[i][0][0], obstacles[i][0][1], obstacles[i][1][0], obstacles[i][1][1]);
    }
    println!();
    {
        let mut obstacles_lock = OBSTACLES.lock().unwrap();
        *obstacles_lock = Some(obstacles);
    }
}