use std::f64::consts::PI;

pub fn distance(pos1: &[f64], pos2: &[f64]) -> f64 {
    let dx = pos1[0] - pos2[0];
    let dy = pos1[1] - pos2[1];
    let dz = pos1[2] - pos2[2];
    norm(&[dx, dy, dz])
}

pub fn norm(vec: &[f64]) -> f64 {
    let mut sum = 0.0;
    for v in vec {
        sum += v*v;
    }
    sum.sqrt()
}

// Function to normalize the angle between -π and π
pub fn normalize_angle(angle: f64) -> f64 {
    let mut normalized = angle;
    while normalized > PI {
        normalized -= 2.0 * PI;
    }
    while normalized < -PI {
        normalized += 2.0 * PI;
    }
    normalized
}