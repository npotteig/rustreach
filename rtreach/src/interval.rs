use super::geometry::Interval;

pub const TWO_PI: f64 = 2.0 * std::f64::consts::PI;


pub fn new_interval(min: f64, max: f64) -> Interval {
    Interval { min, max }
}

pub fn new_interval_v(val: f64) -> Interval {
    Interval { min: val, max: val }
}

pub fn add_interval(i: Interval, j: Interval) -> Interval {
    Interval {
        min: i.min + j.min,
        max: i.max + j.max,
    }
}

pub fn sub_interval(i: Interval, j: Interval) -> Interval {
    Interval {
        min: i.min - j.max,
        max: i.max - j.min,
    }
}

pub fn mul_interval(i: Interval, j: Interval) -> Interval {
    let a: f64 = i.min;
    let b: f64 = i.max;
    let c: f64 = j.min;
    let d: f64 = j.max;

    Interval {
        min: f64::min(f64::min(a * c, a * d), f64::min(b * c, b * d)),
        max: f64::max(f64::max(a * c, a * d), f64::max(b * c, b * d)),
    }
}

pub fn div_interval(i: Interval, j: Interval) -> Interval {
    let c: f64 = j.min;
    let d: f64 = j.max;

    mul_interval(i, new_interval(1.0 / d, 1.0 / c))
}

pub fn pow_interval(i: Interval, n: i32) -> Interval {
    let a: f64 = i.min;
    let b: f64 = i.max;
    let c: f64;
    let d: f64;

    if n % 2 == 1 {
        // If n is odd, raise both min and max to power n directly
        c = a.powi(n);
        d = b.powi(n);
    } else {
        // If n is even, check the signs of a and b
        if a >= 0.0 {
            // Both are non-negative
            c = a.powi(n);
            d = b.powi(n);
        } else if b < 0.0 {
            // Both are negative
            c = b.powi(n);
            d = a.powi(n);
        } else {
            // a < 0 and b >= 0, so the interval crosses zero
            c = 0.0;
            d = f64::max(a.powi(n), b.powi(n));
        }
    }

    new_interval(c, d)
}

pub fn sin_interval(i: Interval) -> Interval {
    let a: f64 = i.min;
    let b: f64 = i.max;
    let c: f64;
    let d: f64;
    
    if ((a-1.5*std::f64::consts::PI) / TWO_PI).floor() != ((b-1.5*std::f64::consts::PI) / TWO_PI).floor() {
        c = -1.0;
    } else {
        c = f64::min(a.sin(), b.sin());
    }

    if ((a-0.5*std::f64::consts::PI) / TWO_PI).floor() != ((b-0.5*std::f64::consts::PI) / TWO_PI).floor() {
        d = 1.0;
    } else {
        d = f64::max(a.sin(), b.sin());
    }

    new_interval(c, d)
}

pub fn cos_interval(i: Interval) -> Interval {
    let a: f64 = i.min;
    let b: f64 = i.max;
    let c: f64;
    let d: f64;

    if ((a+std::f64::consts::PI) / TWO_PI).floor() != ((b+std::f64::consts::PI) / TWO_PI).floor() {
        c = -1.0;
    } else {
        c = f64::min(a.cos(), b.cos());
    }

    if (a / TWO_PI).floor() != (b / TWO_PI).floor() {
        d = 1.0;
    } else {
        d = f64::max(a.cos(), b.cos());
    }

    new_interval(c, d)
}

#[cfg(test)]
mod tests {

    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    #[test]
    fn test_new_interval() {
        let i = new_interval(0.0, 1.0);
        assert_eq!(i.min, 0.0);
        assert_eq!(i.max, 1.0);
    }

    #[test]
    fn test_new_interval_v() {
        let i = new_interval_v(1.0);
        assert_eq!(i.min, 1.0);
        assert_eq!(i.max, 1.0);
    }

    #[test]
    fn test_add_interval() {
        let i = new_interval(0.0, 1.0);
        let j = new_interval(1.0, 2.0);
        let k = add_interval(i, j);
        assert_eq!(k.min, 1.0);
        assert_eq!(k.max, 3.0);
    }

    #[test]
    fn test_sub_interval() {
        let i = new_interval(0.0, 1.0);
        let j = new_interval(1.0, 2.0);
        let k = sub_interval(i, j);
        assert_eq!(k.min, -2.0);
        assert_eq!(k.max, 0.0);
    }

    #[test]
    fn test_mul_interval() {
        let i = new_interval(0.0, 1.0);
        let j = new_interval(1.0, 2.0);
        let k = mul_interval(i, j);
        assert_eq!(k.min, 0.0);
        assert_eq!(k.max, 2.0);
    }

    #[test]
    fn test_div_interval() {
        let i = new_interval(0.0, 1.0);
        let j = new_interval(1.0, 2.0);
        let k = div_interval(i, j);
        assert_eq!(k.min, 0.0);
        assert_eq!(k.max, 1.0);
    }

    #[test]
    fn test_pow_interval() {
        let i = new_interval(0.0, 2.0);
        let k = pow_interval(i, 2);
        assert_eq!(k.min, 0.0);
        assert_eq!(k.max, 4.0);
        let i = new_interval(0.0, 2.0);
        let k = pow_interval(i, 3);
        assert_eq!(k.min, 0.0);
        assert_eq!(k.max, 8.0);
    }

    #[test]
    fn test_sin_interval() {
        let i = new_interval(TWO_PI, TWO_PI);
        let k = sin_interval(i);
        assert!((k.min - 0.0).abs() < 1e-10);
        assert!((k.max - 0.0).abs() < 1e-10);
        let i = new_interval(0.0, 5.0);
        let k = sin_interval(i);
        assert_eq!(k.min, -1.0);
        assert_eq!(k.max, 1.0);
    }

    #[test]
    fn test_cos_interval() {
        let i = new_interval(TWO_PI, TWO_PI);
        let k = cos_interval(i);
        assert!((k.min - 1.0).abs() < 1e-10);
        assert!((k.max - 1.0).abs() < 1e-10);
        let i = new_interval(0.0, 5.0);
        let k = cos_interval(i);
        assert_eq!(k.min, -1.0);
        assert_eq!(k.max, 1.0);
    }
}