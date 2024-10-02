#[derive(Copy, Clone)]
pub struct Interval {
    pub min: f64, 
    pub max: f64,
}

impl Default for Interval {
    fn default() -> Self {
        Interval {
            min: 0.0, // or some other default value
            max: 0.0, // or some other default value
        }
    }
}

pub struct HyperPoint<const NUM_DIMS: usize> {
    dims: [f64; NUM_DIMS],  
}

#[derive(Copy, Clone)]
pub struct HyperRectangle<const NUM_DIMS: usize> {
    pub dims: [Interval; NUM_DIMS],
}

impl<const NUM_DIMS: usize> Default for HyperRectangle<NUM_DIMS> {
    fn default() -> Self {
        HyperRectangle {
            dims: [Interval::default(); NUM_DIMS], // Initialize array with default Intervals
        }
    }
}

impl<const NUM_DIMS: usize> HyperRectangle<NUM_DIMS> {
    pub fn num_dims(&self) -> usize {
        NUM_DIMS
    }
}

pub fn interval_width(i: &Interval) -> f64 {
    i.max - i.min
}

pub fn hyperrectangle_max_width<const NUM_DIMS: usize>(rect: &HyperRectangle<NUM_DIMS>) -> f64 {
    let mut rv: f64 = 0.0;

    for dim in &rect.dims {
        let min: f64 = dim.min;
        let max: f64 = dim.max;
        let dif: f64 = max - min;

        if !min.is_finite() || !max.is_finite() || !dif.is_finite() {
            rv = f64::MAX;
            break;
        }

        if dif > rv {
            rv = dif;
        }
    }

    rv
}

pub fn hyperrectangle_contains<const NUM_DIMS: usize>(outside: &HyperRectangle<NUM_DIMS>, inside: &HyperRectangle<NUM_DIMS>, print_errors: bool) -> bool {
    let mut rv = true;

    for d in 0..NUM_DIMS {
        let outside_dim = &outside.dims[d];
        let inside_dim = &inside.dims[d];

        if inside_dim.min < outside_dim.min || inside_dim.max > outside_dim.max {
            if print_errors {
                if inside_dim.min < outside_dim.min {
                    println!(
                        "inside->dim[{}].min ({}) < outside->dim[{}].min ({})",
                        d, inside_dim.min, d, outside_dim.min
                    );
                } else {
                    println!(
                        "inside->dim[{}].max ({}) > outside->dim[{}].max ({})",
                        d, inside_dim.max, d, outside_dim.max
                    );
                }
            }
            rv = false;
            break;
        }
    }

    rv
}

pub fn hyperrectangle_grow_to_convex_hull<const NUM_DIMS: usize>(grower: &mut HyperRectangle<NUM_DIMS>, contained: &HyperRectangle<NUM_DIMS>) {

    for d in 0..NUM_DIMS {
        let grower_dim: &mut Interval = &mut grower.dims[d];
        let contained_dim: &Interval = &contained.dims[d];

        if contained_dim.min < grower_dim.min {
            grower_dim.min = contained_dim.min;
        }

        if contained_dim.max > grower_dim.max {
            grower_dim.max = contained_dim.max;
        }
    }
}

pub fn print<const NUM_DIMS: usize>(hyperrectangle: &HyperRectangle<NUM_DIMS>) {
    // Print the start of the HyperRectangle representation
    print!("[HyperRectangle");

    // Iterate over dimensions and print each dimension's min and max
    for dim in &hyperrectangle.dims {
        print!(" ({}, {})", dim.min, dim.max);
    }

    // Print the end of the HyperRectangle representation
    println!("]");
}

pub fn println<const NUM_DIMS: usize>(hyperrectangle: &HyperRectangle<NUM_DIMS>) {
    print(hyperrectangle);
    println!();
}

pub fn hyperrectangle_bloat<const NUM_DIMS: usize>(out: &mut HyperRectangle<NUM_DIMS>, from: [f64; NUM_DIMS], width: f64) {
    for d in 0..NUM_DIMS {
        out.dims[d].min = from[d] - width;
        out.dims[d].max = from[d] + width;
    }
}

#[cfg(test)]
mod tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    #[test]
    fn test_interval_width() {
        let i = Interval { min: 0.0, max: 1.0 };
        assert_eq!(interval_width(&i), 1.0);
    }

    #[test]
    fn test_hyperrectangle_max_width() {
        let r = HyperRectangle::<4> {
            dims: [
                Interval { min: 0.0, max: 1.0 },
                Interval { min: 0.0, max: 1.0 },
                Interval { min: 0.0, max: 1.0 },
                Interval { min: 0.0, max: 1.0 },
            ],
        };
        assert_eq!(hyperrectangle_max_width(&r), 1.0);
    }

    #[test]
    fn test_hyperrectangle_contains() {
        let outside = HyperRectangle::<4> {
            dims: [
                Interval { min: 0.0, max: 1.0 },
                Interval { min: 0.0, max: 1.0 },
                Interval { min: 0.0, max: 1.0 },
                Interval { min: 0.0, max: 1.0 },
            ],
        };
        let inside = HyperRectangle::<4> {
            dims: [
                Interval { min: 0.1, max: 0.9 },
                Interval { min: 0.1, max: 0.9 },
                Interval { min: 0.1, max: 0.9 },
                Interval { min: 0.1, max: 0.9 },
            ],
        };
        assert_eq!(hyperrectangle_contains(&outside, &inside, false), true);
        assert_eq!(hyperrectangle_contains(&inside, &outside, false), false);
    }

    #[test]
    fn test_hyperrectangle_grow_to_convex_hull() {
        let mut grower = HyperRectangle::<4> {
            dims: [
                Interval { min: 0.0, max: 1.0 },
                Interval { min: 0.0, max: 1.0 },
                Interval { min: 0.0, max: 1.0 },
                Interval { min: 0.0, max: 1.0 },
            ],
        };
        let contained = HyperRectangle::<4> {
            dims: [
                Interval { min: -1.0, max: 2.0 },
                Interval { min: -1.0, max: 2.0 },
                Interval { min: -1.0, max: 2.0 },
                Interval { min: -1.0, max: 2.0 },
            ],
        };
        hyperrectangle_grow_to_convex_hull(&mut grower, &contained);
        for d in 0..grower.num_dims() {
            assert_eq!(grower.dims[d].min, -1.0);
            assert_eq!(grower.dims[d].max, 2.0);
        }
    }

    #[test]
    fn test_hyperrectangle_bloat() {
        let mut out = HyperRectangle::<4> {
            dims: [
                Interval { min: 0.0, max: 0.0 },
                Interval { min: 0.0, max: 0.0 },
                Interval { min: 0.0, max: 0.0 },
                Interval { min: 0.0, max: 0.0 },
            ],
        };
        hyperrectangle_bloat(&mut out, [0.0, 0.0, 0.0, 0.0], 1.0);
        for d in 0..out.num_dims() {
            assert_eq!(out.dims[d].min, -1.0);
            assert_eq!(out.dims[d].max, 1.0);
        }
    }
}