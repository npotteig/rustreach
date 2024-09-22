use super::geometry::HyperRectangle;
pub trait SystemModel<const NUM_DIMS: usize> {
    fn get_derivative_bounds(
        &self,
        rect: &HyperRectangle<NUM_DIMS>,
        face_index: usize,
        ctrl_inputs: &Vec<f64>,
    ) -> f64;

    fn num_dims(&self) -> usize {
        NUM_DIMS
    }

    fn num_faces(&self) -> usize {
        2 * NUM_DIMS
    }
}