pub struct Cordic;

impl Cordic {
    pub const fn new() -> Self {
        Self
    }

    pub fn sin_cos(&mut self, theta: f32) -> (f32, f32) {
        (libm::sinf(theta), libm::cosf(theta))
    }
}

impl Default for Cordic {
    fn default() -> Self {
        Self::new()
    }
}
