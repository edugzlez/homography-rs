use nalgebra::{ArrayStorage, Vector, Vector2, Vector3, U2, U3};

use crate::interface::Vectorizable;

pub struct Point<T = f32> {
    pub x: T,
    pub y: T,
}

pub struct Line<T = f32> {
    pub a: T,
    pub b: T,
    pub c: T,
}

impl<T> Point<T> {
    pub fn new(x: T, y: T) -> Self {
        Point { x, y }
    }
}

impl<T: num_traits::Float> Line<T> {
    pub fn new(a: T, b: T, c: T) -> Self {
        Line { a, b, c }
    }

    pub fn from_points(p1: &Point<T>, p2: &Point<T>) -> Line<T> {
        let a = p2.y - p1.y;
        let b = p1.x - p2.x;
        let c = -a * p1.x - b * p1.y;
        Line { a, b, c }
    }
}

impl Vectorizable<f32, U3, ArrayStorage<f32, 3, 1>> for Line {
    fn to_vector(&self) -> Vector<f32, U3, ArrayStorage<f32, 3, 1>> {
        Vector3::new(self.a, self.b, self.c)
    }
}

impl Vectorizable<f32, U2, ArrayStorage<f32, 2, 1>> for Point {
    fn to_vector(&self) -> Vector<f32, U2, ArrayStorage<f32, 2, 1>> {
        Vector2::new(self.x, self.y)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_point() {
        let p = Point::new(1.0, 2.0);
        assert_eq!(p.x, 1.0);
        assert_eq!(p.y, 2.0);
    }

    #[test]
    fn test_create_line() {
        let l = Line::new(1.0, 2.0, 3.0);
        assert_eq!(l.a, 1.0);
        assert_eq!(l.b, 2.0);
        assert_eq!(l.c, 3.0);
    }

    #[test]
    fn test_line_from_points() {
        let p1 = Point::new(1.0, 2.0);
        let p2 = Point::new(3.0, 4.0);
        let l = Line::from_points(&p1, &p2);
        assert_eq!(l.a, 2.0);
        assert_eq!(l.b, -2.0);
        assert_eq!(l.c, 2.0);
    }

    #[test]
    fn test_point_to_vector() {
        let p = Point::new(1.0, 2.0);
        let v = p.to_vector();
        assert_eq!(v[0], 1.0);
        assert_eq!(v[1], 2.0);
    }

    #[test]
    fn test_line_to_vector() {
        let l = Line::new(1.0, 2.0, 3.0);
        let v = l.to_vector();
        assert_eq!(v[0], 1.0);
        assert_eq!(v[1], 2.0);
        assert_eq!(v[2], 3.0);
    }
}
