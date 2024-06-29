use nalgebra::{ArrayStorage, Vector, Vector2, Vector3, U2, U3};

use crate::interface::Vectorizable;

pub struct Point {
    pub x: f32,
    pub y: f32,
}

pub struct Line {
    pub a: f32,
    pub b: f32,
    pub c: f32,
}

impl Point {
    pub fn new(x: f32, y: f32) -> Self {
        Point { x, y }
    }
}

impl Line {
    pub fn new(a: f32, b: f32, c: f32) -> Self {
        Line { a, b, c }
    }

    pub fn from_points(p1: &Point, p2: &Point) -> Line {
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
