use crate::{
    geo::{Line, Point},
    interface::{Matrix2x9, WithRestriction},
};

use nalgebra::{ArrayStorage, Matrix, U1, U9};

pub struct PointPair {
    pub p1: Point,
    pub p2: Point,
}

pub struct LinePair {
    pub l1: Line,
    pub l2: Line,
}

type RowVector9<T> = Matrix<T, U1, U9, ArrayStorage<T, 1, 9>>;

impl WithRestriction<f32> for PointPair {
    fn generate_restriction(&self) -> Matrix2x9<f32> {
        let p1 = &self.p1;
        let p2 = &self.p2;

        let first_row = RowVector9::from_vec(vec![
            0.,
            0.,
            0.,
            -p1.x,
            -p1.y,
            -1.,
            p1.x * p2.y,
            p1.y * p2.y,
            p2.y,
        ]);

        let second_row = RowVector9::from_vec(vec![
            p1.x,
            p1.y,
            1.,
            0.,
            0.,
            0.,
            -p1.x * p2.x,
            -p1.y * p2.x,
            -p2.x,
        ]);

        return Matrix2x9::from_rows(&vec![first_row, second_row]);
    }
}

impl WithRestriction<f32> for LinePair {
    fn generate_restriction(&self) -> Matrix2x9<f32> {
        let l1 = &self.l1;
        let l2 = &self.l2;

        let first_row = RowVector9::from_vec(vec![
            0.,
            -l1.c * l2.a,
            l1.b * l2.a,
            0.,
            -l1.c * l2.b,
            l1.b * l2.b,
            0.,
            -l1.c * l2.c,
            l1.b * l2.c,
        ]);

        let second_row = RowVector9::from_vec(vec![
            l1.c * l2.a,
            0.,
            -l1.a * l2.a,
            l1.c * l2.b,
            0.,
            -l1.a * l2.b,
            l1.c * l2.c,
            0.,
            -l1.a * l2.c,
        ]);

        return Matrix2x9::from_rows(&vec![first_row, second_row]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_generate_restriction_point_pair() {
        let p1 = Point::new(1., 2.);
        let p2 = Point::new(3., 4.);
        let pair = PointPair { p1, p2 };

        let _restriction = pair.generate_restriction();
    }

    #[test]
    fn test_generate_restriction_line_pair() {
        let l1 = Line::new(1., 2., 3.);
        let l2 = Line::new(4., 5., 6.);
        let pair = LinePair { l1, l2 };

        let _restriction = pair.generate_restriction();
    }
}
